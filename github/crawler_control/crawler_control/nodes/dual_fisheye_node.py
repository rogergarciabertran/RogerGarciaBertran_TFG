#!/usr/bin/env python3

import subprocess
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def generar_mapas(cx: int, cy: int, r: int, out_w: int, out_h: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Genera mapas de remapeo para convertir una imagen fisheye hemisférica
    a una vista rectangular aplanada.
    """
    theta = np.linspace(-np.pi / 2, np.pi / 2, out_w, dtype=np.float32)
    phi = np.linspace(-np.pi / 2, np.pi / 2, out_h, dtype=np.float32)
    theta, phi = np.meshgrid(theta, phi)

    x = np.cos(phi) * np.sin(theta)
    y = np.sin(phi)
    z = np.cos(phi) * np.cos(theta)

    angle = np.arccos(np.clip(z, -1.0, 1.0))
    radius = angle / (np.pi / 2) * r

    denom = np.sqrt(x**2 + y**2) + 1e-8
    x_img = cx + radius * (x / denom)
    y_img = cy + radius * (y / denom)

    return x_img.astype(np.float32), y_img.astype(np.float32)


class FFmpegReader:
    def __init__(self, url: str, width: int = 1920, height: int = 1080):
        self.url = url
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        cmd = [
            "ffmpeg",
            "-loglevel", "error",
            "-rtsp_transport", "tcp",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-i", self.url,
            "-vf", f"scale={self.width}:{self.height}",
            "-an",
            "-sn",
            "-dn",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]

        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**8
        )

    def read(self) -> Optional[np.ndarray]:
        if self.proc is None or self.proc.stdout is None:
            return None

        raw = self.proc.stdout.read(self.frame_size)
        if len(raw) != self.frame_size:
            return None

        frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
        return frame

    def stop(self) -> None:
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None


class FFmpegWriter:
    def __init__(self, output_path: Path, width: int, height: int, fps: float):
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        cmd = [
            "ffmpeg",
            "-y",
            "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", f"{self.fps}",
            "-i", "-",
            "-an",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-crf", "23",
            "-pix_fmt", "yuv420p",
            "-f", "matroska",
            str(self.output_path),
        ]

        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**8
        )

    def write(self, frame: np.ndarray) -> None:
        if self.proc is None or self.proc.stdin is None:
            return

        try:
            self.proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            pass

    def stop(self) -> None:
        if self.proc is not None:
            try:
                if self.proc.stdin is not None:
                    self.proc.stdin.close()
            except Exception:
                pass

            try:
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass

            self.proc = None


class DualFisheyeNode(Node):
    def __init__(self) -> None:
        super().__init__("dual_fisheye_node")

        self.declare_parameter("url_left", "rtsp://admin:sartisarti@192.168.1.12:554/stream1")
        self.declare_parameter("url_right", "rtsp://admin:sartisarti@192.168.1.13:554/stream1")

        self.declare_parameter("input_width", 1920)
        self.declare_parameter("input_height", 1080)

        self.declare_parameter("out_width", 960)
        self.declare_parameter("out_height", 1080)

        self.declare_parameter("left_cx_offset", 35)
        self.declare_parameter("left_cy_offset", -15)
        self.declare_parameter("right_cx_offset", 30)
        self.declare_parameter("right_cy_offset", 15)
        self.declare_parameter("radius_scale", 0.48)

        self.declare_parameter("publish_hz", 8.0)

        self.declare_parameter("record_video", True)
        self.declare_parameter("record_dir", str(Path.home() / "crawler_ws" / "logs" / "videos"))
        self.declare_parameter("record_fps", 8.0)
        self.declare_parameter("swap_order", False)

        self.url_left = str(self.get_parameter("url_left").value)
        self.url_right = str(self.get_parameter("url_right").value)

        self.in_w = int(self.get_parameter("input_width").value)
        self.in_h = int(self.get_parameter("input_height").value)

        self.out_w = int(self.get_parameter("out_width").value)
        self.out_h = int(self.get_parameter("out_height").value)

        self.left_cx_offset = int(self.get_parameter("left_cx_offset").value)
        self.left_cy_offset = int(self.get_parameter("left_cy_offset").value)
        self.right_cx_offset = int(self.get_parameter("right_cx_offset").value)
        self.right_cy_offset = int(self.get_parameter("right_cy_offset").value)
        self.radius_scale = float(self.get_parameter("radius_scale").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        if self.publish_hz <= 0.0:
            self.publish_hz = 8.0

        self.record_video = bool(self.get_parameter("record_video").value)
        self.record_dir = Path(str(self.get_parameter("record_dir").value))
        self.record_fps = float(self.get_parameter("record_fps").value)
        if self.record_fps <= 0.0:
            self.record_fps = self.publish_hz

        self.swap_order = bool(self.get_parameter("swap_order").value)

        self.bridge = CvBridge()

        self.pub_left = self.create_publisher(Image, "/camera360/flat_left", 10)
        self.pub_right = self.create_publisher(Image, "/camera360/flat_right", 10)
        self.pub_dual = self.create_publisher(Image, "/camera360/dual_flat", 10)

        cx1 = self.in_w // 2 + self.left_cx_offset
        cy1 = self.in_h // 2 + self.left_cy_offset
        r1 = int(self.in_h * self.radius_scale)

        cx2 = self.in_w // 2 + self.right_cx_offset
        cy2 = self.in_h // 2 + self.right_cy_offset
        r2 = int(self.in_h * self.radius_scale)

        self.map_x1, self.map_y1 = generar_mapas(cx1, cy1, r1, self.out_w, self.out_h)
        self.map_x2, self.map_y2 = generar_mapas(cx2, cy2, r2, self.out_w, self.out_h)

        self.get_logger().info(
            f"Left map: cx={cx1}, cy={cy1}, r={r1} | Right map: cx={cx2}, cy={cy2}, r={r2}"
        )

        self.reader_left = FFmpegReader(self.url_left, self.in_w, self.in_h)
        self.reader_right = FFmpegReader(self.url_right, self.in_w, self.in_h)

        self.reader_left.start()
        self.reader_right.start()

        self.writer: Optional[FFmpegWriter] = None
        self.video_path: Optional[Path] = None

        if self.record_video:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.video_path = self.record_dir / f"dual360_{timestamp}.mkv"
            self.writer = FFmpegWriter(
                output_path=self.video_path,
                width=self.out_w * 2,
                height=self.out_h,
                fps=self.record_fps,
            )
            self.writer.start()
            self.get_logger().info(f"Grabando vídeo en: {self.video_path}")

        self._lock = threading.Lock()
        self.frame_flat1: Optional[np.ndarray] = None
        self.frame_flat2: Optional[np.ndarray] = None
        self.left_frames = 0
        self.right_frames = 0

        self._running = True

        self.thread_left = threading.Thread(
            target=self._capture_thread,
            args=(self.reader_left, self.map_x1, self.map_y1, "left"),
            daemon=True,
        )
        self.thread_right = threading.Thread(
            target=self._capture_thread,
            args=(self.reader_right, self.map_x2, self.map_y2, "right"),
            daemon=True,
        )

        self.thread_left.start()
        self.thread_right.start()

        self.timer = self.create_timer(1.0 / self.publish_hz, self._publish_callback)
        self.log_timer = self.create_timer(5.0, self._debug_log)

        self.get_logger().info("Dual fisheye node started with FFmpeg readers and FFmpeg writer")

    def _capture_thread(
        self,
        reader: FFmpegReader,
        map_x: np.ndarray,
        map_y: np.ndarray,
        side: str,
    ) -> None:
        while self._running:
            frame = reader.read()
            if frame is None:
                continue

            remap = cv2.remap(
                frame,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )

            with self._lock:
                if side == "left":
                    self.frame_flat1 = remap
                    self.left_frames += 1
                else:
                    self.frame_flat2 = remap
                    self.right_frames += 1

    def _publish_callback(self) -> None:
        with self._lock:
            if self.frame_flat1 is None or self.frame_flat2 is None:
                return

            left = self.frame_flat1.copy()
            right = self.frame_flat2.copy()

        if self.swap_order:
            dual = np.hstack((right, left))
        else:
            dual = np.hstack((left, right))

        if self.writer is not None:
            self.writer.write(dual)

        stamp = self.get_clock().now().to_msg()

        msg_left = self.bridge.cv2_to_imgmsg(left, encoding="bgr8")
        msg_left.header.stamp = stamp
        msg_left.header.frame_id = "camera360_left_flat_frame"

        msg_right = self.bridge.cv2_to_imgmsg(right, encoding="bgr8")
        msg_right.header.stamp = stamp
        msg_right.header.frame_id = "camera360_right_flat_frame"

        msg_dual = self.bridge.cv2_to_imgmsg(dual, encoding="bgr8")
        msg_dual.header.stamp = stamp
        msg_dual.header.frame_id = "camera360_dual_flat_frame"

        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)
        self.pub_dual.publish(msg_dual)

    def _debug_log(self) -> None:
        with self._lock:
            l = self.left_frames
            r = self.right_frames
        self.get_logger().info(f"Frames recibidos -> left: {l}, right: {r}")

    def destroy_node(self) -> None:
        self._running = False

        try:
            self.reader_left.stop()
        except Exception:
            pass

        try:
            self.reader_right.stop()
        except Exception:
            pass

        try:
            if self.writer is not None:
                self.writer.stop()
                self.get_logger().info(f"Vídeo guardado en: {self.video_path}")
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = DualFisheyeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"Fatal error: {exc}")
        else:
            print(f"Fatal error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()