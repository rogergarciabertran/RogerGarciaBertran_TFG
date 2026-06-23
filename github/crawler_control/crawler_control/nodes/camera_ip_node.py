#!/usr/bin/env python3

import math
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


# =========================
# CONFIG
# =========================
CAMERA_LEFT_URL = "rtsp://admin:sartisarti@192.168.1.13:554/stream1"
CAMERA_RIGHT_URL = "rtsp://admin:sartisarti@192.168.1.12:554/stream1"

INPUT_W =1920
INPUT_H =960

ROTATE_LEFT = None
ROTATE_RIGHT = None

OUT_W = 1920
OUT_H = 960

LEFT_CX_OFFSET = 55
LEFT_CY_OFFSET = -15
RIGHT_CX_OFFSET = 25
RIGHT_CY_OFFSET = 40
LEFT_RADIUS = 517
RIGHT_RADIUS = 502

FISHEYE_FOV_DEG = 180.0

CAMERA_YAW_LEFT_DEG = 180.0
CAMERA_YAW_RIGHT_DEG = 0.0

CAMERA_PITCH_LEFT_DEG = 0.0
CAMERA_ROLL_LEFT_DEG = 0.0
CAMERA_PITCH_RIGHT_DEG = 0.0
CAMERA_ROLL_RIGHT_DEG = 0.0

BLEND_WIDTH_DEG = 80.0

PAN_SHIFT_PX = -OUT_W // 4
ROTATE_FINAL_180 = True
FLIP_FINAL_HORIZONTAL = True

VIEW_W = 900
VIEW_H = 900

WINDOW_NAME = "ROS2 IP Camera Calib + 360"
SCREENSHOT_DIR = Path("/home/crawler/crawler_ws/plots")

# =========================
# STREAM A YOUTUBE
# =========================
STREAM_TO_YOUTUBE = True

# Mejor usar RTMPS
YOUTUBE_RTMP_URL = "rtmps://a.rtmps.youtube.com/live2"

# PON AQUÍ TU CLAVE REAL
YOUTUBE_STREAM_KEY = "e9ff-r9vj-ezvy-xgvw-80t9"

STREAM_FPS = 5
STREAM_BITRATE = "8000k"
STREAM_PRESET = "ultrafast"


@dataclass
class CamCalib:
    cx: int
    cy: int
    r: int
    step: int = 5


def rotate_frame(frame: np.ndarray, mode: Optional[str]) -> np.ndarray:
    if mode is None:
        return frame
    if mode == "cw":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if mode == "ccw":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if mode == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    raise ValueError(f"Modo de rotación no válido: {mode}")


def rotated_dims(mode: Optional[str], w: int, h: int) -> Tuple[int, int]:
    if mode in ("cw", "ccw"):
        return h, w
    return w, h


def rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)


def camera_rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    return rot_y(yaw_deg) @ rot_x(pitch_deg) @ rot_z(roll_deg)


def build_equirect_dirs(out_w: int, out_h: int) -> np.ndarray:
    xs = np.linspace(0, out_w - 1, out_w, dtype=np.float32)
    ys = np.linspace(0, out_h - 1, out_h, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys)

    lon = (xx / out_w) * (2.0 * np.pi) - np.pi
    lat = np.pi / 2.0 - (yy / out_h) * np.pi

    x = np.cos(lat) * np.sin(lon)
    y = np.sin(lat)
    z = np.cos(lat) * np.cos(lon)

    return np.stack([x, y, z], axis=-1).astype(np.float32)


def build_fisheye_map(
    out_w: int,
    out_h: int,
    src_w: int,
    src_h: int,
    cx: float,
    cy: float,
    radius: float,
    fov_deg: float,
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    dirs_world = build_equirect_dirs(out_w, out_h).reshape(-1, 3)

    r_cam_to_world = camera_rotation_matrix(yaw_deg, pitch_deg, roll_deg)
    r_world_to_cam = r_cam_to_world.T
    dirs_cam = dirs_world @ r_world_to_cam.T

    x = dirs_cam[:, 0]
    y = dirs_cam[:, 1]
    z = dirs_cam[:, 2]

    theta = np.arccos(np.clip(z, -1.0, 1.0))
    fov_rad = np.deg2rad(fov_deg)
    theta_max = fov_rad / 2.0

    valid = theta <= theta_max

    f = radius / theta_max
    r_img = f * theta

    denom = np.sqrt(x * x + y * y) + 1e-8
    u = cx + r_img * (x / denom)
    v = cy + r_img * (y / denom)

    valid &= (u >= 0) & (u < src_w) & (v >= 0) & (v < src_h)

    map_x = u.reshape(out_h, out_w).astype(np.float32)
    map_y = v.reshape(out_h, out_w).astype(np.float32)
    valid_mask = valid.reshape(out_h, out_w)

    return map_x, map_y, valid_mask


def angular_distance_deg(a_deg: np.ndarray, b_deg: float) -> np.ndarray:
    d = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return np.abs(d)


def build_blend_weights(
    out_w: int,
    out_h: int,
    yaw_left_deg: float,
    yaw_right_deg: float,
    blend_width_deg: float,
    valid_left: np.ndarray,
    valid_right: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    xs = np.linspace(0, out_w - 1, out_w, dtype=np.float32)
    lon_deg = (xs / out_w) * 360.0 - 180.0
    lon_deg = np.tile(lon_deg[None, :], (out_h, 1))

    d_left = angular_distance_deg(lon_deg, yaw_left_deg)
    d_right = angular_distance_deg(lon_deg, yaw_right_deg)

    w_left = np.zeros((out_h, out_w), dtype=np.float32)
    w_right = np.zeros((out_h, out_w), dtype=np.float32)

    only_left = valid_left & (~valid_right)
    only_right = valid_right & (~valid_left)
    both = valid_left & valid_right

    w_left[only_left] = 1.0
    w_right[only_right] = 1.0

    if np.any(both):
        dl = d_left[both]
        dr = d_right[both]

        total = dl + dr + 1e-8
        wl = 1.0 - (dl / total)
        wr = 1.0 - (dr / total)

        if blend_width_deg > 0:
            diff = np.abs(dl - dr)
            alpha = np.clip(diff / blend_width_deg, 0.0, 1.0)
            wl = 0.5 * (1.0 - alpha) + wl * alpha
            wr = 0.5 * (1.0 - alpha) + wr * alpha

        s = wl + wr + 1e-8
        wl /= s
        wr /= s

        w_left[both] = wl.astype(np.float32)
        w_right[both] = wr.astype(np.float32)

    return w_left, w_right


def render_fisheye_view(
    frame: np.ndarray,
    cx: int,
    cy: int,
    radius: int,
    out_w: int,
    out_h: int,
    title: str,
    selected: bool,
) -> np.ndarray:
    h, w = frame.shape[:2]
    x1 = max(0, cx - radius)
    y1 = max(0, cy - radius)
    x2 = min(w, cx + radius)
    y2 = min(h, cy + radius)

    crop = frame[y1:y2, x1:x2].copy()
    crop_h, crop_w = crop.shape[:2]

    mask = np.zeros((crop_h, crop_w), dtype=np.uint8)
    local_cx = cx - x1
    local_cy = cy - y1
    cv2.circle(mask, (local_cx, local_cy), radius, 255, -1)

    masked = cv2.bitwise_and(crop, crop, mask=mask)

    out = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    scale = min(out_w / max(1, crop_w), out_h / max(1, crop_h))
    new_w = max(1, int(crop_w * scale))
    new_h = max(1, int(crop_h * scale))
    resized = cv2.resize(masked, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    ox = (out_w - new_w) // 2
    oy = (out_h - new_h) // 2
    out[oy:oy + new_h, ox:ox + new_w] = resized

    approx_r = int(radius * scale)
    color = (0, 255, 255) if selected else (180, 180, 180)
    cv2.circle(out, (out_w // 2, out_h // 2), approx_r, color, 2)
    cv2.drawMarker(out, (out_w // 2, out_h // 2), color, cv2.MARKER_CROSS, 20, 2)
    cv2.putText(out, title, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
    return out


class FFmpegStreamer:
    def __init__(self, width: int, height: int, fps: int, rtmp_url: str, stream_key: str, bitrate: str, preset: str):
        self.width = width
        self.height = height
        self.fps = fps
        self.rtmp_url = rtmp_url.rstrip("/")
        self.stream_key = stream_key.strip()
        self.bitrate = bitrate
        self.preset = preset
        self.proc: Optional[subprocess.Popen] = None
        self.stderr_thread: Optional[threading.Thread] = None
        self.frames_sent = 0

    @property
    def output_url(self) -> str:
        return f"{self.rtmp_url}/{self.stream_key}"

    def _read_stderr(self) -> None:
        if self.proc is None or self.proc.stderr is None:
            return

        for line in iter(self.proc.stderr.readline, b""):
            if not line:
                break
            try:
                print("[FFMPEG]", line.decode("utf-8", errors="replace").rstrip())
            except Exception:
                pass

    def start(self) -> None:
        if not STREAM_TO_YOUTUBE:
            print("[INFO] STREAM_TO_YOUTUBE=False")
            return

        if not self.stream_key or self.stream_key == "PON_AQUI_TU_STREAM_KEY":
            raise RuntimeError("Debes poner una stream key válida en YOUTUBE_STREAM_KEY")

        if self.proc is not None and self.proc.poll() is None:
            print("[INFO] FFmpeg ya está en marcha.")
            return

        cmd = [
            "ffmpeg",
            "-loglevel", "warning",
            "-re",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "-",
            "-f", "lavfi",
            "-i", "anullsrc=channel_layout=stereo:sample_rate=44100",
            "-shortest",
            "-c:v", "libx264",
            "-preset", self.preset,
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-g", str(self.fps * 2),
            "-b:v", self.bitrate,
            "-maxrate", self.bitrate,
            "-bufsize", "8M",
            "-c:a", "aac",
            "-b:a", "128k",
            "-ar", "44100",
            "-f", "flv",
            self.output_url,
        ]

        print("[INFO] Lanzando FFmpeg:")
        print(" ".join(cmd[:-1] + ["<YOUTUBE_URL_OCULTA>"]))

        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        self.stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
        self.stderr_thread.start()

        time.sleep(1.0)
        if self.proc.poll() is not None:
            raise RuntimeError("FFmpeg se ha cerrado nada más arrancar. Mira las líneas [FFMPEG] en consola.")

        print("[INFO] FFmpeg arrancado correctamente.")

    def write(self, frame: np.ndarray) -> None:
        if self.proc is None or self.proc.stdin is None:
            return

        if self.proc.poll() is not None:
            print("[ERROR] FFmpeg terminó inesperadamente.")
            return

        try:
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

            if not frame.flags["C_CONTIGUOUS"]:
                frame = np.ascontiguousarray(frame)

            self.proc.stdin.write(frame.tobytes())
            self.frames_sent += 1

            if self.frames_sent % (self.fps * 5) == 0:
                print(f"[INFO] Frames enviados a FFmpeg: {self.frames_sent}")

        except BrokenPipeError:
            print("[ERROR] BrokenPipeError: FFmpeg cerró la entrada.")
        except Exception as e:
            print(f"[ERROR] No se pudo escribir frame a FFmpeg: {e}")

    def stop(self) -> None:
        if self.proc is not None:
            try:
                if self.proc.stdin is not None:
                    self.proc.stdin.close()
            except Exception:
                pass

            try:
                self.proc.terminate()
                self.proc.wait(timeout=3)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass

            self.proc = None
            print("[INFO] FFmpeg detenido.")


class IPCameraReader:
    def __init__(self, stream_url: str, name: str):
        self.stream_url = stream_url
        self.name = name
        self.cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        self.close()

        if self.stream_url.lower().startswith("rtsp://"):
            self.cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            print(f"[{self.name}] No se pudo abrir el stream IP: {self.stream_url}")
            return False

        print(f"[{self.name}] Stream IP abierto correctamente.")
        return True

    def read(self) -> Optional[np.ndarray]:
        if self.cap is None:
            return None

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None

        if frame.shape[1] != INPUT_W or frame.shape[0] != INPUT_H:
            frame = cv2.resize(frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)

        return frame

    def reopen(self) -> bool:
        self.close()
        time.sleep(1.0)
        return self.open()

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class RawCaptureWorker:
    def __init__(self, reader: IPCameraReader, name: str, rotation_mode: Optional[str]):
        self.reader = reader
        self.name = name
        self.rotation_mode = rotation_mode

        self.frame_rot: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.frames_ok = 0
        self.frames_fail = 0
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        while self.running:
            frame = self.reader.read()

            if frame is None:
                self.frames_fail += 1
                if not self.reader.reopen():
                    time.sleep(2.0)
                continue

            frame = rotate_frame(frame, self.rotation_mode)

            with self.lock:
                self.frame_rot = frame
                self.frames_ok += 1

    def get(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.frame_rot is None:
                return None
            return self.frame_rot.copy()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)


class IPCalibAndStitchNode(Node):
    def __init__(self) -> None:
        super().__init__("ip_calib_and_stitch_node")

        self.mode = "CALIB"
        self.selected = "LEFT"

        src_w_left, src_h_left = rotated_dims(ROTATE_LEFT, INPUT_W, INPUT_H)
        src_w_right, src_h_right = rotated_dims(ROTATE_RIGHT, INPUT_W, INPUT_H)

        self.left_src_w = src_w_left
        self.left_src_h = src_h_left
        self.right_src_w = src_w_right
        self.right_src_h = src_h_right

        self.left = CamCalib(
            cx=src_w_left // 2 + LEFT_CX_OFFSET,
            cy=src_h_left // 2 + LEFT_CY_OFFSET,
            r=LEFT_RADIUS,
            step=5,
        )
        self.right = CamCalib(
            cx=src_w_right // 2 + RIGHT_CX_OFFSET,
            cy=src_h_right // 2 + RIGHT_CY_OFFSET,
            r=RIGHT_RADIUS,
            step=5,
        )

        self.map_x_left = None
        self.map_y_left = None
        self.map_x_right = None
        self.map_y_right = None
        self.w_left_3 = None
        self.w_right_3 = None

        self.reader_left = IPCameraReader(CAMERA_LEFT_URL, "LEFT")
        self.reader_right = IPCameraReader(CAMERA_RIGHT_URL, "RIGHT")

        ok_left = self.reader_left.open()
        ok_right = self.reader_right.open()
        if not ok_left and not ok_right:
            raise RuntimeError("No se pudo abrir ninguna de las dos cámaras IP")

        self.worker_left = RawCaptureWorker(self.reader_left, "LEFT", ROTATE_LEFT) if ok_left else None
        self.worker_right = RawCaptureWorker(self.reader_right, "RIGHT", ROTATE_RIGHT) if ok_right else None

        if self.worker_left is not None:
            self.worker_left.start()
        if self.worker_right is not None:
            self.worker_right.start()

        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)

        self.streamer = FFmpegStreamer(
            width=OUT_W,
            height=OUT_H,
            fps=STREAM_FPS,
            rtmp_url=YOUTUBE_RTMP_URL,
            stream_key=YOUTUBE_STREAM_KEY,
            bitrate=STREAM_BITRATE,
            preset=STREAM_PRESET,
        )

        self.timer = self.create_timer(1.0 / STREAM_FPS, self.loop)
        self.last_log = time.time()

        self.get_logger().info("Nodo iniciado en modo CALIB.")
        self.get_logger().info(
            "TAB selecciona LEFT/RIGHT | WASD mueve | Q/E radio | I/K paso | "
            "M cambia CALIB/STITCH | P imprime | C captura | X o ESC sale"
        )

    def rebuild_stitch_maps(self) -> None:
        self.get_logger().info("Reconstruyendo mapas de stitch...")

        self.map_x_left, self.map_y_left, valid_left = build_fisheye_map(
            out_w=OUT_W,
            out_h=OUT_H,
            src_w=self.left_src_w,
            src_h=self.left_src_h,
            cx=self.left.cx,
            cy=self.left.cy,
            radius=self.left.r,
            fov_deg=FISHEYE_FOV_DEG,
            yaw_deg=CAMERA_YAW_LEFT_DEG,
            pitch_deg=CAMERA_PITCH_LEFT_DEG,
            roll_deg=CAMERA_ROLL_LEFT_DEG,
        )

        self.map_x_right, self.map_y_right, valid_right = build_fisheye_map(
            out_w=OUT_W,
            out_h=OUT_H,
            src_w=self.right_src_w,
            src_h=self.right_src_h,
            cx=self.right.cx,
            cy=self.right.cy,
            radius=self.right.r,
            fov_deg=FISHEYE_FOV_DEG,
            yaw_deg=CAMERA_YAW_RIGHT_DEG,
            pitch_deg=CAMERA_PITCH_RIGHT_DEG,
            roll_deg=CAMERA_ROLL_RIGHT_DEG,
        )

        w_left, w_right = build_blend_weights(
            out_w=OUT_W,
            out_h=OUT_H,
            yaw_left_deg=CAMERA_YAW_LEFT_DEG,
            yaw_right_deg=CAMERA_YAW_RIGHT_DEG,
            blend_width_deg=BLEND_WIDTH_DEG,
            valid_left=valid_left,
            valid_right=valid_right,
        )

        self.w_left_3 = w_left[..., None]
        self.w_right_3 = w_right[..., None]

        self.get_logger().info("Mapas listos.")

    def save_screenshot(self, img: np.ndarray) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = SCREENSHOT_DIR / f"captura_360_{timestamp}.png"
        cv2.imwrite(str(filename), img)
        self.get_logger().info(f"Captura guardada en: {filename}")

    def print_values(self) -> None:
        print("\n=== VALORES ACTUALES ===")
        print(f"ROTATE_LEFT  = {ROTATE_LEFT!r}")
        print(f"ROTATE_RIGHT = {ROTATE_RIGHT!r}")
        print(f"LEFT_CX_OFFSET  = {self.left.cx - self.left_src_w // 2}")
        print(f"LEFT_CY_OFFSET  = {self.left.cy - self.left_src_h // 2}")
        print(f"RIGHT_CX_OFFSET = {self.right.cx - self.right_src_w // 2}")
        print(f"RIGHT_CY_OFFSET = {self.right.cy - self.right_src_h // 2}")
        print(f"LEFT_RADIUS  = {self.left.r}")
        print(f"RIGHT_RADIUS = {self.right.r}")
        print("========================\n")

    def render_calib(self, frame_left: Optional[np.ndarray], frame_right: Optional[np.ndarray]) -> np.ndarray:
        if frame_left is None:
            left_view = np.zeros((VIEW_H, VIEW_W, 3), dtype=np.uint8)
        else:
            left_view = render_fisheye_view(
                frame_left, self.left.cx, self.left.cy, self.left.r,
                VIEW_W, VIEW_H, "LEFT", self.selected == "LEFT"
            )

        if frame_right is None:
            right_view = np.zeros((VIEW_H, VIEW_W, 3), dtype=np.uint8)
        else:
            right_view = render_fisheye_view(
                frame_right, self.right.cx, self.right.cy, self.right.r,
                VIEW_W, VIEW_H, "RIGHT", self.selected == "RIGHT"
            )

        top = np.hstack((left_view, right_view))
        info = np.zeros((250, top.shape[1], 3), dtype=np.uint8)

        lines = [
            f"MODO: CALIB | editando {self.selected}",
            "TAB LEFT/RIGHT | WASD mueve centro | Q/E radio | I/K paso",
            "M genera 360 con estos valores y empieza stream | P imprime valores | C captura | X/ESC salir",
            f"LEFT  cx={self.left.cx} cy={self.left.cy} r={self.left.r} step={self.left.step}",
            f"RIGHT cx={self.right.cx} cy={self.right.cy} r={self.right.r} step={self.right.step}",
        ]

        y = 35
        for line in lines:
            cv2.putText(info, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
            y += 35

        return np.vstack((top, info))

    def render_stitch(self, frame_left: Optional[np.ndarray], frame_right: Optional[np.ndarray]) -> np.ndarray:
        if self.map_x_left is None:
            self.rebuild_stitch_maps()

        if frame_left is not None:
            eq_left = cv2.remap(
                frame_left,
                self.map_x_left,
                self.map_y_left,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            ).astype(np.float32)
        else:
            eq_left = np.zeros((OUT_H, OUT_W, 3), dtype=np.float32)

        if frame_right is not None:
            eq_right = cv2.remap(
                frame_right,
                self.map_x_right,
                self.map_y_right,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            ).astype(np.float32)
        else:
            eq_right = np.zeros((OUT_H, OUT_W, 3), dtype=np.float32)

        if frame_left is not None and frame_right is not None:
            pano_f = eq_left * self.w_left_3 + eq_right * self.w_right_3
        elif frame_left is not None:
            pano_f = eq_left
        elif frame_right is not None:
            pano_f = eq_right
        else:
            pano_f = np.zeros((OUT_H, OUT_W, 3), dtype=np.float32)

        pano = np.clip(pano_f, 0, 255).astype(np.uint8)

        if ROTATE_FINAL_180:
            pano = cv2.rotate(pano, cv2.ROTATE_180)

        pano = np.roll(pano, shift=PAN_SHIFT_PX, axis=1)

        if FLIP_FINAL_HORIZONTAL:
            pano = cv2.flip(pano, 1)

        txt = "MODO: STITCH | m volver a CALIB | c captura | x salir"
        cv2.putText(pano, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        return pano

    def handle_key(self, key: int, current_img: np.ndarray) -> None:
        cfg = self.left if self.selected == "LEFT" else self.right
        max_w = self.left_src_w if self.selected == "LEFT" else self.right_src_w
        max_h = self.left_src_h if self.selected == "LEFT" else self.right_src_h

        if key in (ord("x"), 27):
            raise KeyboardInterrupt

        if key == ord("c"):
            self.save_screenshot(current_img)
            return

        if key == ord("p"):
            self.print_values()
            return

        if key == ord("m"):
            if self.mode == "CALIB":
                self.get_logger().info("Pasando a modo STITCH...")
                self.rebuild_stitch_maps()

                if STREAM_TO_YOUTUBE and self.streamer.proc is None:
                    self.get_logger().info("Intentando arrancar FFmpeg streamer...")
                    self.streamer.start()
                    self.get_logger().info("FFmpeg streamer lanzado.")

                self.mode = "STITCH"
                self.get_logger().info("Cambiado a modo STITCH.")
            else:
                self.mode = "CALIB"
                self.get_logger().info("Cambiado a modo CALIB.")
            return

        if self.mode != "CALIB":
            return

        if key == 9:
            self.selected = "RIGHT" if self.selected == "LEFT" else "LEFT"
        elif key == ord("a"):
            cfg.cx -= cfg.step
        elif key == ord("d"):
            cfg.cx += cfg.step
        elif key == ord("w"):
            cfg.cy -= cfg.step
        elif key == ord("s"):
            cfg.cy += cfg.step
        elif key == ord("q"):
            cfg.r -= cfg.step
        elif key == ord("e"):
            cfg.r += cfg.step
        elif key == ord("i"):
            cfg.step = max(1, cfg.step - 1)
        elif key == ord("k"):
            cfg.step += 1

        cfg.cx = max(0, min(max_w - 1, cfg.cx))
        cfg.cy = max(0, min(max_h - 1, cfg.cy))
        cfg.r = max(50, min(min(max_w, max_h), cfg.r))

    def loop(self) -> None:
        frame_left = self.worker_left.get() if self.worker_left is not None else None
        frame_right = self.worker_right.get() if self.worker_right is not None else None

        if self.mode == "CALIB":
            img = self.render_calib(frame_left, frame_right)
        else:
            img = self.render_stitch(frame_left, frame_right)
            frame_out = cv2.resize(img, (OUT_W, OUT_H), interpolation=cv2.INTER_LINEAR)

            if STREAM_TO_YOUTUBE:
                self.streamer.write(frame_out)

            if STREAM_TO_YOUTUBE and self.streamer.proc is not None and self.streamer.proc.poll() is not None:
                self.get_logger().error("FFmpeg se ha cerrado.")

        cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self.handle_key(key, img)

        if time.time() - self.last_log > 5.0:
            l_ok = self.worker_left.frames_ok if self.worker_left is not None else 0
            l_fail = self.worker_left.frames_fail if self.worker_left is not None else 0
            r_ok = self.worker_right.frames_ok if self.worker_right is not None else 0
            r_fail = self.worker_right.frames_fail if self.worker_right is not None else 0
            self.get_logger().info(
                f"LEFT ok={l_ok} fail={l_fail} | RIGHT ok={r_ok} fail={r_fail} | mode={self.mode}"
            )
            self.last_log = time.time()

    def destroy_node(self):
        if self.worker_left is not None:
            self.worker_left.stop()
        if self.worker_right is not None:
            self.worker_right.stop()

        self.reader_left.close()
        self.reader_right.close()
        self.streamer.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IPCalibAndStitchNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
