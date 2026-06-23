import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def load_json_config(filename: str) -> dict:
    # 1) Ruta estándar ROS2: install/.../share/<pkg>/config/<file>
    candidates = []
    try:
        share_dir = Path(get_package_share_directory("crawler_control"))
        candidates.append(share_dir / "config" / filename)
    except Exception:
        pass

    # 2) Tu estructura actual dentro del paquete python
    #    .../crawler_control/core/config_loader.py -> .../crawler_control/config/<file>
    here = Path(__file__).resolve()
    candidates.append(here.parents[1] / "config" / filename)

    # 3) Ruta relativa (por si ejecutas desde scripts)
    candidates.append(Path.cwd() / filename)
    candidates.append(Path.cwd() / "config" / filename)

    for path in candidates:
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)

    tried = "\n".join([f" - {p}" for p in candidates])
    raise FileNotFoundError(
        f"No se encuentra el fichero de config: {filename}\n"
        f"Probado en:\n{tried}"
    )
