"""Generated gRPC/protobuf stubs for the msensor protos (package `sensors`).

The stubs are not checked in. `ensure_generated()` runs grpc_tools.protoc over
`third_party/msensor/proto` the first time they are needed (or run
`uv run msim gen-proto`). The generated modules import each other by bare name
(`import header_pb2`), so this directory is put on `sys.path`.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parents[2].parent / "third_party" / "msensor" / "proto"
PROTOS = ("header", "lidar", "imu", "recording")


def generated() -> bool:
    return all((HERE / f"{p}_pb2.py").exists() for p in PROTOS) and all(
        (HERE / f"{p}_pb2_grpc.py").exists() for p in ("lidar", "imu")
    )


def generate(force: bool = False) -> None:
    if generated() and not force:
        return
    if not PROTO_DIR.is_dir():
        raise FileNotFoundError(f"msensor proto directory not found: {PROTO_DIR}")
    cmd = [
        sys.executable, "-m", "grpc_tools.protoc",
        f"-I{PROTO_DIR}", f"--python_out={HERE}", f"--grpc_python_out={HERE}",
        *[str(PROTO_DIR / f"{p}.proto") for p in PROTOS],
    ]
    subprocess.run(cmd, check=True)


def ensure_generated() -> None:
    generate(force=False)
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
