from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def single_room():
    from msim.scene import load_scene
    return load_scene(ROOT / "scenes" / "single_room.yaml")


@pytest.fixture(scope="session")
def two_rooms():
    from msim.scene import load_scene
    return load_scene(ROOT / "scenes" / "two_rooms.yaml")
