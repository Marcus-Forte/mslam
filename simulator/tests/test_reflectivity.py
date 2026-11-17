"""Reflectivity model: Livox intensity scale, incidence angle, datasheet detection ranges, scene materials."""

import numpy as np
import pytest

from msim.raycast import BoxCaster
from msim.reflectivity import RANGE_EXPONENT, ReflectivityModel
from msim.scene import DEFAULT_REFLECTIVITY, scene_from_dict
from msim.sensors import LidarModel

from test_sensors import const_kin, fill_history


class Along:
    """Every ray along one sensor-frame direction."""

    def __init__(self, d):
        self.d = np.asarray(d, float) / np.linalg.norm(d)

    def directions(self, scan_index, n):
        return np.tile(self.d, (n, 1))


def room(**extra):
    d = {"name": "r", "wall_height": 2.8, "start": {"position": [0, 0, 1]},
         "rooms": [{"polygon": [[-2, -2], [2, -2], [2, 2], [-2, 2]]}]}
    d.update(extra)
    return scene_from_dict(d)


def scan_along(scene, d, p=(0.0, 0.0, 1.0), model=None):
    h = fill_history(lambda t: const_kin(list(p)), 0.2)
    lidar = LidarModel(BoxCaster(scene), Along(d), reflectivity=model or ReflectivityModel.from_scene(scene))
    return lidar.scan(h, 10**12 + 50_000_000, 0)


def test_detection_range_matches_the_datasheet():
    m = ReflectivityModel.uniform(1)
    assert m.detection_range(0.1) == pytest.approx(40.0)
    assert m.detection_range(0.8) == pytest.approx(70.0)
    assert RANGE_EXPONENT == pytest.approx(0.2691, abs=1e-4)
    assert m.detection_range(0.0) == 0.0


def test_detection_cut_off_and_intensity_scale():
    m = ReflectivityModel(np.array([0.05, 0.1]), np.array([False, False]))
    r = np.array([35.0, 35.0, 39.9, 41.0])
    det, inten = m.apply(r, np.ones(4), np.array([0, 1, 1, 1]), np.random.default_rng(0))
    # 5 % reaches 40 * 0.5**0.269 = 33 m: missed at 35 m; 10 % reaches 40 m
    assert det.tolist() == [False, True, True, False]
    assert inten.tolist() == [0, 15, 15, 0]                       # 150 * 0.1


def test_perpendicular_wall_reads_150_times_reflectivity():
    s = scan_along(room(), [1, 0, 0])
    assert s.hit.all() and set(s.intensity.tolist()) == {round(150 * DEFAULT_REFLECTIVITY["wall"])}


def test_intensity_follows_the_cosine_of_incidence():
    a = np.radians(60)
    s = scan_along(room(), [np.cos(a), np.sin(a), 0.0])           # hits the +y wall at 30 deg from the normal
    assert set(s.intensity.tolist()) == {round(150 * DEFAULT_REFLECTIVITY["wall"] * np.cos(np.radians(30)))}


def test_scene_overrides_kind_defaults_and_per_item():
    sc = room(reflectivity={"wall": 0.2, "ceiling": 0.9})
    assert set(scan_along(sc, [1, 0, 0]).intensity.tolist()) == {30}
    assert set(scan_along(sc, [0, 0, 1]).intensity.tolist()) == {135}
    assert set(scan_along(sc, [0, 0, -1]).intensity.tolist()) == {round(150 * DEFAULT_REFLECTIVITY["floor"])}
    sc2 = room(boxes=[{"name": "dark", "center": [1.0, 0.0], "size": [0.2, 1.0, 2.0], "reflectivity": 0.04}])
    assert set(scan_along(sc2, [1, 0, 0]).intensity.tolist()) == {6}


def test_retroreflector_reads_151_to_255():
    sc = room(boxes=[{"name": "sign", "center": [1.0, 0.0], "size": [0.2, 1.0, 2.0], "retroreflective": True}])
    assert set(scan_along(sc, [1, 0, 0]).intensity.tolist()) == {255}
    a = np.radians(60)
    i = scan_along(sc, [np.cos(a), np.sin(a), 0.0], p=(0.5, -0.2, 1.0)).intensity
    assert np.all((i >= 151) & (i < 255))


def test_bad_reflectivity_is_rejected():
    with pytest.raises(ValueError):
        room(reflectivity={"wall": 1.5})
    with pytest.raises(ValueError):
        room(reflectivity={"window": 0.5})
    with pytest.raises(ValueError):
        room(boxes=[{"center": [0, 0], "size": [1, 1, 1], "reflectivity": -0.1}])


def test_intensity_noise_is_seeded_and_bounded():
    sc = room()
    m1 = ReflectivityModel.from_scene(sc, noise=0.2)
    m2 = ReflectivityModel.from_scene(sc, noise=0.2)
    a = scan_along(sc, [1, 0, 0], model=m1).intensity
    b = scan_along(sc, [1, 0, 0], model=m2).intensity
    assert np.array_equal(a, b)                                   # same seed in LidarModel's rng
    assert a.std() > 5 and a.max() <= 150 and abs(a.mean() - 60) < 3
