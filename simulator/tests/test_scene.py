import mujoco
import numpy as np
import pytest

from msim.raycast import BoxCaster, MujocoCaster
from msim.scene import scene_from_dict

AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "down": (0, 0, -1), "up": (0, 0, 1)}
# single_room: inner walls +-1.9, floor z = 0, ceiling inner face z = 2.8, sensor at z = 1
EXPECTED = {"+x": 1.9, "-x": 1.9, "+y": 1.9, "-y": 1.9, "down": 1.0, "up": 1.8}


@pytest.mark.parametrize("caster_cls", [BoxCaster, MujocoCaster])
def test_known_room_distances(single_room, caster_cls):
    c = caster_cls(single_room)
    names = list(AXES)
    o = np.tile([0.0, 0.0, 1.0], (len(names), 1))
    d = np.array([AXES[n] for n in names], float)
    r, _, _ = c.cast(o, d, 0.1, 40.0)
    for n, v in zip(names, r):
        assert v == pytest.approx(EXPECTED[n], abs=1e-9), n


def test_diagonal_and_incidence(single_room):
    c = BoxCaster(single_room)
    d = np.array([[np.cos(np.pi / 6), np.sin(np.pi / 6), 0.0]])
    r, cos, _ = c.cast(np.array([[0.0, 0.0, 1.0]]), d, 0.1, 40.0)
    assert r[0] == pytest.approx(1.9 / np.cos(np.pi / 6))
    assert cos[0] == pytest.approx(np.cos(np.pi / 6))


def test_range_limits(single_room):
    c = BoxCaster(single_room)
    o, d = np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 0.0, 0.0]])
    assert np.isinf(c.cast(o, d, 0.1, 1.5)[0][0])                # beyond max range: miss
    assert np.isinf(c.cast(o, d, 2.0, 40.0)[0][0])               # nearer than min range: miss


def test_box_and_mujoco_agree_on_random_rays(two_rooms):
    rng = np.random.default_rng(3)
    n = 3000
    o = np.column_stack([rng.uniform(-3.5, 9.5, n), rng.uniform(-2.5, 2.5, n), rng.uniform(0.3, 2.5, n)])
    # an origin inside a solid is outside the contract (MuJoCo and the slab test treat it differently)
    o = o[[not two_rooms.collides(p, 1e-6) for p in o]]
    n = len(o)
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    a, ca, ia = BoxCaster(two_rooms).cast(o, d, 0.1, 40.0)
    b, cb, ib = MujocoCaster(two_rooms).cast(o, d, 0.1, 40.0)
    assert n > 2500
    assert np.array_equal(np.isfinite(a), np.isfinite(b))
    both = np.isfinite(a)
    assert np.abs(a[both] - b[both]).max() < 1e-6
    assert np.array_equal(ia, ib)                                 # same surface hit
    assert np.abs(ca[both] - cb[both]).max() < 1e-6               # same incidence angle


def test_mjcf_loads_and_counts(two_rooms):
    model = mujoco.MjModel.from_xml_string(two_rooms.to_mjcf())
    assert model.ngeom == len(two_rooms.boxes) + 2               # + sensor body and nose
    assert model.nmocap == 1


def test_doorway_is_open(two_rooms):
    c = BoxCaster(two_rooms)
    # from room A towards +x: through the doorway (y = 0) reaches room B's east wall (inner face x = 9.9),
    # beside it (y = 1.0) hits the shared wall's inner face (x = 3.9)
    o = np.array([[0.0, 0.0, 1.2], [0.0, 1.0, 1.2]])
    d = np.array([[1.0, 0.0, 0.0]] * 2)
    r, _, _ = c.cast(o, d, 0.1, 40.0)
    assert r[1] == pytest.approx(3.9)
    assert r[0] > 9.0                                            # may be cut by a box in B, never by the wall
    # above the doorway there is the lintel (opening height 2.1)
    r2, _, _ = c.cast(np.array([[0.0, 0.0, 2.4]]), np.array([[1.0, 0.0, 0.0]]), 0.1, 40.0)
    assert r2[0] == pytest.approx(3.9)


def test_invalid_scenes():
    with pytest.raises(ValueError):
        scene_from_dict({"walls": []})
    with pytest.raises(ValueError):
        scene_from_dict({"walls": [{"from": [0, 0], "to": [0, 0]}]})
    with pytest.raises(ValueError):
        scene_from_dict({"walls": [{"from": [0, 0], "to": [2, 0], "openings": [{"at": 1.9, "width": 1.0}]}]})
    with pytest.raises(ValueError):
        scene_from_dict({"rooms": [{"polygon": [[0, 0], [1, 0]]}]})
