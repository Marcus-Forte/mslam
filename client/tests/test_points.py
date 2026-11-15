import unittest

import numpy as np

from tests.helpers import lidar_pb2, make_cloud
from points import (
    HEIGHT_RANGE_M,
    INTENSITY_RANGE,
    intensity_to_colors,
    to_point_colors,
    to_viser_colors,
    to_viser_points,
)


def _colors(cloud: lidar_pb2.PointCloud3) -> np.ndarray:
    return to_point_colors(cloud, to_viser_points(cloud))


class ColorTest(unittest.TestCase):
    def test_intensity_colors_do_not_depend_on_the_other_points_of_the_message(self) -> None:
        whole = make_cloud(300)
        expected = _colors(whole)
        for lo, hi in [(0, 1), (5, 6), (0, 17), (100, 300), (255, 256)]:
            part = lidar_pb2.PointCloud3()
            part.x.extend(whole.x[lo:hi])
            part.y.extend(whole.y[lo:hi])
            part.z.extend(whole.z[lo:hi])
            part.intensity.extend(whole.intensity[lo:hi])
            np.testing.assert_array_equal(_colors(part), expected[lo:hi])

    def test_intensity_uses_the_fixed_range_and_saturates_outside_it(self) -> None:
        cloud = lidar_pb2.PointCloud3()
        cloud.x.extend([0.0] * 3)
        cloud.y.extend([0.0] * 3)
        cloud.z.extend([0.0] * 3)
        cloud.intensity.extend([int(INTENSITY_RANGE[0]), int(INTENSITY_RANGE[1]), 1000])
        colors = intensity_to_colors(cloud, 3)
        assert colors is not None
        np.testing.assert_array_equal(colors[1], colors[2])
        self.assertFalse(np.array_equal(colors[0], colors[1]))

        # A single point of value 100 is coloured the same alone and next to 0 and 255.
        alone = lidar_pb2.PointCloud3(intensity=[100])
        alone.x.append(0.0)
        alone.y.append(0.0)
        alone.z.append(0.0)
        full = lidar_pb2.PointCloud3(intensity=[0, 100, 255])
        full.x.extend([0.0] * 3)
        full.y.extend([0.0] * 3)
        full.z.extend([0.0] * 3)
        np.testing.assert_array_equal(_colors(alone)[0], _colors(full)[1])

    def test_missing_or_mismatched_intensity_returns_none(self) -> None:
        cloud = make_cloud(4)
        del cloud.intensity[:]
        self.assertIsNone(intensity_to_colors(cloud, 4))
        self.assertIsNone(intensity_to_colors(make_cloud(0), 0))

    def test_height_fallback_is_stable_per_point(self) -> None:
        z = np.linspace(HEIGHT_RANGE_M[0] - 1.0, HEIGHT_RANGE_M[1] + 1.0, 50, dtype=np.float32)
        points = np.zeros((50, 3), dtype=np.float32)
        points[:, 2] = z
        expected = to_viser_colors(points)
        for lo, hi in [(0, 1), (10, 11), (3, 40), (49, 50)]:
            np.testing.assert_array_equal(to_viser_colors(points[lo:hi]), expected[lo:hi])
        # Outside the range the colour saturates instead of stretching.
        np.testing.assert_array_equal(expected[0], expected[1])
        np.testing.assert_array_equal(expected[-1], expected[-2])

    def test_cloud_without_intensity_falls_back_to_height(self) -> None:
        cloud = make_cloud(10)
        del cloud.intensity[:]
        np.testing.assert_array_equal(_colors(cloud), to_viser_colors(to_viser_points(cloud)))

    def test_empty(self) -> None:
        self.assertEqual(to_viser_colors(np.empty((0, 3), dtype=np.float32)).shape, (0, 3))
        self.assertEqual(_colors(make_cloud(0)).shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
