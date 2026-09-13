import math
import unittest
from delivery_ros.core import search_path, distance, centered


class GeometryTests(unittest.TestCase):
    def test_search_covers_both_edges_without_overshoot(self):
        path = search_path(10, 20, -5, 3, 2.5)
        self.assertEqual(path[0], (7, 17, -5))
        self.assertEqual(path[-1][0], 13)
        self.assertTrue(all(7 <= n <= 13 and 17 <= e <= 23 and d == -5
                            for n, e, d in path))
        self.assertEqual(path[1][1], path[2][1])

    def test_invalid_geometry(self):
        for radius, spacing in [(0, 1), (1, 0), (math.nan, 1), (1000, .01)]:
            with self.assertRaises(ValueError):
                search_path(0, 0, -5, radius, spacing)

    def test_marker_validation(self):
        self.assertTrue(centered(320, 240, 640, 480, .1))
        self.assertFalse(centered(0, 0, 640, 480, .1))
        self.assertFalse(centered(math.nan, 240, 640, 480, .1))
        self.assertFalse(centered(0, 0, 0, 0, .1))

    def test_distance(self):
        self.assertEqual(distance((0, 0, -5), (3, 4, -5)), 5)


if __name__ == '__main__':
    unittest.main()
