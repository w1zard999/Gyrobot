import unittest

from wasdmap import MapView, Trail, parse_pose


class T(unittest.TestCase):
    def test_parse(self):
        line = ('A a=0.1 r=2 v=0.0 vA=0 vB=0 x=5 m=0 yr=0 h=0.0 uL=1 uR=1 '
                'px=12 py=-3 ph=-170 nav=2 hz=200 ie=0')
        self.assertEqual(parse_pose(line), (12.0, -3.0, -170.0, 2))
        self.assertIsNone(parse_pose('- a=1 r=0 v=0'))

    def test_trail_thins_and_caps(self):
        t = Trail(min_step=1.0, cap=3)
        for x in (0, 0.5, 2, 4, 6):
            t.add(x, 0)
        self.assertEqual(t.points, [(2, 0), (4, 0), (6, 0)])
        t.clear()
        self.assertEqual(t.points, [])

    def test_view_axes(self):
        m = MapView(200, min_extent=50)
        m.fit([], (0, 0))
        cx, cy = m.to_screen(0, 0)
        fx, fy = m.to_screen(10, 0)     # вперёд — вверх
        lx, ly = m.to_screen(0, 10)     # влево — влево
        self.assertEqual((cx, cy), (100, 100))
        self.assertTrue(fy < cy and fx == cx)
        self.assertTrue(lx < cx and ly == cy)

    def test_view_fits_far_point(self):
        m = MapView(200, min_extent=50)
        m.fit([(300, 0)], (0, 0))
        px, py = m.to_screen(300, 0)
        self.assertTrue(0 <= py <= 200)


if __name__ == '__main__':
    unittest.main()
