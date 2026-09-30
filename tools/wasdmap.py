"""Карта для клиента WASD: разбор позы из телеметрии, след, пересчёт в экран."""
import re

POSE = re.compile(r' px=(-?\d+) py=(-?\d+) ph=(-?\d+) nav=(\d)')


def parse_pose(line):
    """(x см, y см, курс °, фаза возврата) из строки телеметрии или None."""
    m = POSE.search(line)
    return (float(m[1]), float(m[2]), float(m[3]), int(m[4])) if m else None


class Trail:
    """След робота: точка добавляется, если отошли на min_step см; не больше cap."""

    def __init__(self, min_step=1.0, cap=3000):
        self.min_step, self.cap, self.points = min_step, cap, []

    def add(self, x, y):
        if self.points:
            lx, ly = self.points[-1]
            if (x - lx) ** 2 + (y - ly) ** 2 < self.min_step ** 2:
                return
        self.points.append((x, y))
        if len(self.points) > self.cap:
            del self.points[0]

    def clear(self):
        self.points = []


class MapView:
    """Квадрат size_px, дом в центре; вперёд (x) — вверх, влево (y) — влево."""

    def __init__(self, size_px, min_extent=50):
        self.size, self.min_extent, self.extent = size_px, min_extent, min_extent

    def fit(self, points, robot_xy):
        """Масштаб под след и робота: самая дальняя точка + 20 %, не меньше min_extent."""
        far = max([abs(c) for p in list(points) + [robot_xy] for c in p] + [0])
        self.extent = max(self.min_extent, far * 1.2)

    @property
    def scale(self):
        """Пикселей на сантиметр."""
        return (self.size / 2) / self.extent

    def to_screen(self, x, y):
        c = self.size / 2
        return round(c - y * self.scale), round(c - x * self.scale)
