"""ROS-independent local NED search geometry and detection validation."""
import math


def search_path(north, east, down, radius, spacing):
    values = (north, east, down, radius, spacing)
    if not all(math.isfinite(v) for v in values) or radius <= 0 or spacing <= 0:
        raise ValueError('Finite coordinates and positive radius/spacing required')
    count = int(math.ceil(2 * radius / spacing))
    if count > 200:
        raise ValueError('Search exceeds 200 rows')
    result = []
    for row in range(count + 1):
        n = north - radius + min(row * spacing, 2 * radius)
        ends = (-radius, radius) if row % 2 == 0 else (radius, -radius)
        result.extend((n, east + e, down) for e in ends)
    return result


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def centered(x, y, width, height, tolerance):
    return (width > 0 and height > 0 and 0 <= x < width and 0 <= y < height
            and abs(x / width - .5) <= tolerance
            and abs(y / height - .5) <= tolerance)
