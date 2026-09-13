"""Explicit ENU/NED, FLU/FRD and PX4 spherical map projection."""
import math


def enu_ned(v):
    return (float(v[1]), float(v[0]), -float(v[2]))


def flu_frd(v):
    return (float(v[0]), -float(v[1]), -float(v[2]))


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def normalize(q):
    n = math.sqrt(sum(x*x for x in q))
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('Invalid quaternion')
    return tuple(x/n for x in q)


def rotate(q, v):
    q = normalize(q)
    return multiply(multiply(q, (*v, 0.)), (-q[0], -q[1], -q[2], q[3]))[:3]


def px4_attitude_to_ros(wxyz):
    w, x, y, z = wxyz
    h = math.sqrt(.5)
    return normalize(multiply(multiply((h, h, 0., 0.), (x, y, z, w)),
                              (1., 0., 0., 0.)))


def project_enu(lat, lon, ref_lat, ref_lon):
    """PX4 MapProjection azimuthal equidistant convention; return east,north."""
    if not all(math.isfinite(x) for x in (lat, lon, ref_lat, ref_lon)):
        raise ValueError('Nonfinite coordinates')
    if abs(lat) > 90 or abs(ref_lat) > 90 or abs(lon) > 180 or abs(ref_lon) > 180:
        raise ValueError('Invalid latitude/longitude')
    p, p0, dl = map(math.radians, (lat, ref_lat, lon-ref_lon))
    c = math.acos(max(-1., min(1., math.sin(p0)*math.sin(p)
                              + math.cos(p0)*math.cos(p)*math.cos(dl))))
    k = c / math.sin(c) if c > 1e-8 else 1.
    return (6371000.*k*math.cos(p)*math.sin(dl),
            6371000.*k*(math.cos(p0)*math.sin(p)-math.sin(p0)*math.cos(p)*math.cos(dl)))


def step_toward(a, b, limit):
    d = math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    return tuple(a[i]+(b[i]-a[i])*min(1., limit/max(d,1e-9)) for i in range(3))
