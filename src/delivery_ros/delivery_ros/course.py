"""Validated competition-course model and deterministic graph planner."""
from dataclasses import dataclass
import heapq
import math
from pathlib import Path
import yaml


@dataclass(frozen=True)
class Destination:
    name: str
    marker_id: int
    latitude: float
    longitude: float
    altitude: float
    route: tuple
    position: tuple


@dataclass(frozen=True)
class Package:
    name: str
    marker_id: int
    mass_kg: float
    destination: str
    position: tuple


@dataclass(frozen=True)
class Course:
    course_id: str
    home_marker_id: int
    packages: tuple
    destinations: dict
    nodes: dict
    edges: tuple
    geofence: tuple
    origin: tuple
    home: tuple
    obstacles: tuple

    @classmethod
    def load(cls, path, course_id):
        raw = yaml.safe_load(Path(path).read_text()) or {}
        data = (raw.get('courses') or {}).get(course_id)
        if not isinstance(data, dict):
            raise ValueError(f'Unknown course_id: {course_id}')
        origin_data=data.get('origin') or {}
        origin=(float(origin_data['latitude']),float(origin_data['longitude']),float(origin_data.get('altitude',0.)))
        home_data=data.get('home') or {}
        nodes = {str(k): tuple(map(float, v)) for k, v in (data.get('route_nodes') or {}).items()}
        edges = tuple((str(e[0]), str(e[1]), float(e[2]) if len(e) > 2 else 1.)
                      for e in data.get('route_edges', ()))
        destinations = {}
        for name, item in (data.get('delivery_points') or {}).items():
            position=tuple(map(float,item['position']))
            latitude,longitude=local_to_gps(position[:2],origin[:2])
            destinations[str(name)] = Destination(str(name), int(item['marker_id']),
                latitude,longitude,float(item.get('flight_altitude',2.)),
                tuple(map(str,item.get('route',()))),position)
        packages = tuple(Package(str(p['name']),int(p['marker_id']),float(p['mass_kg']),
                                 str(p['destination']),tuple(map(float,p['position'])))
                         for p in data.get('pickup_points', ()))
        course = cls(course_id,int(home_data['marker_id']),packages,destinations,nodes,edges,
                     tuple(tuple(map(float,p)) for p in data.get('geofence',())),origin,
                     tuple(map(float,home_data['position'])),tuple(data.get('obstacles',())))
        course.validate()
        return course

    def validate(self):
        if len(self.packages) != 3 or [p.mass_kg for p in self.packages] != sorted(p.mass_kg for p in self.packages):
            raise ValueError('A Table C course requires three packages in increasing mass order')
        marker_ids = [self.home_marker_id] + [p.marker_id for p in self.packages]
        marker_ids += [d.marker_id for d in self.destinations.values()]
        if len(marker_ids) != len(set(marker_ids)) or any(not 0 <= i < 250 for i in marker_ids):
            raise ValueError('Marker IDs must be unique and in DICT_5X5_250')
        for p in self.packages:
            if p.destination not in self.destinations or not .2 <= p.mass_kg <= 1.1:
                raise ValueError('Invalid package destination or mass')
            if len(p.position)!=3 or (self.geofence and not point_in_polygon(p.position[:2],self.geofence)):
                raise ValueError(f'Invalid pickup position for {p.name}')
        for name, xyz in self.nodes.items():
            if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
                raise ValueError(f'Invalid node {name}')
            if self.geofence and not point_in_polygon(xyz[:2], self.geofence):
                raise ValueError(f'Node {name} outside geofence')
        planner = RoutePlanner(self.nodes, self.edges)
        for d in self.destinations.values():
            if any(n not in self.nodes for n in d.route):
                raise ValueError(f'Unknown route node for {d.name}')
            if d.route:
                planner.path(d.route[0], d.route[-1])
        for obstacle in self.obstacles:
            if obstacle.get('type') not in ('box','gate','tunnel') or not obstacle.get('name'):
                raise ValueError('Obstacle requires unique name and supported type')
        names=[o['name'] for o in self.obstacles]
        if len(names)!=len(set(names)):
            raise ValueError('Obstacle names must be unique')


def local_to_gps(position,origin):
    east,north=position; latitude,longitude=origin
    radius=6371000.
    return (latitude+math.degrees(north/radius),
            longitude+math.degrees(east/(radius*math.cos(math.radians(latitude)))))


def point_in_polygon(point, polygon):
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    j = len(polygon)-1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
            inside = not inside
        j = i
    return inside


class RoutePlanner:
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.graph = {name: [] for name in nodes}
        for a, b, cost in edges:
            if a not in nodes or b not in nodes or cost <= 0:
                raise ValueError('Invalid route edge')
            self.graph[a].append((b, cost)); self.graph[b].append((a, cost))

    def path(self, start, goal, blocked=()):
        blocked = set(blocked)
        queue = [(0., start, ())]
        best = {}
        while queue:
            cost, node, prefix = heapq.heappop(queue)
            if node in best and best[node] <= cost:
                continue
            best[node] = cost
            route = prefix+(node,)
            if node == goal:
                return route
            for nxt, weight in self.graph.get(node, ()):
                if tuple(sorted((node, nxt))) not in blocked:
                    heapq.heappush(queue, (cost+weight, nxt, route))
        raise ValueError(f'No safe route from {start} to {goal}')
