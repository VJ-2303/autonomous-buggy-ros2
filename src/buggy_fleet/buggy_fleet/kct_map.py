"""KCT Campus Map: Georeferenced graph definition and coordinate transformations."""

import math
from typing import Dict, List, Tuple

# Calibration settings from doc/campus_nav.py
SCALE_METERS_PER_PIXEL = 0.6  # 1 pixel = 0.6 metres

# Real-world Anchor: Kumaraguru College of Technology (KCT, Coimbatore)
# Campus Center reference point (near Admin Block / The Orb)
CAMPUS_REF_PIXEL_X = 500.0
CAMPUS_REF_PIXEL_Y = 500.0
CAMPUS_REF_LAT = 11.079700
CAMPUS_REF_LON = 76.992500

METERS_PER_DEGREE_LAT = 110600.0
METERS_PER_DEGREE_LON = 111320.0 * math.cos(math.radians(CAMPUS_REF_LAT))

# The 37 Nodes from doc/campus_nav.py & index.html
# Format: (name, pixel_x, pixel_y, is_landmark)
RAW_NODES: List[Tuple[str, int, int, int]] = [
    ("Main Gate", 740, 67, 1),
    ("Craft", 730, 310, 1),
    ("Junction", 725, 401, 0),
    ("Junction", 732, 507, 0),
    ("G Hostel", 755, 737, 1),
    ("Junction", 770, 773, 0),
    ("Junction", 785, 913, 0),
    ("Junction", 530, 927, 0),
    ("M Gate", 430, 931, 1),
    ("Junction", 397, 936, 0),
    ("Junction", 233, 940, 0),
    ("Junction", 78, 947, 0),
    ("Back Gate", 45, 952, 1),
    ("Junction", 88, 638, 0),
    ("Junction", 95, 422, 0),
    ("KVB", 160, 420, 1),
    ("Junction", 245, 410, 0),
    ("Kore", 323, 407, 1),
    ("Junction", 502, 404, 0),
    ("The Orb", 580, 407, 1),
    ("B Block", 248, 318, 1),
    ("A Block", 255, 208, 1),
    ("E Block", 258, 80, 1),
    ("OSA", 108, 323, 1),
    ("Garage", 118, 243, 1),
    ("Junction", 120, 150, 0),
    ("Junction", 188, 13, 0),
    ("C Block", 238, 527, 1),
    ("Auditorium", 238, 608, 1),
    ("Junction", 238, 738, 0),
    ("F Block", 345, 738, 1),
    ("Junction", 397, 736, 0),
    ("Junction", 513, 728, 0),
    ("Junction", 518, 760, 0),
    ("AP Backside", 625, 777, 1),
    ("Junction", 507, 513, 0),
    ("Admin Block", 610, 507, 1),
]

# The 45 Edges connecting nodes
RAW_EDGES: List[Tuple[int, int]] = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10), (10, 11), (11, 12),
    (14, 13), (13, 11), (14, 23), (23, 24), (24, 25), (14, 15), (15, 16), (16, 17), (17, 18), (18, 19),
    (19, 2), (16, 20), (20, 21), (21, 22), (16, 27), (27, 28), (28, 29), (29, 10), (25, 26), (26, 22),
    (13, 28), (29, 30), (30, 31), (31, 32), (31, 9), (18, 35), (35, 32), (32, 33), (33, 7), (35, 36),
    (36, 3), (33, 34), (34, 5),
]


def pixel_to_meters(px: float, py: float) -> Tuple[float, float]:
    """Convert drawing pixel coordinates to local Cartesian metric coordinates (x_east, y_north)."""
    dx_meters = (px - CAMPUS_REF_PIXEL_X) * SCALE_METERS_PER_PIXEL
    dy_meters = (CAMPUS_REF_PIXEL_Y - py) * SCALE_METERS_PER_PIXEL
    return dx_meters, dy_meters


def meters_to_pixel(xm: float, ym: float) -> Tuple[float, float]:
    """Convert local Cartesian metric coordinates to drawing pixel coordinates."""
    px = CAMPUS_REF_PIXEL_X + (xm / SCALE_METERS_PER_PIXEL)
    py = CAMPUS_REF_PIXEL_Y - (ym / SCALE_METERS_PER_PIXEL)
    return px, py


def pixel_to_gps(px: float, py: float) -> Tuple[float, float]:
    """Convert drawing pixel coordinates to real GPS coordinates (latitude, longitude)."""
    dx_meters, dy_meters = pixel_to_meters(px, py)
    lat = CAMPUS_REF_LAT + (dy_meters / METERS_PER_DEGREE_LAT)
    lon = CAMPUS_REF_LON + (dx_meters / METERS_PER_DEGREE_LON)
    return lat, lon


def gps_to_pixel(lat: float, lon: float) -> Tuple[float, float]:
    """Convert GPS coordinates to drawing pixel coordinates."""
    dy_meters = (lat - CAMPUS_REF_LAT) * METERS_PER_DEGREE_LAT
    dx_meters = (lon - CAMPUS_REF_LON) * METERS_PER_DEGREE_LON
    return meters_to_pixel(dx_meters, dy_meters)


def gps_to_meters(lat: float, lon: float) -> Tuple[float, float]:
    """Convert GPS coordinates to local Cartesian metric coordinates."""
    px, py = gps_to_pixel(lat, lon)
    return pixel_to_meters(px, py)


class KctMap:
    """Represents the complete KCT campus road network with both metric and GPS coordinates."""

    def __init__(self):
        self.nodes = []
        self.name_to_id: Dict[str, int] = {}
        self.id_to_name: Dict[int, str] = {}
        self.landmarks: List[Dict] = []

        for idx, (name, px, py, is_landmark) in enumerate(RAW_NODES):
            lat, lon = pixel_to_gps(px, py)
            xm, ym = pixel_to_meters(px, py)
            node_data = {
                "id": idx,
                "name": name,
                "px": px,
                "py": py,
                "x_m": xm,
                "y_m": ym,
                "lat": lat,
                "lon": lon,
                "is_landmark": bool(is_landmark),
            }
            self.nodes.append(node_data)
            self.id_to_name[idx] = name
            if is_landmark:
                self.name_to_id[name] = idx
                self.landmarks.append(node_data)

        self.edges = RAW_EDGES
        self.adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(self.nodes))}
        for u, v in self.edges:
            w = self.dist_nodes(u, v)
            self.adj[u].append((v, w))
            self.adj[v].append((u, w))

    def dist_nodes(self, a: int, b: int) -> float:
        """Straight-line metric distance in metres between two nodes."""
        na = self.nodes[a]
        nb = self.nodes[b]
        return round(math.hypot(na["px"] - nb["px"], na["py"] - nb["py"]) * SCALE_METERS_PER_PIXEL, 1)

    def to_geojson(self) -> Dict:
        """Export the road network and landmark nodes as a GeoJSON FeatureCollection for Leaflet/OSM."""
        features = []

        # Road network LineStrings
        for u, v in self.edges:
            nu = self.nodes[u]
            nv = self.nodes[v]
            features.append({
                "type": "Feature",
                "properties": {
                    "type": "road",
                    "from_id": u,
                    "to_id": v,
                    "from_name": nu["name"],
                    "to_name": nv["name"],
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [nu["lon"], nu["lat"]],
                        [nv["lon"], nv["lat"]],
                    ]
                }
            })

        # Landmark Points
        for node in self.nodes:
            features.append({
                "type": "Feature",
                "properties": {
                    "type": "landmark" if node["is_landmark"] else "junction",
                    "id": node["id"],
                    "name": node["name"],
                    "is_landmark": node["is_landmark"],
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [node["lon"], node["lat"]]
                }
            })

        return {
            "type": "FeatureCollection",
            "features": features,
            "center": [CAMPUS_REF_LAT, CAMPUS_REF_LON],
            "zoom": 17,
        }
