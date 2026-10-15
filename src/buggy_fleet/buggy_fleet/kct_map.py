"""
KCT Campus Map: Real OpenStreetMap (OSM) Road Network, Georeferenced Coordinates,
and Smooth Trajectory Generation.
"""

import json
import math
import os
from typing import Dict, List, Optional, Tuple

CAMPUS_REF_LAT = 11.077800
CAMPUS_REF_LON = 76.988500
METERS_PER_DEGREE_LAT = 110600.0
METERS_PER_DEGREE_LON = 111320.0 * math.cos(math.radians(CAMPUS_REF_LAT))


def gps_to_meters(lat: float, lon: float) -> Tuple[float, float]:
    """Convert GPS coordinates (lat, lon) to local Cartesian metric coordinates (x_m, y_m)."""
    xm = (lon - CAMPUS_REF_LON) * METERS_PER_DEGREE_LON
    ym = (lat - CAMPUS_REF_LAT) * METERS_PER_DEGREE_LAT
    return xm, ym


def meters_to_gps(xm: float, ym: float) -> Tuple[float, float]:
    """Convert local Cartesian metric coordinates (x_m, y_m) to GPS coordinates (lat, lon)."""
    lat = CAMPUS_REF_LAT + (ym / METERS_PER_DEGREE_LAT)
    lon = CAMPUS_REF_LON + (xm / METERS_PER_DEGREE_LON)
    return lat, lon


class KctMap:
    """Represents the real KCT campus road network extracted from OpenStreetMap."""

    def __init__(self, json_path: Optional[str] = None):
        if json_path is None:
            # Look in package directory or resource folder
            candidates = [
                os.path.join(os.path.dirname(__file__), "../resource/kct_osm_network.json"),
                os.path.join(os.path.dirname(__file__), "../../resource/kct_osm_network.json"),
                os.path.abspath("src/buggy_fleet/resource/kct_osm_network.json"),
                "/opt/ros/jazzy/share/buggy_fleet/resource/kct_osm_network.json",
            ]
            for path in candidates:
                if os.path.isfile(path):
                    json_path = path
                    break

        if json_path and os.path.isfile(json_path):
            with open(json_path, "r") as f:
                data = json.load(f)
        else:
            raise FileNotFoundError(f"Could not find kct_osm_network.json in candidates: {candidates}")

        self.nodes = data["nodes"]
        self.edges = [tuple(e) for e in data["edges"]]
        self.landmarks_data = data["landmarks"]

        self.name_to_id: Dict[str, int] = {}
        self.id_to_name: Dict[int, str] = {}
        self.landmarks: List[Dict] = []

        for node in self.nodes:
            idx = node["id"]
            name = node["name"]
            self.id_to_name[idx] = name
            if node.get("is_landmark"):
                self.name_to_id[name] = idx
                self.landmarks.append(node)

        # Adjacency list
        self.adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(self.nodes))}
        for u, v in self.edges:
            w = self.dist_nodes(u, v)
            self.adj[u].append((v, w))
            self.adj[v].append((u, w))

    def dist_nodes(self, a: int, b: int) -> float:
        """Metric distance in metres between two nodes."""
        na = self.nodes[a]
        nb = self.nodes[b]
        return round(math.hypot(na["x_m"] - nb["x_m"], na["y_m"] - nb["y_m"]), 2)

    def interpolate_path(self, node_ids: List[int], step_meters: float = 1.0) -> List[Tuple[float, float, float, float]]:
        """
        Subdivide a sequence of road node IDs into fine-grained interpolated waypoints.
        Returns: list of (x_m, y_m, lat, lon) tuples spaced evenly for smooth vehicle motion.
        """
        if not node_ids:
            return []

        fine_points: List[Tuple[float, float, float, float]] = []

        for i in range(len(node_ids) - 1):
            u = node_ids[i]
            v = node_ids[i + 1]
            nu = self.nodes[u]
            nv = self.nodes[v]

            dx = nv["x_m"] - nu["x_m"]
            dy = nv["y_m"] - nu["y_m"]
            seg_len = math.hypot(dx, dy)

            steps = max(1, int(math.ceil(seg_len / step_meters)))

            for s in range(steps):
                alpha = s / float(steps)
                xm = nu["x_m"] + alpha * dx
                ym = nu["y_m"] + alpha * dy
                lat, lon = meters_to_gps(xm, ym)
                fine_points.append((round(xm, 2), round(ym, 2), lat, lon))

        # Add final destination point
        last_node = self.nodes[node_ids[-1]]
        fine_points.append((last_node["x_m"], last_node["y_m"], last_node["lat"], last_node["lon"]))

        return fine_points

    def to_geojson(self) -> Dict:
        """Export real OSM road network and landmarks as GeoJSON for Leaflet."""
        features = []

        # Road segments
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

        # Landmark Nodes
        for node in self.landmarks:
            features.append({
                "type": "Feature",
                "properties": {
                    "type": "landmark",
                    "id": node["id"],
                    "name": node["name"],
                    "is_landmark": True,
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
