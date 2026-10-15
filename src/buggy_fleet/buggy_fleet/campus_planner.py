"""
Campus Car Navigation Planner: Dijkstra, A*, Multi-Rider Pooling, and Constraints.
Implements the exact algorithms from doc/campus_nav.py.
"""

import heapq
import math
from itertools import permutations
from typing import Dict, List, Optional, Set, Tuple

from .kct_map import KctMap

# Operational constants from doc/campus_nav.py
SCALE_METERS_PER_PIXEL = 0.6
BATTERY_RATE_PER_METER = 0.01   # % battery used per meter
BATTERY_RESERVE_MIN = 20.0       # Minimum % battery that must remain after the trip
MAX_DETOUR_RATIO = 1.5           # Max detour: ride distance <= 1.5 * direct distance
MAX_SEATS = 4                    # Car passenger capacity
CRUISING_SPEED_MPS = 5.0         # 5 m/s (~18 km/h)


class CampusPlanner:
    """Multi-rider campus car planner matching doc/campus_nav.py."""

    def __init__(self, kct_map: Optional[KctMap] = None):
        self.map = kct_map if kct_map is not None else KctMap()
        self.adj = self.map.adj
        self.num_nodes = len(self.map.nodes)

        # Precompute all-pairs shortest distances D[u][v] using Dijkstra
        self.D: Dict[int, Dict[int, float]] = {
            i: self._dijkstra_all(i) for i in range(self.num_nodes)
        }

    def _dijkstra_all(self, src: int) -> Dict[int, float]:
        """Dijkstra algorithm: shortest distance from src to every node."""
        distances = {src: 0.0}
        pq = [(0.0, src)]
        visited: Set[int] = set()

        while pq:
            du, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)

            for v, w in self.adj[u]:
                nd = du + w
                if nd < distances.get(v, float("inf")):
                    distances[v] = nd
                    heapq.heappush(pq, (nd, v))

        return distances

    def astar(self, src: int, goal: int) -> Tuple[List[int], float, int]:
        """
        A* search src -> goal.
        f(n) = g(n) + h(n), h = straight-line Euclidean distance to goal (admissible heuristic).
        Returns: (path_node_indices, total_length_m, nodes_explored).
        """
        if src == goal:
            return [src], 0.0, 1

        g = {src: 0.0}
        prev: Dict[int, int] = {}
        visited: Set[int] = set()
        pq = [(self.map.dist_nodes(src, goal), src)]

        while pq:
            _, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)

            if u == goal:
                break

            for v, w in self.adj[u]:
                ng = g[u] + w
                if ng < g.get(v, float("inf")):
                    g[v] = ng
                    prev[v] = u
                    h = self.map.dist_nodes(v, goal)
                    heapq.heappush(pq, (ng + h, v))

        if goal not in prev and goal != src:
            return [], float("inf"), len(visited)

        path = [goal]
        while path[0] != src:
            path.insert(0, prev[path[0]])

        return path, g[goal], len(visited)

    def valid_orders(self, riders: List[Dict]) -> List[List[Tuple[str, Dict]]]:
        """
        Generate all valid stop orders where each rider's pickup ('P') comes before their drop ('D').
        If a rider is already onboard ('on'), only their drop ('D') is scheduled.
        """
        stops = []
        for r in riders:
            if r.get("status") != "on":
                stops.append(("P", r))
            stops.append(("D", r))

        out = []

        def recurse(current: List[Tuple[str, Dict]], used: Set[int]):
            if len(current) == len(stops):
                out.append(list(current))
                return

            for i, stop in enumerate(stops):
                if i in used:
                    continue
                k, rider = stop
                # If it's a drop, ensure rider is either already onboard OR picked up earlier in current
                if k == "D" and rider.get("status") != "on":
                    if not any(c[0] == "P" and c[1]["id"] == rider["id"] for c in current):
                        continue

                used.add(i)
                current.append(stop)
                recurse(current, used)
                current.pop()
                used.remove(i)

        recurse([], set())
        return out

    def evaluate_order(
        self,
        order: List[Tuple[str, Dict]],
        active_riders: List[Dict],
        start_node: int,
        initial_offset_m: float,
        battery_pct: float,
    ) -> Tuple[bool, str, float]:
        """
        Evaluate an order against capacity, detour limit, and battery reserve.
        Returns: (is_valid, failure_reason, total_distance_m).
        """
        loc = start_node
        total = initial_offset_m
        load = len([r for r in active_riders if r.get("status") == "on"])
        pick_at: Dict[str, float] = {}

        # For already onboard riders, their pickup was in the past (ride distance starts from 0)
        for r in active_riders:
            if r.get("status") == "on":
                pick_at[r["id"]] = -r.get("ride_distance_m", 0.0)

        for k, r in order:
            dest_node = r["pickup"] if k == "P" else r["drop"]
            leg_dist = self.D[loc][dest_node]
            total += leg_dist
            loc = dest_node

            if k == "P":
                load += 1
                pick_at[r["id"]] = total
                if load > MAX_SEATS:
                    return False, "seats", total
            else:
                load -= 1
                ride_distance = total - pick_at[r["id"]]
                direct_dist = self.D[r["pickup"]][r["drop"]]
                if direct_dist > 0 and ride_distance > MAX_DETOUR_RATIO * direct_dist:
                    return False, "detour", total

        battery_remaining = battery_pct - (total * BATTERY_RATE_PER_METER)
        if battery_remaining < BATTERY_RESERVE_MIN:
            return False, "battery", total

        return True, "ok", total

    def plan(
        self,
        car_node: int,
        battery_pct: float,
        active_riders: List[Dict],
        new_rider: Optional[Dict] = None,
        moving_offset_m: float = 0.0,
    ) -> Dict:
        """
        Compute optimal stop order and path matching doc/campus_nav.py plan() function.
        """
        all_riders = list(active_riders)
        if new_rider is not None:
            all_riders.append(new_rider)

        orders = self.valid_orders(all_riders)
        rejected_counts = {"seats": 0, "detour": 0, "battery": 0}
        best_order = None
        best_total = float("inf")
        tried = 0

        for order in orders:
            tried += 1
            ok, why, total = self.evaluate_order(
                order, all_riders, car_node, moving_offset_m, battery_pct
            )
            if not ok:
                rejected_counts[why] += 1
            elif total < best_total:
                best_order = order
                best_total = total

        if best_order is None:
            return {
                "ok": False,
                "tried": tried,
                "rejected": rejected_counts,
                "valid": 0,
                "message": "No valid route satisfying seats, detour, and battery constraints.",
            }

        # Build node-by-node path using A*
        full_node_path = [car_node]
        loc = car_node
        astar_nodes_explored = 0
        dijkstra_nodes_explored = 0

        stop_entries = []
        for k, r in best_order:
            dest_node = r["pickup"] if k == "P" else r["drop"]
            leg_path, leg_len, a_exp = self.astar(loc, dest_node)
            d_dists, _, d_exp = self._dijkstra_goal(loc, dest_node)
            astar_nodes_explored += a_exp
            dijkstra_nodes_explored += d_exp

            if len(leg_path) > 1:
                full_node_path.extend(leg_path[1:])
            loc = dest_node
            stop_entries.append({"type": k, "rider_id": r["id"], "node": dest_node})

        battery_used = round(best_total * BATTERY_RATE_PER_METER, 2)
        battery_left = round(battery_pct - battery_used, 2)
        est_time_s = round(best_total / CRUISING_SPEED_MPS)

        return {
            "ok": True,
            "tried": tried,
            "rejected": rejected_counts,
            "valid": tried - sum(rejected_counts.values()),
            "distance_m": round(best_total, 1),
            "time_s": est_time_s,
            "battery_used": battery_used,
            "battery_left": battery_left,
            "order": [f"{k}_{r['id']}" for k, r in best_order],
            "stops": stop_entries,
            "path_nodes": full_node_path,
            "path_names": [self.map.id_to_name[i] for i in full_node_path if self.map.nodes[i]["is_landmark"]],
            "astar_explored": astar_nodes_explored,
            "dijkstra_explored": dijkstra_nodes_explored,
        }

    def _dijkstra_goal(self, src: int, goal: int) -> Tuple[Dict[int, float], Dict[int, int], int]:
        """Dijkstra with early exit at goal for metric comparison."""
        d = {src: 0.0}
        prev = {}
        visited: Set[int] = set()
        pq = [(0.0, src)]

        while pq:
            du, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break
            for v, w in self.adj[u]:
                nd = du + w
                if nd < d.get(v, float("inf")):
                    d[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        return d, prev, len(visited)
