"""
Campus car navigation planner (one car, shared rides).
Parts: graph -> Dijkstra -> A* -> distance table -> stop-order search
       -> seat / battery / detour checks -> planner -> demo
Run:  python campus_nav.py
"""
import heapq
import math
from itertools import permutations

# ---------------- Settings (tune from real test drives) ----------------
SC = 0.6        # metres per drawing pixel (calibrate with one real distance)
RATE = 0.01     # battery % used per metre
RESERVE = 20    # battery % that must remain after the trip
DETOUR = 1.5    # ride distance <= DETOUR * direct distance, per rider
SEATS = 4
SPEED = 5       # m/s

# ---------------- 1. Graph (from the hand-drawn KCT map) ----------------
# (name, x, y)  -- "Junction" entries are road junctions, not places
NODES = [
    ("Main Gate", 740, 67), ("Craft", 730, 310), ("Junction", 725, 401), ("Junction", 732, 507),
    ("G Hostel", 755, 737), ("Junction", 770, 773), ("Junction", 785, 913), ("Junction", 530, 927),
    ("M Gate", 430, 931), ("Junction", 397, 936), ("Junction", 233, 940), ("Junction", 78, 947),
    ("Back Gate", 45, 952), ("Junction", 88, 638), ("Junction", 95, 422), ("KVB", 160, 420),
    ("Junction", 245, 410), ("Kore", 323, 407), ("Junction", 502, 404), ("The Orb", 580, 407),
    ("B Block", 248, 318), ("A Block", 255, 208), ("E Block", 258, 80), ("OSA", 108, 323),
    ("Garage", 118, 243), ("Junction", 120, 150), ("Junction", 188, 13), ("C Block", 238, 527),
    ("Auditorium", 238, 608), ("Junction", 238, 738), ("F Block", 345, 738), ("Junction", 397, 736),
    ("Junction", 513, 728), ("Junction", 518, 760), ("AP Backside", 625, 777), ("Junction", 507, 513),
    ("Admin Block", 610, 507),
]
EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10), (10, 11), (11, 12),
    (14, 13), (13, 11), (14, 23), (23, 24), (24, 25), (14, 15), (15, 16), (16, 17), (17, 18), (18, 19),
    (19, 2), (16, 20), (20, 21), (21, 22), (16, 27), (27, 28), (28, 29), (29, 10), (25, 26), (26, 22),
    (13, 28), (29, 30), (30, 31), (31, 32), (31, 9), (18, 35), (35, 32), (32, 33), (33, 7), (35, 36),
    (36, 3), (33, 34), (34, 5),
]
ID = {n: i for i, (n, _, _) in enumerate(NODES) if n != "Junction"}  # place name -> node index


def dist(a, b):
    """Straight-line distance in metres between two nodes (also the A* heuristic)."""
    return round(math.hypot(NODES[a][1] - NODES[b][1], NODES[a][2] - NODES[b][2]) * SC)


GRAPH = {i: [] for i in range(len(NODES))}
for a, b in EDGES:
    w = dist(a, b)
    GRAPH[a].append((b, w))
    GRAPH[b].append((a, w))


# ---------------- 2. Dijkstra ----------------
def dijkstra(src, goal=None):
    """Shortest distance from src to every node (stops early if goal is given).
    Returns (dist, prev, nodes_explored)."""
    d = {src: 0}
    prev = {}
    done = set()
    pq = [(0, src)]
    while pq:
        du, u = heapq.heappop(pq)
        if u in done:
            continue
        done.add(u)
        if u == goal:
            break
        for v, w in GRAPH[u]:
            nd = du + w
            if nd < d.get(v, float("inf")):
                d[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return d, prev, len(done)


# ---------------- 3. A* ----------------
def astar(src, goal):
    """Shortest path src -> goal. Returns (path, length, nodes_explored).
    f(n) = g(n) + h(n), h = straight-line distance to goal (never overestimates)."""
    g = {src: 0}
    prev = {}
    done = set()
    pq = [(dist(src, goal), src)]
    while pq:
        _, u = heapq.heappop(pq)
        if u in done:
            continue
        done.add(u)
        if u == goal:
            break
        for v, w in GRAPH[u]:
            ng = g[u] + w
            if ng < g.get(v, float("inf")):
                g[v] = ng
                prev[v] = u
                heapq.heappush(pq, (ng + dist(v, goal), v))
    path = [goal]
    while path[0] != src:
        path.insert(0, prev[path[0]])
    return path, g[goal], len(done)


# ---------------- 4. All-pairs distance table ----------------
D = {i: dijkstra(i)[0] for i in GRAPH}  # D[a][b] = shortest road distance in metres


# ---------------- 5. Stop-order search ----------------
def valid_orders(riders):
    """All stop orders where each rider's pickup comes before their drop.
    riders = {id: (pickup_node, drop_node)}.  Stops are ('P', id) / ('D', id)."""
    stops = [(k, r) for r in riders for k in "PD"]
    for order in permutations(stops):
        picked = set()
        ok = True
        for k, r in order:
            if k == "P":
                picked.add(r)
            elif r not in picked:
                ok = False
                break
        if ok:
            yield order


# ---------------- 6. Constraint checks ----------------
def evaluate(order, car, battery, riders):
    """Returns (valid, reason, total_distance_m). Checks seats, detour and battery."""
    loc, total, load = car, 0, 0
    pick_at = {}
    for k, r in order:
        node = riders[r][0] if k == "P" else riders[r][1]
        total += D[loc][node]
        loc = node
        if k == "P":
            load += 1
            pick_at[r] = total
            if load > SEATS:
                return False, "seats", total
        else:
            load -= 1
            ride = total - pick_at[r]
            direct = D[riders[r][0]][riders[r][1]]
            if ride > DETOUR * direct:
                return False, "detour", total
    if battery - total * RATE < RESERVE:
        return False, "battery", total
    return True, "ok", total


# ---------------- 7. Planner ----------------
def plan(car_place, battery, request):
    """request = {rider_id: (pickup_place, drop_place)} using place names.
    Returns a dict with the best order, distance, time, battery and node-by-node path,
    or None with a reason summary if no valid plan exists."""
    car = ID[car_place]
    riders = {r: (ID[p], ID[d]) for r, (p, d) in request.items()}
    best, best_total = None, float("inf")
    rejected = {"seats": 0, "detour": 0, "battery": 0}
    tried = 0
    for order in valid_orders(riders):
        tried += 1
        ok, why, total = evaluate(order, car, battery, riders)
        if not ok:
            rejected[why] += 1
        elif total < best_total:
            best, best_total = order, total
    if best is None:
        return {"ok": False, "tried": tried, "rejected": rejected}

    # drive the chosen order with A*
    path, loc, an, dn = [car], car, 0, 0
    for k, r in best:
        node = riders[r][0] if k == "P" else riders[r][1]
        p, _, n_a = astar(loc, node)
        dn += dijkstra(loc, node)[2]
        an += n_a
        path += p[1:]
        loc = node
    return {
        "ok": True, "order": [f"{k}_{r}" for k, r in best], "tried": tried, "rejected": rejected,
        "distance_m": best_total, "time_s": round(best_total / SPEED),
        "battery_used": round(best_total * RATE, 1),
        "battery_left": round(battery - best_total * RATE, 1),
        "path": [NODES[i][0] for i in path if NODES[i][0] != "Junction"],
        "astar_explored": an, "dijkstra_explored": dn,
    }


# ---------------- Demo ----------------
if __name__ == "__main__":
    print("--- Two riders, shared drop (battery 80%) ---")
    req = {"A": ("Kore", "Admin Block"), "B": ("The Orb", "Admin Block")}
    res = plan("M Gate", 80, req)
    for k, v in res.items():
        print(f"{k:18}: {v}")

    # compare with separate trips (A fully, then B fully)
    car = ID["M Gate"]
    sep = (D[car][ID["Kore"]] + D[ID["Kore"]][ID["Admin Block"]]
           + D[ID["Admin Block"]][ID["The Orb"]] + D[ID["The Orb"]][ID["Admin Block"]])
    print(f"separate trips    : {sep} m  (sharing saves {sep - res['distance_m']} m)")

    print("\n--- Same request with low battery (25%) ---")
    print(plan("M Gate", 25, req))

    print("\n--- Single rider (battery 60%) ---")
    print(plan("Back Gate", 60, {"A": ("G Hostel", "E Block")}))
