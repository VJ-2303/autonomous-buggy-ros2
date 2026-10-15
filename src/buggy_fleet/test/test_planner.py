import pytest
from buggy_fleet.kct_map import KctMap
from buggy_fleet.campus_planner import CampusPlanner


def test_real_osm_map_integrity():
    kct = KctMap()
    assert len(kct.nodes) == 164
    assert len(kct.edges) == 179
    assert "Main Gate (North)" in kct.name_to_id
    assert "Admin Block" in kct.name_to_id
    assert "Kore Food Court" in kct.name_to_id
    assert "South Gate (M Gate)" in kct.name_to_id

    # Test GeoJSON conversion
    geojson = kct.to_geojson()
    assert geojson["type"] == "FeatureCollection"
    assert len(geojson["features"]) == len(kct.edges) + len(kct.landmarks)

    # Test fine interpolation
    sample_path = [0, 1, 2]
    fine_pts = kct.interpolate_path(sample_path, step_meters=1.0)
    assert len(fine_pts) >= len(sample_path)


def test_planner_single_rider_real_map():
    planner = CampusPlanner()
    start_node = planner.map.name_to_id["South Gate (M Gate)"]
    kore = planner.map.name_to_id["Kore Food Court"]
    admin = planner.map.name_to_id["Admin Block"]

    rider = {
        "id": "A",
        "pickup": kore,
        "drop": admin,
        "status": "wait",
    }

    res = planner.plan(car_node=start_node, battery_pct=60.0, active_riders=[], new_rider=rider)
    assert res["ok"] is True
    assert res["distance_m"] > 0
    assert "P_A" in res["order"][0]
    assert "D_A" in res["order"][1]
    assert len(res["path_nodes"]) >= 2


def test_planner_two_riders_shared_real_map():
    planner = CampusPlanner()
    start_node = planner.map.name_to_id["South Gate (M Gate)"]
    kore = planner.map.name_to_id["Kore Food Court"]
    auditorium = planner.map.name_to_id["Auditorium"]
    admin = planner.map.name_to_id["Admin Block"]

    riders = [
        {"id": "A", "pickup": kore, "drop": admin, "status": "wait"},
        {"id": "B", "pickup": auditorium, "drop": admin, "status": "wait"},
    ]

    res = planner.plan(car_node=start_node, battery_pct=80.0, active_riders=riders)
    assert res["ok"] is True
    assert len(res["order"]) == 4  # P_A, P_B, D_A, D_B (in some order)
    assert res["battery_left"] > 20.0


def test_planner_low_battery_rejection_real_map():
    planner = CampusPlanner()
    start_node = planner.map.name_to_id["South Gate (M Gate)"]
    kore = planner.map.name_to_id["Kore Food Court"]
    admin = planner.map.name_to_id["Admin Block"]

    rider = {"id": "A", "pickup": kore, "drop": admin, "status": "wait"}

    # With only 22% battery, after trip at 0.01%/m, remaining < 20% reserve
    res = planner.plan(car_node=start_node, battery_pct=21.0, active_riders=[], new_rider=rider)
    assert res["ok"] is False
    assert res["rejected"]["battery"] > 0
