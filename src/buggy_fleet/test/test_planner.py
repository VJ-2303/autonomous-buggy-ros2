import pytest
from buggy_fleet.kct_map import KctMap
from buggy_fleet.campus_planner import CampusPlanner


def test_kct_map_integrity():
    kct = KctMap()
    assert len(kct.nodes) == 37
    assert len(kct.edges) == 45
    assert "Main Gate" in kct.name_to_id
    assert "Admin Block" in kct.name_to_id
    assert "M Gate" in kct.name_to_id

    # Test GeoJSON conversion
    geojson = kct.to_geojson()
    assert geojson["type"] == "FeatureCollection"
    assert len(geojson["features"]) == 37 + 45


def test_planner_single_rider():
    planner = CampusPlanner()
    back_gate = planner.map.name_to_id["Back Gate"]
    g_hostel = planner.map.name_to_id["G Hostel"]
    e_block = planner.map.name_to_id["E Block"]

    rider = {
        "id": "A",
        "pickup": g_hostel,
        "drop": e_block,
        "status": "wait",
    }

    res = planner.plan(car_node=back_gate, battery_pct=60.0, active_riders=[], new_rider=rider)
    assert res["ok"] is True
    assert res["distance_m"] > 0
    assert "P_A" in res["order"][0]
    assert "D_A" in res["order"][1]
    assert len(res["path_nodes"]) > 2


def test_planner_two_riders_shared():
    planner = CampusPlanner()
    m_gate = planner.map.name_to_id["M Gate"]
    kore = planner.map.name_to_id["Kore"]
    the_orb = planner.map.name_to_id["The Orb"]
    admin = planner.map.name_to_id["Admin Block"]

    riders = [
        {"id": "A", "pickup": kore, "drop": admin, "status": "wait"},
        {"id": "B", "pickup": the_orb, "drop": admin, "status": "wait"},
    ]

    res = planner.plan(car_node=m_gate, battery_pct=80.0, active_riders=riders)
    assert res["ok"] is True
    assert len(res["order"]) == 4  # P_A, P_B, D_A, D_B (in some order)
    assert res["battery_left"] > 20.0


def test_planner_low_battery_rejection():
    planner = CampusPlanner()
    m_gate = planner.map.name_to_id["M Gate"]
    kore = planner.map.name_to_id["Kore"]
    admin = planner.map.name_to_id["Admin Block"]

    rider = {"id": "A", "pickup": kore, "drop": admin, "status": "wait"}

    # With only 25% battery, after ~600m trip at 0.01%/m, remaining < 20% reserve
    res = planner.plan(car_node=m_gate, battery_pct=22.0, active_riders=[], new_rider=rider)
    assert res["ok"] is False
    assert res["rejected"]["battery"] > 0
