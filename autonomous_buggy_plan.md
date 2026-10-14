# Autonomous Buggy: 4-Week Campus Navigation Plan

## 1. Goal and Scope

The goal is **not to follow one fixed path**. The buggy should accept a destination on a mapped campus road network, plan a route, drive it, respond safely to detected obstacles, and allow the operator to stop or take over at any time.

### Target for Day 28

Demonstrate destination-to-destination autonomous driving across the **mapped and tested campus drivable-road network**, at low speed, within a defined operating area and conditions, with a safety driver present.

“Anywhere” does **not** mean uncharted areas, sidewalks, construction zones, or unrestricted operation among pedestrians.

## 2. Team Structure (20 People)

| Team | People | Ownership |
|---|---:|---|
| **A — Vehicle Control & Safety** | 4 | Computer control of steering, throttle, and brakes; feedback; speed limits; physical emergency stop (E-stop); manual takeover. |
| **B — ROS 2, Sensors & Compute** | 4 | Vehicle computer; sensor drivers; time synchronization; sensor health; logs; startup. |
| **C — Mapping & Localization** | 3 | Campus drivable-road map; localization; map calibration; position quality. |
| **D — Navigation & Perception** | 5 | Existing navigation stack; destination-based route planning; path tracking; obstacle detection and stopping; safe recovery. |
| **E — Integration & Validation** | 3 | End-to-end integration; test cases; reliability metrics; safety checks; regression testing. |
| **Project Lead** | 1 | Permissions; parts; blockers; decisions; daily deadline tracking. |
| **Total** | **20** | |

### Critical safety gate

**Do not send autonomous commands to the real buggy** until the E-stop, safe stop, speed limits, and manual takeover have been demonstrated and signed off. Mapping, package reuse, and simulation can proceed in parallel while vehicle control is prepared.

---

## 3. Week 1 — Prove Feasibility and Run the Autonomy Stack

**Friday outcome:** The team has chosen a compatible software/hardware stack, proved that the vehicle-control approach is feasible, and run the navigation stack in simulation or replay.

### Team deliverables

- [ ] **A — Vehicle Control:** Inspect the existing buggy. Confirm whether steering, throttle, and brakes can be commanded electronically. Select the required interface and parts; bench-test signals; define the E-stop and manual takeover. Order missing hardware on Day 1.
- [ ] **B — ROS 2 & Sensors:** Install ROS 2 and drivers on the target compute. Demonstrate sensor data or recorded-data replay, timestamped logs, and a clean startup.
- [ ] **C — Map & Localization:** Survey the campus drivable-road network. Identify junctions, dead ends, restricted areas, and safe test zones. Produce an initial road-network map draft and localization plan.
- [ ] **D — Navigation & Perception:** Reuse compatible packages (for example, ROS 2/Nav2 where suitable). Demonstrate destination → route plan → path in simulation or replay. Audit sensor requirements for localization and obstacle detection.
- [ ] **E — Integration & Validation:** Create the end-to-end architecture, interface contract, safety checklist, test matrix, and baseline build. Identify what can be tested without moving the vehicle.
- [ ] **Project Lead:** Secure permissions, parts, a test area, and daily owners. Resolve purchase or compatibility blockers on the same day.

### Friday gate

There must be a proven path to safe electronic vehicle control, a working software demo, a campus road-map draft, and a list of exact missing items. If electronic control is not feasible, the four-week target is at risk.

## 4. Week 2 — Make the Real Buggy Move Safely Under Computer Control

**Friday outcome:** Real sensors and the vehicle computer are connected. After safety sign-off, the buggy can execute bounded, low-speed commands in a closed test area.

### Team deliverables

- [ ] **A — Vehicle Control:** Install the interface. Demonstrate low-speed forward, steering, and stop commands. Prove the physical E-stop and manual takeover before allowing autonomous output.
- [ ] **B — ROS 2 & Sensors:** Publish live sensor data to the software stack. Validate timestamps, transforms, and health monitoring. Record a complete sensor log while the buggy moves manually.
- [ ] **C — Map & Localization:** Build the first usable map and demonstrate stable position estimates in the test area. Measure localization drift and fix calibration errors.
- [ ] **D — Navigation & Perception:** Connect the planner/controller to the vehicle interface with output disabled first. After sign-off, run a supervised low-speed path test. Demonstrate obstacle-stop logic in simulation or controlled tests.
- [ ] **E — Integration & Validation:** Run end-to-end bench and closed-area tests for E-stop, stale sensor data, communication loss, command limits, and manual takeover. Track failures and assign owners.
- [ ] **Project Lead:** Keep scope fixed. Prioritize missing hardware and interface failures over new features.

### Friday gate

Achieve repeatable low-speed computer-commanded movement and reliable stopping. If this is not achieved, campus navigation by Week 4 is unlikely. **Do not skip safety gates.**

## 5. Week 3 — Navigate Between Different Destinations on the Campus Map

**Friday outcome:** The buggy accepts multiple start/destination pairs on the mapped road network, plans different routes, and drives them under supervision. This must not be one hard-coded path.

### Team deliverables

- [ ] **A — Vehicle Control:** Tune steering and speed limits for turns and junctions. Verify stopping distance and manual takeover at the chosen low speed.
- [ ] **B — ROS 2 & Sensors:** Keep sensor streams stable across full runs. Add diagnostics, restart checks, and useful fault logs.
- [ ] **C — Map & Localization:** Finish the mapped drivable-road network for the intended campus area. Validate localization at multiple junctions and route segments.
- [ ] **D — Navigation & Perception:** Enable destination-based global route planning across the map; track planned paths; configure obstacle response using existing compatible packages. Demonstrate at least **3 different destination pairs**.
- [ ] **E — Integration & Validation:** Run repeated trials across different routes. Record completion rate, interventions, localization loss, obstacle-stop behavior, and failures. Re-test fixes.
- [ ] **Project Lead:** Do not add new features after midweek unless they fix a blocker for the target.

### Friday gate

Multiple destination pairs must work in the mapped test area, and the system must stop safely when tested conditions require it. If route changes fail, Week 4 should focus on map/planner integration, not cosmetic features.

## 6. Week 4 — Expand Coverage, Harden, and Demonstrate

**Friday outcome:** Demonstrate destination-based autonomous travel across the validated mapped campus road network, with known limitations documented and a safety driver present.

### Team deliverables

- [ ] **A — Vehicle Control:** Inspect wiring and mechanics; freeze safe speed limits; verify E-stop, braking, and manual takeover before each trial.
- [ ] **B — ROS 2 & Sensors:** Freeze package versions and configuration. Validate cold startup, sensor health, and recovery after restart.
- [ ] **C — Map & Localization:** Validate all intended drivable-road segments and junctions. Mark unvalidated areas as unavailable; correct map/localization failures.
- [ ] **D — Navigation & Perception:** Fix high-impact route-planning, tracking, and obstacle-stop defects. Freeze features by Wednesday; do not add untested behavior before the demo.
- [ ] **E — Integration & Validation:** Run regression tests across varied start/destination pairs, turns, and tested obstacle cases. Save logs/video and publish measured results.
- [ ] **Project Lead:** Run the final checklist and operator briefing. Report exactly which road network and conditions were tested.

### Friday gate

Demonstrate only in mapped areas and obstacle cases that have actually been tested. Do not claim unrestricted campus autonomy if any sections or conditions remain unvalidated.

---

## 7. Parallel Work Plan

| Week | Team A: Vehicle | Team B: Sensors | Team C: Map | Team D: Autonomy | Team E: Test |
|---|---|---|---|---|---|
| **1** | Audit controls and parts | Install stack and prove data flow | Survey/map network | Route planning in simulation | Safety plan and test matrix |
| **2** | Safe real control | Connect live sensors | First map and position estimate | Integrate planner/controller | Closed-area fault tests |
| **3** | Tune turns and stops | Stabilize sensor streams | Expand/validate road map | Multi-destination navigation | Repeated route trials |
| **4** | Safety inspection | Freeze configuration and startup tests | Validate network coverage | Fix blockers and freeze features | Regression tests and demo evidence |

## 8. Sequential Safety Dependencies

### Work that can run in parallel from Day 1

- Package research and simulation
- Vehicle-control retrofit
- Sensor setup
- Map creation
- Test planning

### Work that must happen in order

1. Real autonomous actuator commands only after vehicle control, E-stop, and manual takeover are approved.
2. Multi-destination real driving only after localization and vehicle control are stable.
3. Expand coverage only after the current area works reliably.
4. Run the final demo only after regression testing.

## 9. Daily Cadence

| Time | Activity |
|---|---|
| **09:00–09:15** | Stand-up: each team states today's concrete output and blockers. |
| **09:15–12:30** | Build/configure independently; reuse proven packages rather than rewriting them. |
| **13:15–16:00** | Integrate and test on a bench or in an approved closed area. |
| **16:00–17:15** | Review code/logs. The lead marks every deliverable **DONE / BLOCKED / NOT DONE** and assigns an owner/date. |

## 10. Reality Check

Four weeks is an aggressive deadline, especially if the buggy needs major drive-by-wire modifications. Existing packages reduce software effort, but do not remove the need for sensor calibration, integration, real-world validation, or permissions.

- Keep speed low.
- Test in controlled areas first.
- Use a trained safety operator.
- Never remove the E-stop or manual takeover to save time.
