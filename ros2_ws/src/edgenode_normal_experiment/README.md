# Phase A NORMAL safety adapter

This opt-in package preserves the v61 Planning, Control, Localization and
Perception packages. `NormalPlanningNode` retains their normal mission,
source-time, FSM, speed and Control interfaces, closes lateral search workers,
and replaces only the static motion planner with `NormalSafetyPlanner`.
The original normal path remains the commanded path. Entry/return methods
raise if called. The package cannot execute lane changes or Offset maneuvers.

The running mode is `NORMAL_SAFETY_ONLY / NONE_PHASE_A`. The bootstrap ROS
parameter `static_avoidance_method=lattice` only permits construction of the
unchanged base node. `/planning/phase_a_runtime` publishes the separate A
implementation manifest; the v61 SHA describes the unchanged imported base.

`NormalObservationMemory` is the active memory. Every current return enters
safety immediately. Two consecutive distinct source frames confirm a static
track. Confirmed occupancy survives missing views and occlusion; it retires
only after the whole shape is behind the passed route. An unconfirmed return
can retire after at least 0.6 s and three fresh views with no current foreground
occluder. A repeated timestamp or missing/stale frame cannot advance that
process. Occupancy retains the observed shapes without filling their convex
hull. STRtree queries preserve exact shape distances and intersections.
`ExactObservedTiles` remains a prototype geometry test utility and is not used
by `NormalSafetyPlanner.plan`.

The complete measured body and its motion sweep must remain in the proven
road and clear forbidden B2 and physical C3 boundaries. Existing directional
merge certificates are checked again. Actual body clearance 0.25 m, axle
clearance 1.20 m and padding 0.10 m remain. The earliest full-body or axle
conflict along the original path defines travel available for stopping. The
hard stop uses the original reaction time 0.3 s and deceleration 4 m/s².
An earlier speed cap uses reaction 1.1 s and a 2 m reserve; the retained-occupancy
visibility cap adds the unchanged worst source age 0.6 s. Current-body danger,
invalid input and insufficient stopping distance latch a safe stop. Sensor
freshness and Control watchdogs remain unchanged. No Actor truth is read here.

An obstacle case is separate from an obstacle-free A driving success. The
unchanged Control explicitly holds brake 0.65 for target speed <=0.05 km/h;
`tools/s4_normal_stop_audit.py` checks that threshold with raw Ego standstill
before supervisor intervention. It does not relabel a blocked driving mission
as successful normal driving.

Use the dedicated launch and profile documented in `docs/s4_a_scurve.md`.
MORAI results are under `/workspace/local_data/s4_a_scurve_20261119/`.
