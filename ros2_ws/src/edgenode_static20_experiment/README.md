# Phase B static avoidance with preserved A NORMAL safety

This explicit package imports the preserved v61 and completed A modules.
Neither their source nor their default launch/profile changes. The B launch
selects this node; `use_v61_baseline:=true` selects the original implementation.

NORMAL uses A's current/confirmed observation memory, body/map checks and early
speed cap. Unconfirmed hazards participate in braking immediately. A current path conflict starts the original two-source-frame confirmation
without waiting for a second independent surface-shape confirmation. Retained
confirmed static occupancy can also request preparation; vanished provisional
returns alone cannot. During
its maneuver all original LiDAR, whole body, B2/C3, road containment, steering,
lateral acceleration, PP rollout and fresh-source gates remain in effect.
The A memory also receives active observations for a safe NORMAL handoff.

Native continuation uses only the actual directed MGeo join. A natural join
must preserve the ordered mandatory waypoints before that join within the
original 1 m limit after reserving the configured tracking error. Otherwise
the original complete return proof is required. Original geometric selection,
whole-maneuver checks and committed-lane ownership remain unchanged.

Road-offset search considers requested speed, then 15 and 10 km/h when lower
than requested. Each higher-speed probe has at most 3 s; the last lower probe
uses the remainder of the same global 12 s entry episode. Every probe uses all
original geometry and PP checks at its candidate speed. A computation timeout
is not labeled physical impossibility. A hard map/input rejection remains a
hard rejection. Only a complete proof may be selected. Worker adoption accepts
only an avoidance-speed config difference, waits for actual speed to be low
enough and still runs the original current-input validation. Rejected adoption
restores the previous config. NORMAL releases the selected lower speed and
restores the requested ceiling for the next obstacle.

The ROS profile speed is a request/ceiling. Runtime debug and
`/planning/phase_b_runtime` distinguish requested and effective speed. Passing
an Offset at 10 km/h is a safe lower-speed result, not a 20 km/h performance
success. Native lane cases retain their normal requested-speed search.

Search worker snapshots exclude the node-bound A guard/lambda; only the live
node owns that guard. v61 base, A guard and B implementation manifests are
reported separately. Actor API data is never an algorithm input.

See `docs/s4_b_static20.md` and the separate result root
`/workspace/local_data/s4_b_static20_20261119/` for actual validation status,
failed captures, profiles, logs, command lines and completion decision.

Every pending search is limited to its reserved original-lane approach. No
reserved distance means braking, not drifting past the captured search origin.
This preserves the live TRACKING_ERROR and stopping-distance gates.

The B node retains serialized Planning/input/service callbacks. It cancels and
reschedules its planning timer after each callback: nominal 100 ms start-to-start
when computation fits, otherwise at least 20 ms of idle time for other callbacks.
This prevents a continuously ready overrun timer from starving parameter/reset
services. It changes neither source timestamps nor the 0.6 s watchdogs.

A static obstruction already confirmed by the original two distinct source
frames is retained while NORMAL waits for the speed-based detection trigger.
A temporarily missing LiDAR face must not erase that preparation proof.
Single-frame convex history is still discarded. On completion of a fixed
maneuver, the old core confirmation episode is cleared; the A guard continues
to own exact observed occupancy for the next NORMAL segment. Failed v4/v5
traversals with computation-induced standstill are not counted as qualified.

Lower-speed Offset proofs use min(captured speed, candidate ceiling) as their
conditional initial speed. Adoption still waits for measured speed below that
ceiling and validates the original live pose/speed/curvature. This avoids
requiring a10 km/h proof to begin at the pre-braking20 km/h state.
Higher probes remain at most3 s. When alignment has consumed episode time,
intermediate probes are shortened to reserve up to4 s for the final complete
lower-speed proof. The original total12 s episode deadline remains unchanged.

Speed and strategy are coupled at lower tiers too. After the requested native
lane and Offset fail, a15/10 km/h tier first regenerates a native entry and
complete return at that speed, then falls back to its Offset within the same
per-tier deadline. A native20 return rejection does not prove that native15
is impossible. No native-only replay may recursively invoke Offset search.
The selected lower native proof uses the same measured-speed wait/live checks.


The B launch optionally selects SourceTimeLocalizationNode (defaulttrue in the
explicit B launch). It reuses the original EKF equations and reset interface,
replays late GPS in a bounded0.8 s history, and measures current ROS vehicle
speed. Position comes from GPS/IMU; Actor API pose/velocity is never an input.
The separate localization_source_time.yaml uses GPS std0.05 m and vehicle
speed std0.1 m/s for the preserved MORAI sensor configuration. All three
sources must remain within0.6 s, with the original0.05 s future tolerance;
output headers retain the actual filter sensor epoch. Source-time diagnostics
are recorded in the control-detail bag. source_time_localization=false or
use_v61_baseline=true selects the untouched original localizer/profile.

When the requested avoidance speed leaves less origin reserve than braking
needs, the B entry worker can reserve a conditional lower-speed approach
using the existing map/kinematic estimator. Preparation is capped at10 km/h.
This origin heuristic grants no geometry permission; complete proofs and
live adoption remain mandatory. NORMAL60/sharp30 and body/margins stay fixed.

B normal_clearance.py also extends only failed/missing optional normal-center
cache entries. Existing successful A entries remain unchanged. A small C2
normal-reference correction preserves each native endpoint and its first/second
derivatives. The entire link body sweep plus15cm tracking reserve must clear
all forbidden B2 and physical C3 and stay inside the pinned measured road mask.
Native-link/node/waypoint order stays unchanged. Curvature stays inside the
vehicle steering bound and cannot exceed the native peak above the ordinary
.08/m floor; the unchanged normal speed planner still limits acceleration.
Unproved candidates retain the original reference and live stop gates. No
obstacles/Actor truth/location-specific branch is used by this map-only step.

Search pacing retains the same12s episode and original reaction/braking bound.
The admitted approach is spread over the remaining episode, also respecting
the previous solve horizon, so a first slow multilevel solve does not consume
its reservation under an unsupported2s completion assumption. Zero support
still means zero speed. This changes only waiting motion; complete proofs and
live adoption remain mandatory. The recorded v11 full mission reached the
goal safely but failed performance due2.396s standstill and is kept as failure.

Normal centering is prepared for the configured startup mission before ROS
callbacks run. A later alternate goal does not expand that B preparation;
new links use the preserved original correction and live guards. The observer
recomputes this same startup scope. An earlier alternate-goal verifier wrongly
centered an extra link and rejected the1479-point live path before arming;
the scope mismatch, wrong1592-point expectation and exact reproduction are
retained in v12-natural-prearm-verifier-scope-forensics.json.
