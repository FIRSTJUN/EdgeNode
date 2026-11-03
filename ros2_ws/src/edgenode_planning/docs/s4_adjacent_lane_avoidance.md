# S4 MGeo adjacent-lane avoidance / RViz 정리 보고서

기준: `/workspace`, `k-city`, HEAD `81a2add`와 작업 시작 당시 미커밋 변경. 분석 bag: `logs/kcity_s4_static_obstacle_20kph_holdfix_01`.

**실제 MORAI 주행·commit·push를 하지 않았다.** 원본 bag은 read-only SQLite로 읽었다. steering clamp 0.70, curvature limit **0.16046692048050848 /m**, required clearance **1.20 m**를 유지했다. Localization/Dijkstra/Control/PID/Dynamic Lookahead/Perception self-mask와 MGeo 원본 geometry를 변경하지 않았다. 기존 `lattice_planner.py`도 이번 단계 시작 snapshot과 동일하다.

새 구현의 obstacle clearance는 여전히 관측 footprint 점에 대한 reference-path 거리다. road gate는 실제 두 MGeo 중심선 사이의 **보수적인 Ego reference-center corridor**다. **검증된 차도 경계 또는 Sonata full-body swept clearance라고 주장하지 않는다.** 해당 자료가 없으므로 body/outer-lane boundary 검증은 아래 H/L/U의 후속 요구사항이다.

이번 source 변경: `adjacent_lane_planner.py`, `lane_geometry.py`, `planning_visualization_node.py`, `vehicle_geometry.py` 신규; `planning_node.py`, planning.yaml, setup.py, route_editor.launch.py, RViz 설정, Route Editor 배경선 스타일, 관련 테스트와 self-contained bag/MGeo fixture, 이 보고서. 기존 vehicle/odometry callback은 그대로다. 작업 전 미커밋 수정은 보존했다.

분석 재현 코드와 CSV/JSON/그림은 `logs/kcity_s4_adjacent_lane_analysis/`에 있다. 주요 코드: `analyze.py`, `measure.py`, `clearance_and_plot.py`, `replay_adjacent.py`, `kinematic_check.py`.

## A. 실제 bag의 경계 부근 진행

bag은 16.010288037초, 4,696 messages, planning cycle 55개다. 첫 target=20은 t=6.271784794 s, BLOCKED/target=0은 t=10.692919958 s다. 최고 Ego status speed는 약 20.315 km/h이며 승인 avoidance 목표는 전 구간 -1.8 m였다. runtime cruise/curve/sharp/max_drive/recommended=20은 정상이다.

동일한 original-lane signed road-normal 좌표에서 정지 구간을 비교했다.

| 항목 | 값 |
|---|---:|
| Original MGeo lane center | d=0 |
| Right adjacent center | d≈-3.475 m |
| 두 중심의 midpoint proxy | d≈-1.737 m |
| 마지막 승인 목표 | d=-1.800 m |
| 마지막 보고 EgoVehicleStatus position | d≈-1.997 m |
| 마지막 Localization position | d≈-2.004 m |
| 마지막 관측 NPC footprint lateral extent | d≈[-0.935, +0.375] m |

즉 -1.8 목표는 midpoint proxy를 우측으로 약 **6 cm** 지난 정도이고, 정지 Ego position도 약 **26 cm** 지난 정도다. 인접 중심까지는 여전히 약 **1.48 m** 부족했다. 여기서 midpoint는 **실제 차선 도색선을 측정한 값이 아니다.** bag에 차선 경계/카메라/차도 polygon이 없어 물리적인 도색선 점유량이나 차체가 경계를 몇 cm 물었는지는 확정할 수 없다.

793개 Ego status 표본과 각 시각의 관측 near footprint를 비교하면, 보고 Ego reference point가 관측 footprint의 front/rear 사이에 들어간 표본은 없다. 정지 마지막 표본은 original progress s≈21.400 m, NPC front≈23.543 m로 약 2.143 m 앞이다. 따라서 ‘Ego 기준점이 NPC와 정확히 나란히 된 시점’의 수치를 만들지 않았다. 물리적 pose 기준점/차체 길이가 미확정이므로 차체 전단이 닿았는지와 reference point가 나란히 있는지는 구분해야 한다.

관측 footprint에 대한 최소 보고 Ego point 거리는 약 **2.393667 m**(t≈10.826913 s)이며, 마지막은 약 **2.417881 m**다. 이는 full-body clearance가 아니다. 마지막 lateral 간격만 보면 Ego point와 관측 NPC 우측 edge 사이가 약 1.062 m다. body 길이/기준점/보이지 않은 NPC 면 없이 실제 차체 최소 간격은 계산할 수 없다. 사용자 관찰의 충돌을 부정하지 않지만 collision event/NPC ground truth가 이 bag에 없어 충돌 시각·차체 접촉 위치를 독립 확정하지 않았다.

BLOCKED 이후 old `/planning/local_path`는 미승인 원래 road reference를 publish하던 구조여서, 그 선과 관측 점의 최소 거리를 실제 승인 회피 경로 clearance로 사용하지 않았다. 분석 CSV는 status를 함께 보존한다.

증적: `lane_measurements.csv`, `ego_gt_clearance.csv`, `clearance_metrics.json`.

![Recorded positions, old target and new actual-lane target](/workspace/logs/kcity_s4_adjacent_lane_analysis/lane_geometry_comparison.png)

## B. ±2.4 m와 실제 adjacent center

시작 original link `A219BS010415`의 우측 중심 간격은 약 **-3.471 m**, NPC 구간 `A219BS010419`의 우측 중심 간격은 약 **-3.475 m**다. 실제 link geometry로 계산했으며 3.5 m 상수를 목표로 넣지 않았다.

- -1.8 m 목표는 실제 인접 중심보다 약 1.67 m 부족하다.
- 기존 최대 -2.4 m도 약 1.07 m 부족하다.
- 새 구현은 ±2.45/±2.50 등 offset을 추가하는 방식이 아니다. legacy offset 후보 배열은 호환 모드에만 남겨두고, 기본 `adjacent_lane` 모드는 실제 선택된 MGeo centerline을 사용한다.

## C. 사용한 MGeo 정보

실제 JSON의 ordinary/lazy link 352개 중 Dijkstra의 유효 ordinary graph는 254개, lazy lane-change record는 98개다. 실제 필드는 `left_lane_change_dst_link_idx`, `right_lane_change_dst_link_idx`, `from_node_idx`, `to_node_idx`, `lazy_init`, `lane_ch_link_path`, `max_speed`, `points`다. road_id/ego_lane/lane_change_dir는 이 데이터에서 비어 있어 추정값을 만들지 않았다.

선택된 우측 pair는 `415→416`, 다음 original route section에서는 `419→420`, `632→633`이다(접두사 모두 A219BS010). 다음 pair도 실제 node 연결 및 공유 geometry 끝점이 일치해야 이어 붙인다. 420의 다른 outgoing branch를 임의로 선택하지 않는다.

조건: ordinary valid link, finite positive max_speed, 명시된 좌/우 neighbor 및 reciprocal relation, 같은 진행 방향, 올바른 signed side, 실제 geometry overlap, 연결된 successor pair. 없거나 모호하면 BLOCKED다. lazy link의 추상 두 점을 주행 경로로 쓰거나 Dijkstra edge로 추가하지 않는다. 명시된 neighbor metadata를 lane-change 연결 근거로 사용했으며, 실제 차선 도색/교통상 허용 여부까지 확인한 자료는 아니다.

## D. Target 계산

새 `LaneNetwork/LanePair`가 original center의 road-normal과 adjacent polyline의 실제 교점을 구한다. 단순 nearest-point distance나 상수 lateral shift가 아니다. 분리 거리가 변하거나 곡선인 경우에도 실제 adjacent geometry의 점/방향을 endpoint로 사용한다.

world XY quintic Hermite가 현재 position/yaw와 이전 승인 경로의 curvature/derivative boundary를 선택된 adjacent endpoint 및 그 실제 segment tangent에 연결한다. MGeo가 piecewise linear이므로 segment 내부 endpoint curvature는 0이며, 전체 출력 polyline의 map vertex 곡률도 별도 검사한다. 측정하지 않은 실차 curvature를 yaw-rate/v로 만들어 넣지 않는다.

기본 정책 `lane_change_min_duration_sec=3.0`은 신규 planning policy다. 초기 transition은 max(측정 속도, 가능한 runtime recommendation 상한을 max_drive/avoid cap으로 제한한 속도)에 대해 v×3 s와 기존 geometry 최소 길이 중 큰 값을 사용한다. 20 km/h에서는 최소 16.667 m다. front deadline/실제 horizon 안에 끝낼 수 없으면 BLOCKED이며 속도나 curvature gate를 무시해 줄이지 않는다. 이 3초는 검증된 Sonata 동역학 규격이 아니며 별도 검증이 필요하다.

승인 후에는 같은 3초를 매 cycle 새로 시작하지 않는다. 승인 polynomial의 closest world parameter, 현재 yaw, world 완료 station과 원래 global progress를 유지한다. road arc와 avoidance-curve parameter를 구분하며, pose jitter에서도 완료점을 보존한다. explicit global progress가 없는 순수 함수 호출은 고정된 original pair world station을 사용한다.

## E. 상태 구조

`ENTRY → ADJACENT_LANE_HOLD → RECOVERING → CLEAR`다. 공개 lattice status는 ENTRY/HOLD 동안 기존 AVOIDING을 유지하며 상세 단계는 `/planning/lane_change_state`에 발행한다.

HOLD는 adjacent MGeo centerline suffix 자체다. 허용되는 작은 tracking correction은 같은 centerline으로 연결하는 boundary transition이며 새 큰 offset 목표를 만들지 않는다. 선택 pair는 active maneuver 동안 고정하고, Map Matching이 adjacent link로 바뀌어도 original Dijkstra route를 바꾸지 않는다. 기존 방향 continuity와 3-cycle BLOCKED release를 유지한다.

실제 Ego signed progress가 remembered rear + 0.5 m를 strict 초과해야 RECOVERING이 가능하다. equality에서는 HOLD다. RECOVERING도 world boundary를 보존해 original center로 복귀하며 동일 gate를 통과해야 한다.

## F. 세 gate와 한계

1. Obstacle-safe: 현재 전체 관측 점을 검사한다. 관련 near 관측이 소실되고 rear가 남았으면 기억한 associated footprint도 유지한다. unrelated far group은 extent에 합치지 않지만 collision set에서는 제거하지 않는다. required=1.20 m 유지.
2. Steering-feasible: 연속 world quintic의 곡률 극값과 출력 world polyline 곡률의 최댓값이 기존 0.16046692048050848 이하이어야 한다. polynomial speed가 0인 퇴화와 연속/샘플 전진 방향 위반도 거부한다. clamp 0.70 유지.
3. Center-corridor-safe: 실제 original/adjacent 중심선을 잇는 convex quad들의 union이 **모든 출력 선분 전체**를 덮어야 한다. 점 몇 개만 검사해 corridor 사이 구멍을 건너뛰지 않는다. map folding/끊김/잘못된 방향/외부 excursion은 거부한다.

세 번째는 actual road polygon이나 Ego body sweep이 아니다. actual outer boundary/width가 없으므로 center-to-center strip 바깥을 임의로 주행 가능하다고 넓히지 않았다. 특히 실제 Ego가 adjacent center 바깥쪽으로 수 cm overshoot하면 이 보수적인 gate에서 거부될 수 있다. 이 제한을 없애려면 verified boundary/corridor 자료가 필요하다. fixed-path kinematic 추종 오차가 작다는 이유만으로 gate를 완화하지 않았다.

새 mode에서 safe candidate가 없으면 target=0이고 **canonical `/planning/local_path`도 empty**다. 미승인 original road fallback을 실제 추종 경로처럼 발행하지 않는다. 이 동작은 알고리즘의 fail-safe 변경이며 시각화 mirror가 Planning 데이터를 수정한 것이 아니다.

세 gate의 별도 Bool 진단은 `/planning/lane_change_obstacle_safe`, `/planning/lane_change_steering_feasible`, `/planning/lane_change_corridor_safe`다. phase CLEAR 등 미평가 상태의 false는 해당 lane-change gate가 승인되지 않았다는 뜻이며, 검증된 full-body safe라는 Bool은 만들지 않았다.

## G. 예상 경로와 오프라인 20 km/h 점검

충돌 bag 첫 관측 입력을 고정해 새 planner에 넣으면:

| 항목 | 결과 |
|---|---:|
| 선택 | right / A219BS010416 |
| actual adjacent endpoint separation | -3.474671 m |
| entry length | 16.666667 m |
| observed obstacle front | 17.520014 m |
| 관측 점 clearance | 2.979798 m |
| continuous maximum curvature | 0.073467 /m |
| center-corridor gate | 통과 |

이는 그 frozen input에 대한 기하 검증이다. 실제 bag을 새 경로로 재주행한 결과는 아니다. pure planner 한 번의 측정 시간은 약 32~36 ms였으며 end-to-end 20 Hz 또는 worst-case deadline을 보장하는 benchmark는 아니다. exact nearest-segment 계산의 AABB pruning으로 반복 투영 비용을 줄였고 원래 투영식과 동등성을 회귀로 확인했다.

기존 PurePursuit/2.84 wheelbase/0.70 clamp와 실제 config의 dynamic lookahead(20 km/h에서 2.5 m)를 그대로 사용한 별도 **fixed-path kinematic bicycle** 점검도 했다. NPC 관측 전단 station 부근에서 old -1.8 경로는 adjacent center와 약 1.678 m 차이, 새 경로는 약 **0.045 m** 차이였다. 새 모델의 |steer| 최대≈0.299로 clamp 이하였다.

이 모델은 pose를 rear axle로 가정하고 일정 속도를 사용한다. actual Localization 물리 기준점, PID/타이어/지연/body sweep 및 closed-loop replanning을 검증한 것이 아니다. 수 cm overshoot와 F의 보수적인 corridor gate를 실제 정지/입력 replay에서 함께 확인해야 한다. 실제 MORAI 20 km/h 통과 성공을 주장하지 않는다.

## H. Full-body geometry

| 항목 | 확인 상태 |
|---|---|
| wheelbase | 기존 Control 설정 2.84 m, planner와 동일. 이번에 변경하지 않음 |
| width | half-width 0.95 m는 기존 provisional 값이며 actual VehicleSpec 확정치 아님 |
| length | 확정 자료 없음 |
| front/rear overhang | 확정 자료 없음 |
| localization/base_link physical reference | frame 이름은 base_link이나 rear axle/body center/GPS 기준과 변환 미확정 |

`VerifiedVehicleGeometry`는 향후 oriented rectangle/swept-body 입력 계약이다. 모든 치수/pose-to-rear-axle transform/provenance가 있어야 생성되고 누락/비정상/불일치는 거부한다. **live collision gate에 연결하지 않았다.** 테스트 숫자는 synthetic이며 Sonata 값이 아니다. 기존 빨간 arrow의 4.0×1.5 표시 크기도 차량 규격으로 사용하지 않는다.

필요 자료: 선택 차량 ID/mesh와 실제 MORAI VehicleSpec 원본 응답, size 축/단위, `overhang` 필드 의미, rear_overhang, pose 물리 기준점과 rear axle transform, sensor extrinsic, NPC full-body/uncertainty 및 lane/road boundary polygon. 현재 MGeo에는 node/link/global_info만 있어 outer boundary를 임의로 만들지 않았다.

## I. Regression

기존 626 pass 대비 신규 39개가 추가돼 총 665 pass다. 좌/우 adjacent, 없음/잘못된 direction/reciprocal/ordinary link, 실제 separation, 속도 의존 20 km/h entry, deadline, exact adjacent HOLD, rear strict gate/복귀, adjacent obstacle veto, corridor 전체 선분/구멍/초기 heading 위반, world curvature dense 비교, progress/suffix/jitter, 실제 MGeo+bag fixture를 포함한다.

기존 최초/최종 unsafe BLOCKED scene은 새 mode에서도 계속 BLOCKED다. runtime 3.5→20 및 실제 ROS parameter storage 회귀, old HOLD/continuity/far association, Planning/FSM/Control/Localization/Perception 회귀를 유지했다. visualization 테스트는 별도 파일로 exact source geometry, duplicate/order 보존, stale cleanup, route replacement, source invalidation, ROS clock rewind, actual ROS marker transport를 검사한다.

## J. 전체 테스트

**665 passed, 1 skipped, 2 warnings**. skip은 기존 copyright 검사, warnings는 기존 logger entry-point deprecation이다.

```bash
cd /workspace/ros2_ws/src/edgenode_logger
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q /workspace/ros2_ws/src
```

logger ament lint의 cwd 범위를 해당 package로 유지하면서 모든 ROS source 테스트를 전달했다. deselect/추가 skip은 없다. 실제 ROS transport 테스트는 namespace/topic/domain을 격리했다. 증적: `pytest.txt`.

## K. Build / diff

`cd /workspace/ros2_ws && colcon build --symlink-install`: **7 packages finished**. `git diff --check` 통과. source branch/HEAD 유지, commit/push 없음. 증적은 `build.txt`, `diff_check.txt`, `verification.json`이다.

## L. 다음 MORAI 정지 검증

실행하지 않은 다음 단계 절차다.

1. enable_drive=false, 실제 차량 정지, mode=adjacent_lane을 확인한다. 세 runtime speed/max_drive/avoid cap이 의도한 값인지 읽고 camera/RViz marker의 timestamp도 확인한다.
2. original/current/selected adjacent ID와 side/separation을 읽는다. 이 scene의 목표는 우측 약 3.47 m다. 실패 후보의 evaluated offset을 approved selection으로 해석하지 않는다.
3. NPC가 있고 충분한 deadline이 있을 때 white Local Path가 cyan ENTRY를 통해 실제 yellow adjacent center까지 도달하는지 확인한다. 단순 -1.8/-2.4 선에 HOLD하지 않아야 한다.
4. rear 미통과 정지 입력에서 purple HOLD를 확인하고, recorded Ego pose 입력을 이용한 offline/replay에서 gate strict 초과 후에만 pink RECOVERING을 확인한다. 실제 차량을 움직여 검증한 것으로 대신하지 않는다.
5. no adjacent/wrong direction/obstacle/corridor violation 입력은 BLOCKED/target=0/empty Local Path여야 한다. 기존 BLOCKED 해제는 안전 확인 3 cycle을 유지한다.
6. 실제 주행 전 H의 physical geometry/road boundary 자료와 연속 pose tracking 및 source-to-command latency/cycle budget을 검증한다. 현재 center-strip 밖 tracking overshoot를 단순 허용 margin으로 덮지 않는다.
7. 다음 bag에는 `/parameter_events`, current_link, local_path_progress_m/index, 모든 evaluated candidate 수치, 각 lane-change diagnostic Path/Bool/state, odometry, Ego status, raw LiDAR/footprint/count, ctrl_cmd와 collision event/NPC ground truth를 가능한 범위에서 포함한다.

## M. 기존 RViz 구조와 문제점

- RouteEditorNode의 `/route_editor/mgeo_links`, `/route_editor/mgeo_nodes`, `/route_editor/waypoint_markers`가 static/draft MarkerArray를 발행했다.
- `/route_editor/preview_path`는 **draft의 표시용 forward slice**이고 live Dijkstra route가 아니다. 기존 설정에서 항상 표시돼 실제 route와 함께 겹쳤다.
- PlanningNode의 `/planning/global_path`와 `/planning/local_path`는 native Path display였다. MGeo 폭 0.35 m와 Global 폭 0.5 m 차이가 작고, 기존 camera scale=0.75의 도시 overview에서는 lane 간격이 작게 보였다.
- `/route_editor/vehicle_marker`는 기존 빨간 arrow다. valid pose 업데이트/invalid pose DELETE는 있었지만 transient-local, lifetime=0이라 source 종료 시 마지막 marker가 남을 수 있었다.
- 기존 obstacle footprint 전용 Marker publisher는 없었다. source PoseArray와 centroid/count debug만 있었다.

## N. 정리한 topic / marker

canonical Planning/Control 토픽은 그대로다. 새 `PlanningVisualizationNode`는 source 메시지를 받아 **복사만 하는 독립 diagnostic publisher**이며 Control/Planning input을 발행하지 않는다. Route Editor launch에서 같은 mgeo_dir로 실행한다.

| Source | Display topic | namespace / 용도 |
|---|---|---|
| `/planning/global_path` | `/planning/global_path_marker` | global_path / exact Dijkstra route |
| `/planning/local_path` | `/planning/local_path_marker` | local_path_debug / exact Control input |
| `/planning/current_link` + same MGeo records | `/planning/current_lane_marker` | current_lane / actual map-matched link |
| `/planning/original_lane_center` | `/planning/original_lane_marker` | original_lane / frozen maneuver origin |
| `/planning/adjacent_lane_center` | `/planning/adjacent_lane_marker` | adjacent_lane / approved selected pair |
| `/planning/lane_change_entry_path` | `/planning/lane_change_entry_marker` | lane_change_transition |
| `/planning/adjacent_lane_hold_path` | `/planning/adjacent_lane_hold_marker` | adjacent_lane_hold |
| `/planning/lane_change_recovery_path` | `/planning/lane_change_recovery_marker` | lane_change_recovery |
| `/planning/lane_change_recovery_gate` | `/planning/recovery_gate_marker` | lane_change_recovery_gate / reference rear+margin crossbar |
| `/planning/obstacles` | `/planning/obstacle_footprint_markers` | obstacle_footprint |
| `/route_editor/vehicle_marker` | `/planning/ego_vehicle_marker` | ego_vehicle / unchanged red arrow geometry |

추가 scalar diagnostics는 lane_change_state, lane_change_side, selected_adjacent_link, adjacent_lane_separation과 세 gate Bool이며 모두 `/planning/` 아래다. 후보가 실패하고 아직 승인된 선택이 없으면 selected ID/center는 비워두고 evaluated candidate 수치는 별도로 보존한다. active maneuver의 승인된 선택을 failed candidate로 덮어쓰지 않는다.

## O. MGeo와 실제 Global Path 구분

MGeo 배경은 **0.035 m / 낮은 alpha**, Global은 **0.28 m green**, Control Local은 **0.32 m white**다. MGeo Marker의 좌표·link 순서·valid set은 변경하지 않고 스타일만 바꿨다.

bag의 Global Path는 54 messages 모두 동일 geometry였다. 실제 ordinary link 16개를 순서대로 이어 다시 조립하면 **1,313 points가 정확히 일치**한다. node 연결 15곳의 geometry gap은 모두 0, 중복 directed XY segment=0, consecutive duplicate XY=0, 최대 sample 간격≈0.5 m다. 다른 MGeo link를 Global처럼 publish한 증거는 없다. 감사 내용은 `clearance_metrics.json`이다.

visualizer는 route search/trim/resampling/중복 제거를 하지 않는다. 실제 source의 intentional revisit/duplicate도 그대로 표시한다. Global mirror를 쓰는 이유는 데이터 변경이 아니라 TTL과 stale 삭제를 위한 것이다.

## P. Current / Original / Adjacent lane

Current는 실제 map-matched ID로 같은 MGeo ordinary link를 읽는다. Original은 maneuver가 시작된 original route pair이며 map match가 adjacent로 넘어가도 바꾸지 않는다. Adjacent는 승인된 실제 centerline chain이다. successor section에 들어가면 selected adjacent ID도 해당 section의 ID로 진단한다.

Current/Original/Global이 같은 선에 있거나 HOLD 중 Current/Adjacent가 겹치는 것은 동일 geometry를 나타내는 정상적인 중첩이다. 표시를 보기 좋게 하려고 XY를 옮기지 않았다. 역할별 display를 선택적으로 켜서 확인할 수 있다.

## Q. ENTRY / HOLD / Recovery 표시

cyan ENTRY, purple adjacent HOLD, pink RECOVERY의 굵은 underlay 위에 **최종 Control Local을 white**로 표시한다. ENTRY 중에는 진입 구간과 그 뒤 adjacent center HOLD suffix를 함께 볼 수 있다. RECOVERING 경로는 실제 rear gate 통과 후 승인될 때만 보인다.

정지 상태에서도 복귀 조건 위치를 알 수 있도록 muted crossbar로 **remembered reference rear+margin gate**를 표시한다. 이는 미래 복귀 경로를 미리 승인한 것이 아니며 bumper/full-body rear clearance 선도 아니다. 실제 pink 복귀는 gate 조건 이후 생성된다. inactive/blocked stage Path는 empty로 보내 이전 stage underlay를 지운다.

## R. Obstacle footprint

Perception이 실제 Planning에 준 ordered 8점 그룹을 그대로 닫힌 outline으로 표시한다. far group도 생략하지 않는다. 그룹 수가 줄거나 empty면 DELETEALL 후 현재 frame을 다시 표시한다. grouping 계약이 없거나 count가 맞지 않으면 points로 표시하며 임의 rectangle/object를 만들지 않는다.

이 outline은 센서 관측 AABB proxy이며 NPC full-body ground truth가 아니다. source message는 수정하지 않고 Planning 판단에도 새 Marker를 쓰지 않는다.

## S. Durability / lifetime / stale

| 항목 | 정책 |
|---|---|
| MGeo roads/nodes | 기존 transient-local depth 1, lifetime=0, 시작 시 DELETEALL. static map으로 의도적 보존 |
| Waypoints / draft Preview | 기존 retained draft. clear 시 empty/DELETEALL. 기본 display는 current route와 분리 |
| Canonical Global/Local/diagnostic Path | volatile depth 10. mirror가 source 좌표를 복사 |
| 새 dynamic Marker outputs | volatile depth 10, 기본 freshness 최대 1.0 s, 고정 namespace/id |
| Footprint outputs | DELETEALL + 현재 그룹, finite lifetime |
| Original red vehicle source | publisher/topic/color/pose/scale/callback 변경 없음 |
| Red vehicle display mirror | source stamp age 검사, finite lifetime. 오래된 transient snapshot이면 DELETE |

source empty/invalid frame/input loss/route replacement 시 explicit DELETE를 보낸다. route epoch 이전의 delayed Local/stage Path는 거부한다. steady-clock timer가 source timeout을 삭제하고 ROS clock rewind에서는 epoch를 재설정한다. source timestamp와 ROS clock이 일치해야 하며 GUI를 재시작했다고 오래된 arrow를 fresh로 취급하지 않는다. native retained source를 수동으로 다시 켜면 원래 source의 보존 정책이 적용됨을 구분해야 한다.

## T. RViz display 구성

`kcity_route_editor.rviz`를 재사용하고 다음 순서로 정리했다.

01 MGeo Roads → 02 exact Global → 03 Current / 03b Original → 04 Selected Adjacent → 05 **CONTROL follows WHITE** → 06a ENTRY / 06b HOLD / 06c RECOVERY / 06d reference recovery gate → 07 Footprints → 08 Fresh red Ego → 09 MGeo nodes → 10 Waypoints → 11 **Draft Preview - NOT active route**.

MGeo nodes와 draft Preview는 기본 off다. Publish Point와 기존 route-editing services는 그대로다. 기본 camera는 S4 scene `(142,1444)` / scale 20으로 설정해 실제 3.47 m 차선 간격을 읽을 수 있게 했으며 pan/zoom으로 전체 map 편집이 가능하다. camera/style은 Planning geometry에 영향을 주지 않는다.

메시지/config/실제 ROS transport는 테스트했지만 headless 환경에서 실제 RViz GUI를 열거나 MORAI 화면을 검증하지 않았다. 그림은 offline geometry plot이며 RViz screenshot이 아니다.

## U. 실제 주행 전 정지 RViz 체크리스트

- route_editor.launch의 mgeo_dir가 Planning과 같고 visualization node가 한 개만 발행하는지 확인한다. 기존 빨간 arrow source와 fresh mirror를 중복 display하지 않는다.
- map fixed frame / ROS clock·source timestamp를 맞춘다. bag replay에서 `/clock`을 쓰면 관련 node의 use_sim_time을 맞춘다.
- grey thin roads는 background, green은 original Dijkstra route, white만 실제 final Control Local인지 확인한다. magenta draft Preview는 active route가 아니다.
- Current link와 Original context가 맞고 selected adjacent ID/side/separation이 실제 이웃 lane과 맞는지 확인한다. 이 scene의 right center는 약 3.47 m다.
- cyan entry 끝이 yellow adjacent center 위에 정확히 놓이고, purple HOLD가 lane boundary proxy가 아닌 adjacent MGeo center를 따라가는지 확인한다.
- NPC outline, red reported Ego position, white Local, yellow center와 recovery crossbar를 함께 본다. reference point/arrow를 full Sonata body로 오해하지 않는다.
- recorded pose 기반 정지 replay에서 rear+margin 이전에는 pink return이 없고 strict 초과 후에만 original lane으로 돌아오는지 확인한다.
- no safe candidate이면 white Local/stage가 empty이고 target=0인지 확인한다. green Global은 여전히 원래 route이므로 회피 승인 경로로 오해하지 않는다.
- source 종료/empty/route 교체/clock rewind에서 이전 path·footprint·red mirror가 남지 않는지 확인한다.
- center-strip 밖 작은 tracking overshoot, 실제 경계/vehicle body 자료, candidate 계산 시간과 end-to-end freshness를 확인하기 전 실차/실제 MORAI 20 km/h 통과 안전을 확정하지 않는다. 이번에는 주행하지 않았다.
