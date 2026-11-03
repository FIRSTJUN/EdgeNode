# S4 runtime speed / maneuver continuity 최소 수정 보고

기준: `/workspace`, `k-city`, 기존 HEAD `81a2add` 및 작업 시작 시 존재하던 미커밋 수정. 근거: `logs/kcity_s4_20kph_blocked_analysis/REPORT.md`와 그 오프라인 scene.

MORAI 실행·주행, commit, push를 하지 않았다. ROS transport 테스트는 별도 namespace/topic과 `ROS_DOMAIN_ID=173`, `ROS_LOCALHOST_ONLY=1`로 격리했다. 실제 PlanningNode parameter 테스트는 timer를 생성하지 않고 수동 cycle만 실행하며 drive를 비활성화했다.

조향 clamp 0.70, 곡률 한계 **0.16046692048050848 /m**, clearance **1.20 m**, 기본 offset 범위 **±2.4 m**를 유지했다. ±2.45/±2.50 후보는 추가하지 않았다. Control/PID/Localization/Dijkstra/Perception self-mask를 변경하지 않았다.

## A. 이번 단계 변경 파일

기존 미커밋 변경을 보존하고 다음 파일에만 작업했다.

- `edgenode_planning/planning_node.py`: cycle 시작에서 runtime 속도 동기화, 진단 publisher, heading/progress 및 그룹 입력 연결.
- `edgenode_planning/lattice_planner.py`: Hermite boundary 연결, world completion boundary/진행률 기억, 객체 연결 extent, 후보별 진단.
- `config/planning.yaml`: `obstacle_points_per_cluster: 8` 계약만 추가. 기존 속도/안전 설정값은 유지.
- `test/test_planning_node.py`: 실제 mutable planner를 사용하는 spy, runtime 및 BLOCKED/FSM 진단 회귀.
- `test/test_lattice_runtime_continuity.py`: 기록 scene, 이동 Ego, heading/미분, 복귀, association 회귀.
- `test/test_runtime_speed_parameters_ros.py`: 실제 ROS parameter storage의 accepted 변경과 다음 cycle 내부 값 검증.
- `test/fixtures/s4_20kph_blocked_scenes.json`: 최초 BLOCKED, 최종 진입, 정지 terminal scene의 최소 입력/기존 후보 수치와 출처.
- `docs/s4_runtime_continuity.md`: 이 보고 및 clearance 후속 설계.

기존 bringup, footprint 문서/테스트, adaptive/safety 문서/fixture 등 작업 시작 전 변경은 그대로 두었다. 실행 증적은 `logs/kcity_s4_runtime_continuity_fix/`에 별도 보관했다.

## B. SpeedPlanner runtime 수정 전/후

전: 생성자에서 cruise/curve/sharp를 복사한 후 ROS parameter만 변경되면 객체는 과거 속도를 계속 사용했다.

후: `PlanningNode.plan()` 시작에서 세 속도를 실제 `SpeedPlanner` 필드에 동기화한다. accepted parameter 변경 다음 cycle의 추천 계산부터 적용된다. 초기화 때의 값과 cycle 중 실제 사용값의 계약이 같다. 세 속도 이외 preview/threshold 설정의 runtime 계약은 이번 범위에 추가하지 않았다.

- 기존 `/planning/recommended_speed`: 경로 곡률 분류가 추천한 km/h. drive enable/FSM/cap 적용 전 값이며 중복 publisher를 만들지 않았다.
- 새 `/planning/runtime_cruise_speed`, `/planning/runtime_curve_speed`, `/planning/runtime_sharp_curve_speed`: 그 cycle에 실제 객체에 적용한 km/h.
- 새 `/planning/runtime_max_drive_speed`: 최종 적용 단계의 max_drive readback.
- 기존 `/planning/obstacle_avoid_speed_cap_kmh`: avoidance cap 설정. 잘못된 값은 기존 fail-closed 처리.
- 기존 `/planning/applied_speed_cap_kmh`: FSM이 이동을 허용할 때 max_drive, OBSTACLE_AVOID에서는 min(max_drive, avoid_cap). FSM이 이동을 막으면 0.
- 기존 `/planning/target_speed`: enable/lease/input 검증 및 FSM이 허용한 경우 min(recommended, 적용 cap), 그 외 0. 추천 20이어도 BLOCKED이면 target/applied cap은 0이다.

예를 들어 drive가 허용될 때 recommendation=20, max_drive=12, avoid_cap=8이면 NORMAL_DRIVE target=12, OBSTACLE_AVOID target=8이다. cap 변경은 기존대로 매 cycle 읽는다. 이번 작업은 시작 설정을 20으로 올리거나 drive를 켜지 않는다.

## C. cached 3.5 재현 및 방지

CRUISE/CURVE/SHARP_CURVE 각각 초기 객체와 parameter를 3.5로 설정해 실제 recommendation/target=3.5를 확인했다. parameter readback을 20으로 바꾸고 **다음 cycle 전 객체를 직접 호출하면 여전히 3.5**가 나오는 기존 실패 조건을 명시적으로 재현했다. 다음 `PlanningNode.plan()` 후에는 해당 내부 필드와 recommendation/target이 모두 20이다(max_drive/avoid_cap=20, drive 허용 조건).

추가 실제 ROS 테스트는 `PlanningNode`를 생성해 `set_parameters`의 successful 결과와 readback=20을 확인한다. 객체가 아직 3.5인 상태에서 수동 cycle을 실행한 뒤 sharp 내부 값/output=20을 검증한다. 이는 정적 코드 문자열 검사가 아니라 실제 `SpeedPlanner.plan` 출력 검사다. 이 테스트의 drive는 false다.

## D. Lattice diagnostic 계약

새 토픽은 모두 `/planning/` 아래이며 수치는 Float32, reason은 String이다. 곡률 단위는 /m, offset/clearance는 m이다.

| 새 이름 | 의미 |
|---|---|
| `actual_lateral_offset` | 현재 Ego를 road reference에 투영한 signed lateral offset |
| `last_approved_offset` | 현재 maneuver에서 마지막 승인한 avoidance 목표. BLOCKED/pending/실패 후보로 덮어쓰지 않는다. CLEAR/reset 후 NaN |
| `evaluated_candidate_offset` | 이번 진단에 사용한 후보의 목표. 승인 후보 또는 BLOCKED의 best failed candidate |
| `evaluated_candidate_clearance` | 위 후보의 현재 전체 obstacle 점에 대한 최소 polyline 거리 |
| `evaluated_candidate_profile_curvature` | 위 후보의 연속 lateral profile 곡률 최대. straight-reference nominal 값 |
| `evaluated_candidate_polyline_curvature` | 위 후보의 실제 world polyline 최대 곡률 |
| `evaluated_candidate_reject_reason` | CLEARANCE/ENTRY_DEADLINE/CURVATURE의 조합, 또는 direction/release 정책 이유. 후보가 없으면 NOT_EVALUATED |

유효 입력이 없으면 새 수치 토픽은 NaN이다. 후보 미평가 정지의 전체 원인은 기존 `lattice_blocked_reason`을 함께 본다. RELEASE_PENDING은 기하 검사를 통과해도 아직 실행 승인되지 않은 후보를 표시한다.

기존 `lattice_offset`은 승인 때 목표, BLOCKED 때 actual이라는 호환 계약을 유지한다. 기존 `lattice_clearance`/`lattice_max_curvature`/entry 값도 진단 후보 기준을 유지한다. 따라서 기존 토픽만 조합해 하나의 후보라고 해석하면 안 된다. 새 evaluated 값들은 동일 후보를 가리킨다. gate는 profile/polyline 곡률 중 큰 값과 기존 한계를 비교한다.

## E. Active maneuver continuity

승인한 quintic의 계수, 길이, world road reference, profile 시작 s, world completion point, 승인 당시 road-global progress를 기억한다. maneuver 시작 global progress는 CLEAR/route reset 전 유지하며, 진행량은 뒤로 가지 않는다. road-global `_progress_index`/`_progress_arc_m` 처리 자체는 변경하지 않았다.

다음 cycle에서 이전 world reference에 현재 Ego를 재투영하고, 승인 profile의 진행 지점에서 d'/d''를 평가한다. 실제 ROS caller는 현재 yaw도 전달한다. 새 경로의 시작은 Ego의 실제 위치이며, road normal의 변화율을 반영해 현재 yaw와 맞는 초기 d'를 계산하고 이전 profile의 d''를 연결한다. 상대 heading이 전진 Frenet profile로 표현되지 않으면 BLOCKED다.

일반 quintic Hermite 경계 `(d,d',d'') -> (target,0,0)`를 사용한다. 같은 목표에서는 이전 completion point를 현재 reference에 재투영한 남은 길이를 먼저 사용한다. 정확히 추종하는 직선 reference에서 새 polynomial은 이전 승인 경로의 남은 suffix와 동등함을 전 구간 비교했다. target 근처에서 5 m entry를 새로 시작하지 않는다. localization jitter에는 monotone road progress를 쓰며, 필요하면 기존 curvature/deadline gate를 만족시키기 위한 소폭 길이 조정만 발생한다.

이동 profile에는 정지 시작용 rest-to-rest 최소 길이를 다시 적용하지 않는다. 실제 nonzero d'/d'' quintic의 연속 곡률 극값은 polynomial stationary root로 평가하고, world polyline 곡률도 별도로 검사한다. reference는 sampled polyline이므로 매끄러운 world 곡면의 정확한 연속 곡률이나 실제 Ego의 측정 곡률을 증명하는 구현은 아니다. d''는 이전 승인 계획에서 전달하며 정지 상태의 yaw-rate/v 추정은 사용하지 않는다.

RECOVERING도 남은 suffix를 연결한다. lateral offset이 과거 0.15 m threshold 안에 들어왔다는 이유만으로 남은 경계를 road center에 snap하지 않는다. d/d'/d''가 수치상 종료될 때 CLEAR가 된다. horizon이 소진되면 미검사 reference로 주행하지 않고 BLOCKED다.

매 cycle 전체 obstacle로 clearance를 다시 평가한다. curvature, entry deadline, rear passage, direction continuity, BLOCKED release persistence를 유지한다. 이전 승인이라는 이유로 unsafe path를 승인하지 않는다. 이 구현은 path continuity 개선이며 20 km/h 동역학 적합성의 증명은 아니다.

## F. Obstacle association

Perception 코드를 변경하지 않았다. 현재 `lidar_obstacle_node.publish_obstacles`가 cluster마다 연속 8점을 내보내는 확인된 계약을 Planning 설정으로 명시했다. YAML은 `obstacle_points_per_cluster=8`, standalone 기본값은 기존 anonymous centers를 위해 1이다. frame 내 cluster index를 영구 object ID라고 취급하지 않는다.

Planning callback은 순서대로 그룹 경계를 보존한다. finite 점 개수가 원본과 다르거나 8점 배수가 아니면 그룹 metadata를 사용하지 않고 모든 finite 점을 유지한다. anonymous API는 공간 연결 그룹을 사용하는 fallback을 제공한다. 이는 semantic/tracker identity가 아니다.

relevant 그룹 중 가까운 객체로 active extent를 시작하고, 다음 관측은 world XY bounds의 overlap/gap(기존 1.20 m clearance를 association gate로 사용)으로 연결한다. 여러 그룹을 임의 합치지 않고 가장 가까운 bounds를 선택한다. 연결된 객체의 front/rear world point만 보수적으로 기억하고 매 cycle path s로 재투영한다. 기억한 rear를 실제 통과하기 전 다른 객체로 교체하거나 관측 소실만으로 recovery하지 않는다.

전체 obstacle 점 배열은 별도로 그대로 유지해 모든 후보의 collision 검사에 넣는다. far patch가 active extent에 들어가지 않아도 shifted path와 충돌하면 BLOCKED가 된다. 그룹 순서가 `[near,far] -> [far,near]`로 바뀌어도 near rear를 유지하는 회귀를 포함했다.

최소 계약의 한계: 근접한 객체의 split/merge나 급격한 이동을 semantic ID로 구별하지 못한다. unmatched 관측으로 기억을 지우지는 않는다. 후속 interface는 acquisition stamp/frame과 함께 per-cluster ID, solid polygon, uncertainty를 전달하는 명시적 메시지가 적절하다. 실제 tracker ID가 없으면 persistent semantic identity를 약속하지 않아야 한다.

## G. 최초/최종 BLOCKED fixture

fixture는 기존 생성기의 기하와 새 live heading 경계를 별도로 검사한다. 아래는 기존 기하에서 마지막 목표 -2.4의 재현값이다.

| scene | -2.4 clearance (m) | -2.4 profile κ (/m) | 기존 거부 이유 | 새 live 경계/FSM 결과 |
|---|---:|---:|---|---|
| 최초 t=16.232000 s | 1.194733405 | 0.1604669195 | CLEARANCE | BLOCKED / EMERGENCY_STOP / target=0 |
| 최종 진입 t=18.531183 s | 1.174928364 | 0.1321457942 | CLEARANCE | BLOCKED / EMERGENCY_STOP / target=0 |
| terminal t≈20.135727 s | 1.171893973 | 0.1307042066 | CLEARANCE | BLOCKED / EMERGENCY_STOP / target=0 |

새 진단은 actual≈-1.540/-1.820/-1.827, last approved=-2.4, evaluated failed candidate=+2.4를 분리한다. measured heading/exact Ego start를 사용하는 새 후보의 곡률 수치는 과거의 zero-derivative 후보와 다르며 과거 0.628을 그대로 고정하지 않는다. 어떤 경우도 failed +2.4를 승인 목표로 해석하지 않는다.

후보를 안전하게 만들 수 없으면 BLOCKED와 target/applied cap=0을 유지한다. association 수정만으로 원래 scene의 clearance 실패가 해결됐다고 주장하지 않는다.

## H. Clearance representation 후속 설계

현재 `footprint_xy()`는 **sensor-local XY min/max AABB**의 네 corner와 네 midpoint를 만든 후 각 점을 world로 회전/이동한다. world-axis AABB를 새로 만드는 구조가 아니다. 따라서 object orientation과 sensor 축이 다르면 관측되지 않은 직사각 corner가 생성된다. 관측 면이 바뀌면 같은 정적 객체라도 min/max 크기가 변한다.

분석 terminal의 -2.4 후보는 raw return 거리≈1.570933 m, 현 8점 footprint 거리≈1.171894 m, 요구값=1.20 m다. 가장 가까운 corner와 실제 return 사이의 빈 공간은 unseen body가 없다는 증거가 아니다. 이번 수정은 clearance representation을 바꾸지 않았다.

후속 단계의 구조:

1. **OBB**: 원시 cluster와 acquisition pose를 유지한 채 object heading 또는 안정된 minimum-area fit으로 관측 OBB를 구성한다. PCA/fit만으로 full vehicle body를 확정하지 않고, heading 불안정·부분 관측·퇴화 cluster의 uncertainty envelope와 conservative fallback을 정의한다. 기존 sensor-axis AABB와 동일 scene에서 비교한다.
2. **Solid polygon**: cluster마다 닫힌 polygon과 uncertainty를 전달한다. 선분이 polygon 내부를 통과하거나 교차하면 clearance=0으로 처리하고, 나머지는 edge-to-segment 거리로 계산한다. 현재 8개의 점만 비교하는 방식은 polygon 내부/edge 전체 검사가 아니다. raw-only 또는 observed convex hull만으로 unseen 면의 점유를 제거하지 않는다.
3. **Sonata full-body swept polygon**: verified body polygon을 localization reference pose에 붙여 각 candidate에서 이동/회전시키고 obstacle solid polygon과 검사한다. 샘플 사이 이동·yaw 보간도 보수적으로 덮으며 front/rear passage를 실제 차체로 검증한다. 지금의 centerline + 1.20 m 거리 검사는 full-body sweep이 아니다.
4. **MGeo/drivable boundary**: 현재 `local_data/kcity_mgeo`에는 global_info/node_set/link_set만 있으며 lane-boundary/차도 solid polygon 파일이 없다. link의 centerline/인접 lane ID로 차도 폭을 임의 추정하지 않는다. lane/road boundary dataset, 좌표계, 연결 관계, 교차로·빈 영역 처리까지 확보한 뒤 swept Ego polygon이 유효 drive area 안에 있는지 검사한다.

필요 자료: 선택한 MORAI Sonata의 실제 `VehicleSpec` 응답 또는 확정된 차체 geometry. 저장소의 메시지 schema에는 `size`, `wheel_base`, `max_steering`, `overhang`, `rear_overhang`가 있으나 현재 차량의 확정 응답은 없다. 각 필드의 축/단위, 특히 `overhang`의 의미, 앞/뒤 overhang, localization pose의 rear axle/차체 중심/GPS 기준점 및 sensor extrinsic을 검증해야 한다. 임의 길이·폭·overhang은 추가하지 않았다. 기존 0.95 m half-width도 확정 geometry라고 승격하지 않았다.

안전한 후보 확대는 위 geometry/solid occupancy/drivable boundary와 불확실성 처리를 검증한 뒤 별도 단계에서 한다. 이번 offset 범위는 확대하지 않는다.

## I. 전체 테스트 결과

최종 전체 ROS source 테스트 및 기존 logger lint: **615 passed, 1 skipped, 2 warnings**. skip은 기존 copyright 검사이며 logger의 selectable entry point deprecated 경고 2개가 있다.

실행:

```bash
cd /workspace/ros2_ws/src/edgenode_logger
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q /workspace/ros2_ws/src
```

logger의 기존 ament lint가 현재 작업 디렉터리를 검사하므로 해당 package에서 전체 테스트 path를 전달했다. 테스트를 deselect/skip하도록 수정하지 않았다. Planning/FSM/Control/Localization/Perception 및 나머지 ROS 패키지 테스트를 포함한다.

루트의 무인자 `pytest`도 시도했으나 외부 `Ultra-Fast-Lane-Detection-v2/morai_ufld_test.py`의 torch import가 실패해 collection이 중단됐다. `/workspace`에서 source path만 전달하면 logger lint가 vendor/logs까지 재귀 검사하는 실행 범위 문제도 확인했다. 외부 torch를 설치하거나 UFLD/MORAI 수동 실행 스크립트를 실행하지 않았으며, 위 패키지 범위의 전체 실행으로 검증했다.

회귀에 포함한 항목: 세 속도 mode의 cached 3.5→readback 20→실제 output 20, 실제 ROS parameter 변경, caps와 recommendation 분리, BLOCKED actual/approved/failed candidate 구분, 최초/최종/terminal scene, moving Ego suffix 전 구간 동등성, nonzero yaw, d'/d'', target 근처 fresh entry 방지, localization jitter, recovery 끝 경계, horizon 소진 시 stop, near NPC/far cluster 분리, far collision 유지, unsafe previous path 재거부, 기존 safety/FSM/Control/Localization/Perception 회귀.

## J. build / diff 결과

`cd /workspace/ros2_ws && colcon build --symlink-install`: **7 packages finished**, 실패/경고 없음.

`git diff --check`: 통과. branch는 `k-city`를 유지했다. commit/push 없음.

## K. 다음 MORAI 정지 검증 및 실제 주행 전 체크리스트

이 목록은 다음 단계 작업이며 이번에는 실행하지 않았다.

- drive=false와 실제 정지 상태에서 시작한다. 세 속도를 3.5로 설정하고 recommendation/runtime 값을 함께 기록한 뒤 20으로 accepted 변경하여 다음 cycle 내부 진단과 추천값이 일치하는지 확인한다. cap을 recommendation과 혼동하지 않는다.
- `/planning/actual_lateral_offset`, `/planning/last_approved_offset`, 모든 evaluated candidate 값/reason, legacy lattice/FSM/target/applied cap, road progress, local path, parameter event, odometry, raw LiDAR, obstacle groups/count/centers를 같은 bag에 기록한다.
- 동일 NPC scene에서 최초/최종 BLOCKED가 계속 target=0을 만드는지 확인한다. 실패 후보 +2.4는 execution target이 아니다. geometry를 유지한 채 단순히 20 설정만으로 통과를 기대하지 않는다.
- 정지 replay/오프라인 이동 입력에서 position/yaw/d'/d''와 world completion boundary 및 forward progress를 확인한다. actual tracking 수정으로 profile이 바뀌면 각 gate의 수치를 기록한다. 20 Hz cycle 시간 예산도 측정한다.
- near/far 순서 변경, 관측 소실/축소, cluster split/merge에서 active front/rear와 world 재투영을 확인한다. far 장애물의 collision veto가 살아 있는지 확인한다.
- 실제 주행 전 Sonata VehicleSpec/차체 기준점, sensor transform, NPC full-body occupancy/uncertainty, drivable boundary 자료와 swept polygon 검증을 완료한다. 불확정 길이/폭을 임의로 채우지 않는다.
- 별도 승인된 주행 검증에서는 기존 steering/curvature/clearance 한계를 유지하고 제한된 enable lease, 안전 정지 수단, collision/실제 속도/명령 속도 및 path tracking 기록을 준비한다. 이번 오프라인 결과만으로 20 km/h 주행을 승인하지 않는다.
