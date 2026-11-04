# S4 A/B/C 오프라인 수정 및 causal stateful 회귀

## 결과와 검증 범위

MORAI 실행/주행, rosbag play, commit/push는 수행하지 않았다. 기존 working tree 변경은 보존했다. Global Path/Dijkstra/Control/PID/Pure Pursuit/Dynamic Lookahead/SpeedPlanner 및 planning.yaml을 변경하지 않았다.

A의 NEW 106/118 envelope failure를 제거했고, B는 아래에 명시한 causal tracking 모델에서 OLD/NEW의 마지막 안전한 상태부터 HOLD까지 진행한다. 좌우 full synthetic maneuver는 무오차/1 mm/1 cm perturbation에서 `ENTRY → ADJACENT_LANE_HOLD → RECOVERING → CLEAR`를 완료한다. 원본 OLD 145 및 NEW 144/145는 계속 BLOCKED다.

**이 결과는 ideal geometric tracker에 대한 회귀다. 실제 Control/Pure Pursuit·actuator·타이어 동역학·지연을 포함한 MORAI 폐루프 검증이라고 주장하지 않는다.** 모델의 오차 가정과 한계는 아래 8절에 명시했다.

코드·결과 증적: `/workspace/logs/kcity_s4_offline_fixes/`.

## 1. 세 root cause의 수정

### A: moving grid와 normal discontinuity

이전 구현은 실제 map vertex의 incoming normal과 바로 뒤 supplemental grid의 outgoing normal을 연결했다. original station이 증가해도 adjacent intersection이 뒤로 가면서 bow-tie quad가 생겼다. station 0.1 mm merge를 2 mm 등으로 확대하는 방법은 쓰지 않았다.

새 `LanePair.corridor_envelope()`는 horizon의 두 cap만 기존 `at()`의 normal intersection으로 잡고, 그 사이 **실제 original/adjacent polyline vertex를 정확히 보존**한다. 이 두 ordered boundary chain을 convex triangle cells로 zipper triangulation한다. legacy `quads` 키는 이 convex cells를 담는다. piecewise-linear boundary에는 supplemental 0.5 m row가 필요 없으므로 생성하지 않는다.

실제 vertex의 삭제/평균/이동/normal smoothing은 없다. 실제 boundary 교차, cap 역행, adjacent 실제 progress 역행, 반전 cell은 fail-closed다. 기존 `corridor_covers`, clipping EPS=1e-7과 obstacle/steering 제한은 그대로다. center-to-center reference-point sanity gate이며 실제 차도 외곽/차체 swept boundary를 새로 가정하지 않는다.

| frame | 수정 전 | 수정 후 |
|---|---|---|
| NEW 106 | NONCONVEX_QUAD #11, station gap 1.104775 mm, adjacent 역행 0.432403 mm | 유효 cells 84, 같은 candidate corridor=true |
| NEW 118 | NONCONVEX_QUAD #24, station gap 1.891871 mm, adjacent 역행 0.871846 mm | 유효 cells 85, 같은 candidate corridor=true |

`test_corridor_sampling_stateful.py`는 실제 두 scene, 실제 vertex 좌표 보존, 1 cm 외부 점 거부, 1 mm 간격 1001개 moving horizon 및 map vertex 근방 cap을 검사한다. NEW 90–130은 같은 planner instance로 41 cycle 모두 AVOIDING이다.

기존의 일부 테스트는 정상 corner에서 발생하던 sampling 오류 자체를 `quads=[]`로 기대했다. 이 기대를 실제 corner 보존/외부 점 거부로 갱신했다. 실제 crossing/unsafe 후보를 통과시키도록 변경하지 않았다. 원래 frozen corridor dump의 clearance/curvature/승인 결과도 유지한다.

### B: parameter jet과 maneuver terminal geometry의 과구속

이전 quintic은 road-station 잔여 길이를 polynomial parameter 잔여 길이처럼 사용했다. 실제 pose/yaw로 시작점을 재고정하면서 이전 curve의 parameter speed/tangential acceleration과 목표의 unit tangent·zero curvature까지 동시에 고정했다. 이 값들은 차량의 실제 m/s 또는 가속도가 아니다.

수정은 세 부분이다.

1. **정확한 suffix 보존.** 실제 pose/yaw가 이전 curve 위에 있으면 이전 curve의 남은 parameter 길이로 그 suffix를 이어간다. road station과 curve parameter를 동일시해 다시 fitting하지 않는다. `_transition_end_station`은 지도 station, public transition length는 road 잔여 길이, 실제 curve parameter length는 별도로 유지한다.
2. **G2 fallback.** 기존 후보를 먼저 평가하고 모두 실패할 때 actual position/yaw와 carried signed geometric curvature를 유지하는 재매개화 후보를 검사한다. `p'=αt`, `p''=κ α² n`이면 α>0에서 signed curvature는 그대로 κ다. parameter derivative를 바꾸면서도 실제 시작점/방향/기하학적 곡률을 버리지 않는다.
3. **실제 corridor에 따른 Bezier handle.** quintic의 앞 두 control point `p+H*t/5`, `p+2H*t/5+κ*H²*n/20`이 실제 corridor 안에 있도록 시작 handle을 제한한다. 끝 두 handle은 실제 목표 중심선의 incoming segment 안에 둔다. 끝점/접선 방향/끝 curvature=0은 유지한다. 모든 후보는 continuous curvature, clearance, 전체 segment corridor, direction, deadline을 다시 통과해야 한다.

또한 acquired HOLD가 zero-length 최적화 후 1 mm 오차를 받았을 때 새 5 m entry를 시작하던 동작을 고쳤다. 이 경우 기존 기하학적 required length와 0.25 m floor로 잔여 correction을 만든다.

샘플 vertex만 corridor 안에 있고 quintic이 샘플 사이에서 나가는 후보도 허용하지 않는다. 새 추가 검사는 각 convex cell edge와 quintic polynomial의 교차 root를 구해 curve 전체의 coverage를 확인한다. 새 경계 허용 폭을 추가하지 않으며 기존 sampled gate보다 엄격하게 floating-point roundoff(8 ULP, 기존 EPS 이하)만 허용한다. 성공한 curve가 다음 sampling frame의 더 짧은 edge에서 갑자기 외부로 보이는 현상도 피한다.

### C: RECOVERING 완료점과 correction 끝점의 분리

첫 **승인된** recovery에서 `_recovery_completion_progress`를 정하고 고정한다. 매 cycle의 `_transition_end_progress`는 correction을 위해 계속 rebase하되, 이 값을 recovery 완료 기준으로 chase하지 않는다.

CLEAR handoff 조건은 다음을 모두 만족해야 한다.

- obstacle rear gate를 통과해 recovery가 허용됨
- 첫 recovery completion progress에 실제 progress가 도달함
- original center 거리 <= 기존 `recovery_threshold_m` (0.15 m)
- 기존 heading alignment 식 `cos(error)>1-1e-5` 충족(약 0.256°)
- **현재 실제 pose에서 만든 recovery correction이 모든 safety gate와 기존 release confirmation을 통과함**

CLEAR가 되는 cycle에도 검증한 correction points를 Control에 전달한다. unchecked original-center snap을 발행하지 않는다. 이후 pair/world transition/anchor를 해제하며 inactive stage Path와 selected adjacent 표시는 비운다.

rear 통과는 recovery 허가 조건이지 진행 중인 HOLD correction의 outward curvature가 곧바로 안전한 recovery launch라는 뜻은 아니다. recovery 후보가 실패하고 현재 HOLD correction이 새로 모든 gate를 통과하면 그 correction을 이어간 뒤 recovery를 재시도한다. 안전한 correction도 없으면 기존처럼 BLOCKED다. phase를 강제로 바꾸지 않는다.

## 2. 변경 파일

Production:

- `edgenode_planning/lane_geometry.py`: exact boundary envelope
- `edgenode_planning/adjacent_lane_planner.py`: suffix/G2/Bezier handles/continuous corridor, recovery acquisition, decision metadata
- `edgenode_planning/planning_node.py`: odometry receipt/header timestamps와 debug publisher/snapshot 연결
- `edgenode_planning/corridor_scene.py`: diagnostic-only context를 PRE-state serialization에서 제외
- `edgenode_planning/maneuver_diagnostics.py`: 새 read-only 진단 formatter

신규 회귀와 helper:

- `test_corridor_sampling_stateful.py`, `test_s4_closed_loop.py`, `test_recovery_completion.py`, `test_maneuver_diagnostics.py`
- `s4_stateful_helpers.py`, `s4_closed_loop.py`
- `fixtures/s4_moving_envelope_scenes.json.gz`, `fixtures/s4_stateful_live_sequences.json.gz`

기존 corridor/continuation/adjacent/node/runtime-parameter tests도 위 의미에 맞게 보강했다. 작업 전후 실제 hash 비교 목록은 `verification.json`에 있다. Planning package 밖의 source는 모두 동일하다.

## 3. Production diff

이번 작업 시작 snapshot 대비 diff: `/workspace/logs/kcity_s4_offline_fixes/production.patch`.

전체 working-tree diff에는 이전 ENTRY anchor/diagnostics 변경도 포함되므로 이번 작업 diff와 구분한다. 원본 baseline production은 같은 디렉터리의 `before_*.py`에 보존했다.

## 4. shortening 7줄 유지 여부

유지했다. 기존 실제 경계조건을 그대로 잇는 길이 후보를 우선하고, 그 계열에 안전한 후보가 없을 때만 G2/geometry-handle fallback을 평가한다. 이번 1.25 m 값을 하드코딩하지 않았다.

기존 preferred/긴 쪽 12개/짧은 쪽 12개에, 필요할 때 `required_transition_length(distance)`와 `distance*cot(|heading error|/2)`로 유도한 최대 두 proposal을 더한다. 이는 polynomial shape 변경과 별개인 제한된 기하 proposal이다. 모든 road length는 기존 0.25 m/half-remainder 정책, horizon/pair 길이, strict front deadline을 만족해야 한다. initial ENTRY의 3 s policy는 변경하지 않았다.

## 5. Corridor anchor 보존 및 unsafe 유지

실제 original 및 adjacent clipped boundary vertex는 각 cell의 정확한 좌표로 남는다. supplemental row만 제거되며 actual point set을 smooth하거나 lane width를 추정하지 않는다. 큰 corner를 숨겨 제거하지 않는다. 기존 normal-row nonconvexity와 실제 boundary crossing을 구분한다.

회귀 결과:

- 기존 20 km/h unsafe fixture 3개: BLOCKED 유지
- 원본 OLD 145: BLOCKED 유지
- 원본 NEW 144/145: BLOCKED 유지
- 실제 boundary crossing / 외부 1 cm / reverse heading / clearance obstacle: 거부 유지
- no adjacent lane: BLOCKED 유지
- unsafe recovery correction: CLEAR 불가

OLD 145는 최대 허용 curvature로 가장 빨리 좌회전해도 0.7 m 뒤 x>=144.097914가 되며 해당 adjacent center 최대 x≈144.095377를 넘는다. NEW 144/145는 시작 pose 자체가 corridor 밖이다. 이 입력을 AVOIDING으로 바꾸는 목표는 적용하지 않았다.

## 6. Stateful 비교와 negative control

`replay_before.json`, `replay_after_a.json`은 동일 recorded-input prefix를 사용한다. A만 수정하면 NEW 106/118의 BLOCKED 및 그 뒤 두 RELEASE_PENDING이 사라진다. OLD terminal 문제와 NEW 141 이후 문제는 별도다.

B causal 회귀는 OLD 143 / NEW 139의 마지막 안전한 상태 이전 전체 prefix를 warm up한다(최소 이전 20 cycle 포함). 그 뒤의 pose는 planner의 승인 curve만으로 생성한다. bag의 미래 odometry를 주입하지 않는다.

| 시작 | noise | planner dt | causal cycle 수 | 결과 | 최대 κ | 최소 clearance |
|---|---:|---:|---:|---|---:|---:|
| OLD 143 | 0 | 0.05 s | 39 | ENTRY→HOLD | 0.146927539 | 2.202947518 m |
| NEW 139 | 0 | 0.05 s | 39 | ENTRY→HOLD | 0.141546891 | 2.246158456 m |
| OLD 143 | 1 mm | 0.05 s | 39 | ENTRY→HOLD | 0.146927539 | 2.202947518 m |
| NEW 139 | 1 mm | 0.05 s | 39 | ENTRY→HOLD | 0.141546891 | 2.246158456 m |
| OLD 143 | 1 cm | 0.134 s | 15 | ENTRY→HOLD | 0.146927539 | 2.202947518 m |
| NEW 139 | 1 cm | 0.134 s | 16 | ENTRY→HOLD | 0.141546891 | 2.246158456 m |

두 run 모두 원래 unsafe frame의 map station을 넘어 진행하며 그 unsafe pose를 재사용하지 않는다.

**이 ideal tracker에서 기존 planner도 일부 짧은 OLD/NEW 구간을 통과한다. 이 표만으로 B 개선 효과를 입증했다고 주장하지 않는다.** 이를 구분하기 위해 A/C를 고정하고 B만 이전 구현으로 교체한 negative control을 별도로 실행했다.

- 동일 right-lane 1 mm perturbation full maneuver: before B는 cycle 330 / progress≈13.026476에서 HOLD→BLOCKED / DRIVABLE_CORRIDOR.
- after B는 cycle 847 / progress≈33.854192에서 CLEAR.
- negative-control source와 모든 cycle 기록: `negative_control.py/json`.

NEW 141의 원본 실제 입력은 변경 후에도 일부 reconstructed history에서 거부될 수 있다. 수정 후 causal trajectory가 그 원본 미래 state를 통과해야 한다는 주장은 하지 않는다. 해당 근방에 더 안전한 상태로 도달하는지 검증한다.

## 7. Recovery completion 회귀와 실제 phase sequence

직접 recovery 회귀는 exact center, 1 mm, 3 cm, threshold 바로 안쪽, 작은 yaw error를 포함한다. 고정 goal의 0.2 m 전부터 3 km/h·20 Hz로 진행하면 goal 도달 후 CLEAR다. threshold 바로 밖 또는 heading alignment 밖이면 RECOVERING을 유지한다. rear+0.5 m 이전에는 recovery를 승인하지 않는다. unsafe candidate나 3-cycle release 확인을 우회하지 않는다.

full synthetic 결과(각 cycle마다 curvature/clearance/corridor/실제 시작 heading 확인):

| 방향/noise | 완료 cycle 수 | phase sequence |
|---|---:|---|
| right / 0 | 837 | ENTRY→HOLD→RECOVERING→CLEAR |
| right / 1 mm | 848 | 동일 |
| right / 1 cm | 837 | 동일 |
| left / 0 | 837 | 동일 |
| left / 1 mm | 858 | 동일 |
| left / 1 cm | 837 | 동일 |

right / 1 mm의 실제 전환(0-based):

| cycle | progress | phase |
|---:|---:|---|
| 0 | 0 | ENTRY |
| 276 | 10.776508 m | ADJACENT_LANE_HOLD |
| 558 | 22.526469 m | RECOVERING |
| 847 | 33.854192 m | CLEAR |

모든 synthetic case의 최대 evaluated curvature는 약0.160466919734 /m로 기존 limit 0.160466920481 /m 이내다. 전체 최소 clearance는 약2.880770353 m 이상이다. fallback curve에는 endpoint 근방을 포함하는 별도의 curvature sample 검사도 둔다.

## 8. 모델과 오차 가정 — 반드시 구분할 것

`s4_closed_loop.py`는 **ideal geometric curvature follower**다. 승인 analytic quintic의 XY arc를 v*dt만큼 전진하고 그 접선으로 다음 yaw를 만든다. curve 끝 이후에는 실제 map suffix를 따른다. polynomial parameter와 실제 arc를 Simpson integration/bisection으로 구분한다. 속도는3 km/h다.

- planner 간격:0.05 s 및0.134 s
- 노이즈:47 cycle마다 strip 안쪽으로 향하는0/1 mm/1 cm position perturbation, yaw 최대0.001 rad
- 각 cycle은 같은 planner private state를 유지
- 모델이 이미 corridor 밖으로 나간 pose를 몰래 center로 clamp하지 않음
- 관측 장애물은 시작 시점의 recorded prefix를 freeze한 static collision superset; 미래 bag 입력 없음
- full synthetic는 일반 waypoint teleport 없이 출력 curve로만 연속 진행

이 모델은 실제 Pure Pursuit, steering actuator 응답/타이어 slip/차체 기준점 및 약0.2 s stale odometry를 포함하지 않는다.0.134 s interval은 관측 지연 모델이 아니다. 모든 종류의 지속 편향/매 cycle lateral impulse/유실을 보장하지 않는다. 더 강한 매-cycle disturbance 탐색에서는 BLOCKED 사례도 관찰했으며 이 결과를 통과로 집계하지 않았다. corridor 밖 perturbation은 계속 reject돼야 한다.

따라서 이 결과를 MORAI 완주 또는 임의 tracking error에 대한 보장으로 해석하지 않는다. 통과한 모델/오차 가정 안에서 phase 유한 종료와 hard gates를 검증한 것이다. source를 실제 차량이 추종할 때의 검증에는 아래 진단을 함께 남겨야 한다.

## 9. 진단과 snapshot

새 `/planning/lane_change_maneuver_debug` (std_msgs/String JSON):

- `before/after`: ENTRY/recovery completion anchor, transition start/end progress/station, last parameter, private phase
- `decision`: current progress, obstacle front/rear, observed/rear_pending, recovery deferred 및 rejection reason
- `candidate`: selected road length, curve parameter length, boundary mode/first/second, endpoint/tangent/second
- continuous maximum curvature, clearance
- adjacent center distance/heading error와 target center distance/heading error
- recovery acquisition 결과와 완료 anchor
- corridor reason/index/first failure/envelope failure/violation
- odometry header stamp, callback receipt stamp, Planning start stamp, output stamp

`previous_curve_parameter`는 실제 boundary 계산에 사용한 값이고, after `_transition_last_parameter=0`은 승인한 새 curve 좌표계의 정상 reset이다.

기존 `/planning/dump_lane_change_corridor_scene` snapshot에 이 same-cycle 진단도 포함된다. 진단 함수의 read-only 및 다음 planning 결과 동일성 테스트를 통과했다. release pending에서도 evaluated candidate metadata를 보존한다.

corridor의 기존 `max_violation_m`는 rejected sampled vertices 기준이다. 추가 continuous check에서 샘플 사이만 이탈하면 `continuous_transition_failure`와 `continuous_witness_violation_m`를 확인한다. witness distance를 연속 curve 전체의 maximum이라고 부르지 않는다.

### 이후 한 번의 MORAI 기록 명령 — 이번 작업에서 실행하지 않음

정지 상태에서 먼저:

```bash
ros2 param dump /planning_node
ros2 topic echo /planning/lane_change_maneuver_debug --once
ros2 service call /planning/dump_lane_change_corridor_scene std_srvs/srv/Trigger '{}'
```

기록:

```bash
ros2 bag record -o /workspace/logs/kcity_s4_abc_fix_live_01 \
  /parameter_events /localization/odometry /ego_vehicle_status /ctrl_cmd \
  /planning/obstacles /planning/global_path /planning/local_path \
  /planning/local_path_progress_m /planning/local_path_index /planning/current_link \
  /planning/map_match_status /planning/map_match_distance /planning/map_match_heading_diff \
  /planning/lane_change_state /planning/lattice_status /planning/drive_status \
  /planning/fsm_state /planning/target_speed /planning/selected_adjacent_link \
  /planning/lattice_blocked_reason /planning/evaluated_candidate_reject_reason \
  /planning/lattice_entry_length /planning/lattice_recovery_length \
  /planning/lattice_clearance /planning/lattice_max_curvature /planning/lattice_curvature_limit \
  /planning/lattice_obstacle_front_s /planning/lattice_obstacle_rear_s \
  /planning/lane_change_obstacle_safe /planning/lane_change_steering_feasible \
  /planning/lane_change_corridor_safe /planning/lane_change_corridor_debug \
  /planning/lane_change_corridor_violation_m /planning/lane_change_corridor_failed_index \
  /planning/lane_change_corridor_failed_point_count /planning/lane_change_maneuver_debug \
  /planning/actual_lateral_offset /planning/last_approved_offset \
  /planning/evaluated_candidate_offset /planning/evaluated_candidate_clearance \
  /planning/evaluated_candidate_profile_curvature /planning/evaluated_candidate_polyline_curvature \
  /planning/original_lane_center /planning/adjacent_lane_center \
  /planning/lane_change_entry_path /planning/adjacent_lane_hold_path \
  /planning/lane_change_recovery_path /planning/lane_change_recovery_gate \
  /control/path_error /control/lookahead_distance /control/pure_pursuit_steering
```

확인할 phase 순서는 ENTRY→HOLD→RECOVERING→CLEAR다. 그 밖에 input age, BLOCKED의 실제 후보/continuous witness, recovery deferred의 지속 여부, 고정 anchor와 이동 correction 끝점을 함께 비교한다. enable_drive나 MORAI 주행을 자동 실행하는 명령은 추가하지 않았다.

## 10. 검증 및 재현

- Planning 전체 pytest:599 passed
- 신규 A/C/causal/stateful/safety/진단 회귀 포함
- `git diff --check`:통과
- `colcon build --packages-select edgenode_planning`:성공
- production source/install Python module:byte-identical
- Control/Global/Dijkstra/Localization/Perception/speed/config source:작업 전과 동일
- MORAI 실행/주행, commit/push:없음

build의 byte-compiling disabled 경고는 `PYTHONDONTWRITEBYTECODE=1`에 따른 생략이며 build 실패가 아니다.

최종 정확한 pytest 시간/build 결과/hash 목록은 `planning_tests.txt`, `build.txt`, `verification.json`을 기준으로 한다. `closed_loop_results.json`은 pytest JUnit properties에서 추출한12개 causal/full-maneuver 결과다.

재현:

```bash
cd /workspace
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -B -m pytest -q -p no:cacheprovider ros2_ws/src/edgenode_planning/test
python3 -B logs/kcity_s4_offline_fixes/replay_stage.py before
python3 -B logs/kcity_s4_offline_fixes/replay_stage.py after_a
python3 -B logs/kcity_s4_offline_fixes/replay_stage.py after_abc
python3 -B logs/kcity_s4_offline_fixes/negative_control.py
```

`before_*.py`와 saved JSON은 비교 증적이다. production을 되돌려 실행하지 않는다. 두 원본 bag은 read-only로 읽었으며 gzip fixture는 필요한 network/PRE seed/recorded inputs를 자체 포함한다. bag 이전 private state와 정확한 callback 선택이 없다는 provenance 한계를 보존했다.
