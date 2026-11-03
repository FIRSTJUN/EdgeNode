# RViz marker flicker 수정: A–J 보고

`/workspace`, `k-city` 브랜치에서 시각화 lifecycle만 수정했다. 색, alpha, 두께, 표시용 Z 레이어, 원본 Global/Local point sequence는 유지했다. Planning/Localization/Perception/Control 및 MGeo/경로 생성 코드는 수정하지 않았다. MORAI 주행, commit, push는 하지 않았다. 현재 실행 중인 기존 노드를 강제로 재시작하지 않았다.

## A. 직접 원인과 실제 관측

실제 ROS domain 0에서 marker와 상태를 7초 동안 읽기 전용으로 수집했다. `/planning/state`는 관측한 77개 모두 `GLOBAL_PATH_READY`, lane-change state는 77개 모두 `BLOCKED`, current link는 77개 모두 `A219BS010415`였다. 센서 입력이나 주행 parameter는 변경하지 않았다.

직접 확인한 문제:

1. `freshness_sec - input age`를 marker lifetime과 삭제 deadline에 함께 사용했다. 지연이 0.99초인 유효 입력은 0.01초만 표시되어 다음 update 전에 사라질 수 있었다. 입력이 나중에 도착하면 현재 표시까지 DELETE했다.
2. 장애물 callback은 유효성 검사 전에 매번 DELETEALL을 발행한 뒤 ADD했다. 만료 시에도 DELETEALL을 사용했다.
3. 동일한 정적 geometry도 매 callback마다 복사/발행했다. 입력 큐 depth 10으로 이전 cycle이 쌓일 수 있었다. 반복 상태 callback도 이미 없는 phase marker에 DELETE를 계속 발행했다.

| 실제 기존 marker, 7초 | ADD | DELETE | DELETEALL | ADD lifetime | 최대 ADD 간격 |
|---|---:|---:|---:|---|---|
| Global | 21 | 47 | 0 | **1.868–68.490 ms** | **822.675 ms** |
| Original lane | 59 | 5 | 0 | 4.947–168.228 ms | 602.497 ms |
| Obstacle footprint | 38 | 0 | **64** | 10.512–99.543 ms | 324.277 ms |

Global geometry는 관측 ADD 전체에서 동일했다. 표시 수명이 실제 ADD 간격보다 짧았으므로 RViz가 정상 동작해도 표시 공백이 생기는 구조였다. Current는 동일 geometry를 64회 ADD했다. BLOCKED에서 빈 Local/Adjacent/gate는 각각 64회 DELETE, ENTRY/HOLD/RECOVERY는 각각 128회 DELETE였다. 빈 경로는 이번 수정 후에도 새로 만들지 않는다.

수정 노드는 같은 실제 입력을 읽고 **모든 출력을 별도 `/rviz_flicker_candidate_<pid>/...` 토픽으로 remap**하여 다시 7초 관측했다. 기존 RViz/Planning/Control 출력은 건드리지 않았다.

| 수정 노드, 별도 토픽 7초 | ADD | DELETE | DELETEALL | lifetime |
|---|---:|---:|---:|---|
| Global | **7** | **0** | 0 | **0, 무기한** |
| Original lane | 7 | 0 | 0 | 0 |
| Current lane | 7 | 0 | 0 | 0 |
| Obstacle footprint | **33** | **0** | **0** | **0** |
| Ego | 38 | 0 | 0 | 0 |

위 수정 후 관측은 초기 cleanup 이후의 갱신 구간이다. 수정 노드 startup의 DELETEALL 1회는 아래 C항에 별도로 명시했다. 두 관측은 다른 시각의 7초 구간이며, 메시지 수 자체가 동일해야 하는 비교는 아니다.

증거: `logs/kcity_s4_rviz_flicker/live_marker_samples_before.json`, `candidate_live_input_observation.json`.

## B. 문제 파일 / 함수 / 라인

주된 문제 파일은 `edgenode_planning/planning_visualization_node.py`다. 수정 전 line은 같은 증거 디렉터리의 `planning_visualization_node.py.before` 기준이다.

| 수정 전 위치 | 문제 | 현재 대응 위치 |
|---|---|---|
| `_marker`, 85 | 짧은 기본 lifetime | `_marker`, 117: lifetime 0 |
| `path_callback`, 130 | 입력 age만큼 표시 수명/deadline 단축 | `path_callback`, 200: age는 입력 수락에만 사용 |
| `obstacles_callback`, 164 / 181 | 매 callback DELETEALL + 짧은 lifetime | `obstacles_callback`, 260: stable ID ADD와 사라진 ID DELETE |
| `vehicle_callback`, 195 | age를 차감한 lifetime | `vehicle_callback`, 300: lifetime 0, pose/style 유지 |
| `state_callback`, 150 / `phase_callback`, 155 | 반복 DELETE, 뒤늦은 path callback의 재추가 가능 | 현재 246 / 252: 중복 삭제 억제 및 상태별 재추가 방지 |
| `expire`, 199 / 214 | age 기반 조기 삭제, obstacle DELETEALL | `expire`, 317: 독립 watchdog, ID별 DELETE |

`route_editor_node.py`의 지도 발행은 `__init__` 124에서 `_publish_map`을 한 번 호출한다. 지도 DELETEALL 176은 매 cycle 동작이 아니다. Draft DELETEALL 196도 편집 시 동작이다. 이 파일과 RViz 설정은 변경하지 않았다.

## C. DELETE / DELETEALL 정책

- 정상 갱신: 같은 namespace/ID에 ADD로 geometry를 교체한다. ADD 앞에 전체 삭제를 하지 않는다.
- 같은 geometry: 입력 heartbeat만 갱신하고 고비용 point copy와 즉시 republish를 생략한다.
- 빈 경로, 무효 frame/geometry, state/phase 해제, 실제 route 교체, 시각 되돌림, source timeout: 필요한 marker만 DELETE한다. 같은 삭제를 callback마다 반복하지 않는다.
- 장애물 그룹 수 감소: `이전 ID - 현재 ID`만 DELETE하고 현재 ID들은 ADD한다. 빈 입력은 기존 ID만 삭제하며 반복 빈 입력은 발행하지 않는다.
- **DELETEALL은 시각화 노드 시작 시 한 번만 사용한다** (`__init__`, 100). 이전 프로세스의 장애물 ID를 알 수 없는 초기 cleanup용이다. 정상 callback, watchdog, clock reset에는 사용하지 않는다.
- 초기 single-marker cleanup도 각 역할의 고정 ID에 한 번만 발행한다.
- Route Editor의 기존 초기 지도/편집 cleanup은 유지했다.

## D. lifetime / freshness 계약

모든 표시용 marker의 `lifetime`은 `sec=0, nanosec=0`이다. 입력 지연을 표시 수명에서 차감하지 않는다.

| 항목 | 값 / 역할 |
|---|---|
| `freshness_sec` | 기존 기본값 1.0초. 입력 timestamp 수락에만 사용 |
| `source_timeout_sec` | 새 기본값 **3.0초**. 마지막으로 수락한 입력 이후 수신이 끊기면 명시적으로 삭제 |
| watchdog | steady clock, 0.1초 timer |
| 캐시 refresh | 1.0초 간격, ADD만 발행. 기존 Volatile RViz의 늦은 접속 지원 |

동일 geometry의 새로운 입력도 watchdog을 연장한다. 오래되거나 역순인 입력은 현재 유효 표시를 지우지 않으며 heartbeat도 연장하지 않는다. 따라서 소스가 사라졌는데 무기한 남는 방식으로 바꾸지는 않았다. 실제 입력 중단 시 보통 3초와 다음 watchdog tick 이내에 정리한다.

이 계약은 이전 `s4_rviz_readability.md` H항에 기록된 당시 최대 1초 marker lifetime을 대체한다.

## E. namespace / ID 안정성

기존 namespace를 유지했다.

| 역할 | namespace | ID |
|---|---|---|
| MGeo links / nodes | `mgeo_links` / `mgeo_nodes` | 기존 지도별 ID / 0 |
| Global / Local | `global_path` / `local_path_debug` | 각각 0 |
| Current / Original / Adjacent | `current_lane` / `original_lane` / `adjacent_lane` | 각각 0 |
| ENTRY / HOLD / RECOVERING | `lane_change_transition` / `adjacent_lane_hold` / `lane_change_recovery` | 각각 0 |
| Recovery gate | `lane_change_recovery_gate` | 0 |
| Obstacle footprint | `obstacle_footprint` | 기존 8점 그룹 순서의 0..N-1 |
| Ego mirror | `ego_vehicle` | 0 |

각 역할의 topic이 분리되어 있고 namespace도 충돌하지 않는다. 장애물 update에서 삭제 ID와 현재 ADD ID는 서로 겹치지 않는다. Perception에 없는 object identity를 새로 가정하지 않았으며, footprint 8점 순서와 geometry는 보존했다.

## F. duplicate publisher / display

읽기 전용 discovery 결과, 확인한 시각화 topic 16개 모두 publisher가 **각각 1개**였다.

- `route_editor_node`: `/route_editor/mgeo_links`, `/route_editor/mgeo_nodes`, `/route_editor/vehicle_marker`, `/route_editor/waypoint_markers`, `/route_editor/preview_path`.
- `planning_visualization_node`: `/planning/global_path_marker`, `/planning/current_lane_marker`, `/planning/original_lane_marker`, `/planning/adjacent_lane_marker`, `/planning/local_path_marker`, `/planning/lane_change_entry_marker`, `/planning/adjacent_lane_hold_marker`, `/planning/lane_change_recovery_marker`, `/planning/recovery_gate_marker`, `/planning/obstacle_footprint_markers`, `/planning/ego_vehicle_marker`.

동일 marker topic을 두 노드가 공유하는 현상은 관측되지 않았다. 저장된 `.rviz`는 15개 display이며 topic 중복이 없다. 실제 GUI에서 사용자가 추가한 display까지 조회한 것은 아니다.

출력 QoS는 기존 Reliable/Volatile/depth 10, RViz 동적 display는 Reliable/Volatile/depth 1을 유지했다. 정적 지도와 원본 Ego의 기존 Transient Local도 유지했다. 같은 출력 topic에 서로 다른 durability의 writer가 교차 발행하는 현상은 없었다. 시각화 입력 구독 큐만 최신 1개로 줄였다. Planning/Control의 구독 QoS는 변경하지 않았다.

원본/출력 frame은 관측 구간에서 `map`이었다. 입력 timestamp와 route epoch 검사는 유지하며 역순 입력이 새 geometry를 덮어쓰지 못하게 했다.

## G. 수정 내용

1. renderer lifetime과 입력 freshness/source timeout을 분리했다.
2. geometry/pose/style signature와 marker 캐시를 두어 동일 입력은 heartbeat만 갱신한다.
3. geometry가 바뀌면 동일 ID의 ADD로 교체한다. 캐시는 1초마다 ADD로만 refresh하여 Volatile late join을 지원한다.
4. 장애물은 사라진 ID만 삭제하며 매 프레임 DELETEALL을 제거했다. 실제 8-point 닫힌 윤곽은 그대로다.
5. 중복 DELETE를 억제하고 BLOCKED/해제 상태의 phase 경로가 늦은 callback으로 다시 ADD되지 않게 했다. 실제 Planning 상태는 수정하지 않는다.
6. 시각화 입력은 최신 sample 1개만 대기하도록 하여 이전 cycle의 처리 지연 누적을 줄였다.
7. 현재 개선된 색/alpha/두께/Z 레이어와 빨간 Ego pose/scale을 유지했다. Global/Local의 point 순서, 중복, XYZ는 그대로 복사한다.

## H. 변경 파일

`ros2_ws/src/edgenode_planning/` 아래:

- `edgenode_planning/planning_visualization_node.py`
- `test/test_planning_visualization.py`
- `test/test_planning_visualization_ros.py`
- `docs/s4_rviz_flicker.md`

`route_editor_node.py`, `rviz/kcity_route_editor.rviz` 및 Planning/Localization/Perception/Control 소스는 이번에 변경하지 않았다. 기존 다른 미커밋 변경은 보존했다. 증거 디렉터리: `/workspace/logs/kcity_s4_rviz_flicker/`.

## I. 테스트 / build 결과

- 전체 workspace 회귀: **683 passed, 1 skipped, 2 warnings**, 13.20초. 경고는 기존 logger flake8의 SelectableGroups 비권장 사용 2건이다.
- 회귀 범위: age 0.99초인 입력의 sub-frame lifetime 방지, 같은 geometry/Current의 캐시, 움직이는 Local의 ADD update, source heartbeat, 역순/오래된 입력, 반복 빈 path, BLOCKED/NOT_READY 재추가 방지, 장애물 ID 감소/반복 빈 입력, 1회 watchdog 삭제, clock 되돌림, Global/Local 점 순서/중복/원본 메시지 보존.
- 실제 ROS 전송 회귀: 이전 TTL보다 오래 유지된 뒤 watchdog으로 삭제됨을 확인했다. 늦은 Volatile subscriber가 추가 source 발행 없이 캐시 refresh를 수신하는 것도 확인했다. 테스트는 별도 domain과 remap을 사용했다.
- 수정 전 스냅샷을 별도 프로세스 메모리에 로드한 negative control: 새 회귀 **3개가 의도대로 실패**했다. 짧은 lifetime, 무변경 geometry의 반복 발행, 매 obstacle update의 DELETEALL을 각각 재현했다. 실제 소스 파일을 과거 버전으로 덮어쓰지 않았다. 수정 후 전체 회귀는 위와 같이 통과했다.
- 수정 전후 비시각화 Python/YAML/RViz 파일 **77개 해시 동일**. Route Editor와 RViz 설정도 포함한다.
- `PATH_STYLES`, `DISPLAY_Z_OFFSETS_M` 동일. 기타 Current/Obstacle 스타일은 새 코드에서도 기존 값 유지.
- RViz YAML parsing / display-topic 중복 검사: PASS.
- `colcon build --symlink-install --packages-select edgenode_planning`: **PASS**, 1 package, 1.37초.
- `git diff --check`: **PASS**.

```bash
cd /workspace/ros2_ws/src/edgenode_logger
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q /workspace/ros2_ws/src
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select edgenode_planning
cd /workspace
git diff --check
```

현재 환경에 RViz2 GUI가 없어 수정 후 실제 창 확인은 남아 있다. 실제 marker 전송에서 삭제 반복과 짧은 lifetime이 제거된 것까지 확인했다. 실제 입력 관측용 candidate node는 관측 후 종료했으며 기존 runtime node는 교체하지 않았다.

## J. 필요한 재실행

**기존 `planning_visualization_node`만 종료하고 재실행한다.** 새 인스턴스를 기존 인스턴스와 동시에 실행하지 않는다.

```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 run edgenode_planning planning_visualization_node --ros-args \
  -p mgeo_dir:=/workspace/local_data/kcity_mgeo
```

기존 실행에서 다른 MGeo 경로나 ROS clock 설정을 사용했다면 같은 값을 전달한다. 현재 설정/색상 파일은 그대로이므로 **Route Editor, RViz, Planning, Localization, Perception, Control의 재시작은 필요 없다.** `route_editor.launch.py` 전체를 다시 실행할 필요도 없다.
