# S4 RViz 가독성 정리: A–K 최종 보고

작업 위치는 `/workspace`, 브랜치는 `k-city`다. 이번 작업은 시각화 스타일과 RViz display 구성에 한정했다. 기존 미커밋 작업은 유지했다. Planning, Dijkstra, Local Path 생성, Lattice, Control, Localization, Perception, MGeo geometry 및 실제 Planning topic의 의미는 변경하지 않았다. MORAI 주행, commit, push는 수행하지 않았다.

## A. 기존 RViz가 잘 안 보였던 직접 원인

- 배경 RGB `(25, 28, 34)`에 비해 MGeo link의 alpha가 `0.25`, 폭이 `0.035 m`로 낮고 가늘었다. 기존 RGB도 `(0.45, 0.52, 0.60)`이었다.
- Original/Adjacent 중심선은 폭 `0.09 m`, alpha `0.8`, Current는 폭 `0.12 m`였다.
- Local은 폭 `0.32 m`, 단계별 경로는 `0.48 m`였다. 단계별 색 선이 더 굵어서 같은 위치의 흰 Local을 가릴 수 있었다. 같은 평면에 겹친 선도 구별하기 어려웠다.
- 기존 display는 Marker/MarkerArray를 구독한다. 색과 두께는 publisher가 지정하므로 `.rviz` 수정과 함께 두 시각화 publisher의 스타일 값을 수정했다.

## B. 수정한 RViz display 목록

저장 파일: `ros2_ws/src/edgenode_planning/rviz/kcity_route_editor.rviz`.

| 순서 / 이름 | 기본 상태 |
|---|---|
| 01 MGeo Roads - visible background | ON |
| 02 MGeo Nodes - optional editing | OFF |
| 03 Global Path - BRIGHT GREEN | ON |
| 04 Current Lane - BLUE | ON |
| 04b Original Lane - CYAN context | ON |
| 05 Selected Adjacent Center - YELLOW | ON |
| 06 Local Path - CONTROL follows WHITE | ON |
| 07 Lane Change ENTRY - CYAN stripe | ON |
| 08 Adjacent Lane HOLD - PURPLE stripe | ON |
| 09 Recovery Path - PINK stripe | ON |
| 09b Recovery gate - reference rear + margin | ON |
| 10 Obstacles - ORANGE closed footprints | ON |
| 11 Ego Vehicle - RED, fresh | ON |
| 12 Selected Waypoints - draft | ON |
| 13 Draft Preview - NOT active route | OFF |

Fixed Frame `map`, 배경, 기존 TopDownOrtho view/중심/배율, 도구 및 display QoS는 유지했다. MGeo Nodes와 Draft Preview는 필요할 때 켜도록 기존 OFF 상태를 유지했다.

## C. topic / marker / namespace

신규 topic이나 중복 publisher를 만들지 않았다. 아래 표는 `planning_visualization_node`가 발행하는 시각화 topic 전체와 Route Editor의 지도 출력을 포함한다.

| 원본 입력 | RViz 출력 | namespace / ID |
|---|---|---|
| 기존 Route Editor MGeo link 집합 | `/route_editor/mgeo_links` | `mgeo_links` / 정렬된 link별 ID |
| 기존 Route Editor MGeo node 점 | `/route_editor/mgeo_nodes` | `mgeo_nodes` / 0 |
| `/planning/global_path` | `/planning/global_path_marker` | `global_path` / 0 |
| `/planning/current_link` + 동일 MGeo | `/planning/current_lane_marker` | `current_lane` / 0 |
| `/planning/original_lane_center` | `/planning/original_lane_marker` | `original_lane` / 0 |
| `/planning/adjacent_lane_center` | `/planning/adjacent_lane_marker` | `adjacent_lane` / 0 |
| `/planning/local_path` | `/planning/local_path_marker` | `local_path_debug` / 0 |
| `/planning/lane_change_entry_path` | `/planning/lane_change_entry_marker` | `lane_change_transition` / 0 |
| `/planning/adjacent_lane_hold_path` | `/planning/adjacent_lane_hold_marker` | `adjacent_lane_hold` / 0 |
| `/planning/lane_change_recovery_path` | `/planning/lane_change_recovery_marker` | `lane_change_recovery` / 0 |
| `/planning/lane_change_recovery_gate` | `/planning/recovery_gate_marker` | `lane_change_recovery_gate` / 0 |
| `/planning/obstacles` | `/planning/obstacle_footprint_markers` | `obstacle_footprint` / 8점 그룹별 ID |
| `/route_editor/vehicle_marker` | `/planning/ego_vehicle_marker` | 원본 `vehicle` / 0, mirror `ego_vehicle` / 0 |

MGeo link/node 출력의 publisher는 `route_editor_node`다. `/planning/*_marker(s)`의 publisher는 `planning_visualization_node`다. 후자는 `/planning/state`, `/planning/lane_change_state`도 구독하여 기존 marker 삭제 정책에 사용한다.

Route Editor의 다른 기존 출력은 `/route_editor/waypoint_markers` (`waypoints`, `waypoint_numbers`), `/route_editor/preview_path` (`Path`), `/route_editor/status` (`String`, 비시각화)다. Draft preview는 활성 Global/Local과 혼동되지 않도록 기본 OFF다.

## D. line width / alpha / 표시 우선순위

폭 단위는 world metre, RGB 범위는 0–1이다.

| 역할 | 변경 전 폭 / alpha | 변경 후 폭 / alpha | 변경 후 RGB |
|---|---|---|---|
| MGeo Roads | 0.035 / 0.25 | **0.14 / 0.90** | (0.65, 0.72, 0.80) |
| Global | 0.28 / 1.0 | **0.46 / 1.0** | (0.20, 1.00, 0.35) |
| Local | 0.32 / 1.0 | **0.70 / 1.0** | (1.00, 1.00, 1.00) |
| Current | 0.12 / 1.0 | **0.30 / 1.0** | (0.20, 0.65, 1.00) |
| Original context | 0.09 / 0.8 | **0.24 / 1.0** | (0.20, 0.80, 0.95) |
| Selected Adjacent | 0.09 / 0.8 | **0.34 / 1.0** | (1.00, 0.88, 0.08) |
| ENTRY | 0.48 / 1.0 | **0.34 / 1.0** | (0.05, 1.00, 1.00) |
| HOLD | 0.48 / 1.0 | **0.34 / 1.0** | (0.78, 0.50, 1.00) |
| RECOVERING | 0.48 / 1.0 | **0.34 / 1.0** | (1.00, 0.28, 0.70) |
| Recovery gate | 0.08 / 0.8 | **0.16 / 1.0** | (0.55, 0.90, 1.00) |
| Obstacle outline | 0.10 / 0.95 | **0.22 / 1.0** | (1.00, 0.40, 0.08) |

MGeo node sphere 지름은 `1.0 → 0.45 m`, RGBA는 `(0.40, 0.80, 1.0, 1.0)`이다. 점을 켰을 때 경로를 덜 가리도록 했다. 기본 OFF인 Draft Preview의 폭은 `0.15 → 0.25 m`다. Ego는 기존 빨강 RGBA `(1.0, 0.3, 0.2, 1.0)`과 ARROW scale `(4.0, 1.5, 1.5)`를 유지했다.

Displays 순서는 역할별 관리 순서다. 겹친 선의 가독성은 폭과 표시 전용 Z 레이어로 조정했다. Original `+0.02 m`, Adjacent `+0.04 m`, Current `+0.05 m`, 각 phase `+0.08 m`, gate `+0.10 m`를 **marker.pose.position.z만** 사용해 표시한다. 원본 point 배열은 바꾸지 않는다. Global/Local marker pose는 identity를 유지하여 표시 높이도 바꾸지 않는다. 모든 XY는 원본과 같다.

## E. Global / Local / Current / Adjacent 구분

- MGeo는 눈에 보이는 회청색 배경, Global은 더 굵은 밝은 녹색, Control용 Local은 가장 굵은 흰색이다.
- Current는 현재 map-matched link의 기존 MGeo 점열을 파랑으로 표시한다. Original은 기존 original-lane 진단 점열을 하늘색으로 보조 표시한다.
- Adjacent는 Planning이 선택하여 기존 topic으로 발행한 중심선을 노랑으로 표시한다. 이번 작업에서 선택이나 offset은 변경하지 않았다. 기존 고정 scene의 `-3.474670896 m`를 동일 좌표에서 확인할 수 있다.
- Global/Local mirror는 모든 point를 순서대로 deep-copy한다. 재탐색, 간소화, 순서 변경, 중복 제거를 하지 않는다. Control 입력은 기존 `/planning/local_path`다.
- 기존 분석의 16 links / 1,313 points route 계약을 유지했다. 이번에는 실시간 route를 재측정하거나 Dijkstra 경로를 새로 계산하지 않았다. 관련 소스 해시 불변과 기존 point 순서/중복 보존 회귀로 확인했다.

## F. ENTRY / HOLD / RECOVERING 표시

기존 세 topic을 각각 기존 namespace와 별도 display로 표시한다. ENTRY는 cyan, HOLD는 purple, RECOVERING은 pink다. 폭 `0.34 m`의 색 띠가 폭 `0.70 m`의 흰 Local에 겹칠 때 흰 테두리가 남도록 했다.

Planning이 실제로 발행한 단계별 점열만 표시한다. 미래 Recovery나 BLOCKED 상태의 경로를 표시 코드에서 생성하지 않는다. 기존 phase 삭제 정책도 유지했다. 따라서 동시에 보이는 구간은 해당 시점의 Planning 진단 출력에 따른다. 비교에 사용한 기존 ENTRY scene에는 Recovery가 없으며, 이미지에도 그리지 않았다.

## G. obstacle footprint 표시

기존 `/planning/obstacles` 입력의 점을 그대로 8점씩 연결하고 첫 점을 끝에 추가하여 9점의 닫힌 `LINE_STRIP`으로 표시하는 기존 구현을 사용한다. 이번에는 밝은 orange, 폭 `0.22 m`로 변경했다. Perception/Planning의 AABB나 clearance 계산은 변경하지 않았다.

8점 단위가 아닌 입력은 기존 POINTS fallback을 유지하고 크기만 `0.22 × 0.22 m`로 키웠다. 표시 코드가 불명확한 그룹의 사각형을 임의로 만들지 않는다. 정상 8-point 입력은 닫힌 윤곽으로 보여 Local/Adjacent와의 관계를 확인할 수 있다.

## H. stale marker 처리

QoS, lifetime, 삭제 로직은 이번에 변경하지 않았다.

| 대상 | publisher QoS / lifetime | 기존 cleanup |
|---|---|---|
| MGeo Roads / Nodes | Reliable, Transient Local, depth 1 / 무기한 | 정적 지도. 발행 시 DELETEALL. 새 스타일 반영에는 Route Editor도 재시작 필요 |
| Draft waypoints | Reliable, Transient Local, depth 1 / 무기한 | 편집 시 DELETEALL 후 현재 값 발행 |
| Draft preview Path | Reliable, Transient Local, depth 1 / Path에 lifetime 없음 | 최신 Path로 교체, 무효 시 빈 Path. display 기본 OFF |
| Planning 진단 marker 전체 | Reliable, Volatile, depth 10 / 최대 1초 | 빈 값/무효/만료 시 DELETE. 장애물은 매번 DELETEALL 후 현재 그룹 발행 |
| 원본 Ego `/route_editor/vehicle_marker` | Reliable, Transient Local, depth 1 / 무기한 | 기존 동작 유지. 무효 frame/pose는 DELETE |
| 표시용 Ego `/planning/ego_vehicle_marker` | Reliable, Volatile, depth 10 / 최대 1초 | 오래된 retained marker 거부, 수신 정지 시 DELETE. RViz는 이 기존 mirror 사용 |

RViz의 동적 display는 Reliable / Volatile / depth 1로 구독한다. 기존 namespace/ID 조합을 유지한다. Route geometry 교체 시 이전 동적 path를 삭제하고, 이전 route 시각의 지연 path를 거부한다. ROS 시각이 되돌아가면 route epoch를 초기화하고 marker를 삭제한다. 수신 정지는 steady timer로 확인한다.

Ego의 pose/color/scale은 원본 marker의 copy다. Route Editor가 유효한 `/localization/odometry`로 설정한 XY, 정규화한 orientation, 기존 표시용 Z +1 m를 그대로 유지한다. 원본 odometry callback과 mirror의 vehicle callback 모두 이번에 변경하지 않았다.

## I. 이번 변경 파일

`ros2_ws/src/edgenode_planning/` 아래의 다음 파일만 이번 작업으로 변경했다.

1. `edgenode_planning/route_editor_node.py`: `_publish_map`의 link/node 스타일.
2. `edgenode_planning/planning_visualization_node.py`: marker 스타일과 표시 전용 Z 레이어.
3. `rviz/kcity_route_editor.rviz`: 기존 display 이름/순서 및 기본 OFF인 preview의 폭.
4. `docs/s4_rviz_readability.md`: 이 보고서.

다른 dirty/untracked 파일은 기존 작업에서 발생한 것이다. 작업 전 스냅샷, 이번 변경만의 diff, 해시 검증, 테스트/build 로그, 비교 이미지는 `/workspace/logs/kcity_s4_rviz_readability/`에 저장했다.

## J. build / 검증 결과

- RViz YAML parsing: PASS. 15개 display의 이름은 중복이 없고, 기존 topic 집합, 배경, view, QoS를 유지했다.
- 비시각화 Python/YAML 소스 77개의 SHA-256: 변경 0. Planning/Control/Localization/Perception/FSM과 기존 테스트/설정을 포함한다.
- 변경한 두 Python 파일에서 의도한 색/폭/표시 레이어만 원래 값으로 정규화하면 작업 전 AST와 일치한다. 분기, 입출력, 원본 점열, Ego, freshness, 삭제 처리는 유지했다.
- 전체 ROS workspace 회귀: **665 passed, 1 skipped, 2 warnings**, 8.15초. 경고는 기존 logger flake8의 SelectableGroups 비권장 사용 2건이다. 테스트 파일은 변경하지 않았다.
- 기존 시각화 회귀는 Global/Local의 점 순서/중복/입력 불변, 지도 점열, 8-point 윤곽/그룹 수 감소, 이전 route 삭제, Ego copy/만료, ROS clock 되돌림을 검증한다.
- 실제 ROS 시각화 node/transport 테스트도 PASS. `ROS_DOMAIN_ID=173`, `ROS_LOCALHOST_ONLY=1`, 모든 project topic을 테스트 전용 이름으로 remap했다. 실제 MORAI/Control에는 발행하지 않았다.
- `colcon build --symlink-install --packages-select edgenode_planning`: **PASS**, 1 package. 설치된 RViz 설정과 source가 일치하며 시각화 실행 파일도 존재한다.
- `git diff --check`: **PASS**.

검증 명령은 다음과 같다. 기존 logger lint가 cwd를 순회하므로 해당 디렉터리에서 전체 src를 지정했다.

```bash
cd /workspace/ros2_ws/src/edgenode_logger
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q /workspace/ros2_ws/src
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select edgenode_planning
cd /workspace
git diff --check
```

현재 컨테이너에는 RViz2 GUI가 없어 실제 RViz 창 확인은 남아 있다. YAML, marker payload, 실제 ROS 전송, 설치 결과를 확인했다. `before_after_style_preview.png`는 기존 고정 scene의 동일한 XY에 이전/새 스타일을 적용한 Matplotlib 비교 이미지이며 RViz 스크린샷이 아니다. 이 비교는 저장된 scene만 읽으며 경로 재계산, bag 재생, MORAI 주행을 수행하지 않는다.

## K. RViz 재실행 명령 및 정지 상태 확인

기존 Route Editor / planning_visualization_node / 해당 RViz를 일반 종료하여 중복 publisher를 피한 뒤, RViz2가 설치된 ROS GUI 환경에서 실행한다. MGeo 스타일은 publisher에서 정하므로 RViz만 다시 열면 기존 publisher의 이전 색과 폭이 유지된다.

```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch edgenode_planning route_editor.launch.py use_rviz:=true
```

이 launch는 Route Editor, 시각화 node, RViz를 실행한다. Planning/Control의 주행 launch는 실행하지 않는다. 동적 경로 표시에는 기존 정지 상태의 Planning 입력이 필요하다. 표시를 위해 새 route를 생성하거나 activate할 필요는 없다.

publisher를 이미 재시작했다면 RViz만 열 수도 있다.

```bash
rviz2 -d /workspace/ros2_ws/src/edgenode_planning/rviz/kcity_route_editor.rviz
```

정지 상태 확인 사항:

- 회청색 도로 형상이 배경과 명확히 구별되고, 밝은 녹색 Global이 더 굵게 보인다.
- 빨간 Ego의 위치/방향이 실제 정지 상태와 일치하고 흰 Local을 즉시 찾을 수 있다.
- 파란 Current와 노란 Adjacent가 구별된다. 기존 S4 scene에서는 약 3.475 m 떨어진 인접 중심선이다.
- 발행 중인 ENTRY/HOLD/RECOVERING이 cyan/purple/pink로 구별되고 흰 Local 테두리가 보인다.
- orange 닫힌 윤곽이 8-point obstacle과 일치하고 Local/Adjacent와의 공간을 확인할 수 있다.
- 입력 중단 후 동적 marker는 최대 1초 lifetime과 다음 cleanup tick에 따라 사라진다. 이전 route/장애물/Ego가 남지 않는다.
- 미발행 단계와 BLOCKED에서 삭제된 단계는 표시되지 않는다.
