# K-City RViz2 Route Editor

RViz의 Publish Point로 **파란 MGeo node 점 근처를 순서대로 클릭**하면 번호가
붙은 waypoint와 자홍색 Preview가 생성된다. 회색 선은 MGeo 도로망이다.
Preview는 기존 `DijkstraPlanner.plan_route()`를 사용하며, 실제 Planning 경로는
Activate를 성공시킬 때만 바뀐다. Local Path Progress Lock은 그대로 사용한다.

## 실행 환경과 좌표

- `map` local 좌표만 사용한다. MGeo origin을 더하거나 빼지 않는다.
- 이 프로젝트 소스에 클릭 frame 변환을 위한 TF tree가 없어, `map` 이외의
  PointStamped는 `FRAME_REJECTED`로 거절한다. 새 TF를 만들지 않는다.
- 지도는 처음에 한 번만 생성/발행한다. MarkerArray, Preview, status는
  reliable / transient-local / depth 1로 발행해 RViz가 늦게 열려도 받을 수 있다.
- 실제 컨테이너 검사 시 RViz2 패키지는 없었다. GUI 렌더링은 검증하지 않았다.
  RViz2가 설치된 GUI 환경에서는 전용 launch가 RViz를 함께 실행한다. 없으면
  명확한 로그와 함께 Editor만 실행한다. RViz 실행 환경에 필요한 패키지는
  `ros-humble-rviz2`이며 Docker/DDS 설정은 이 구현에서 변경하지 않는다.

## 터미널별 전체 사용 순서

### 먼저 MORAI

기존 프로젝트 방식으로 MORAI K-City 시뮬레이션과 네트워크를 실행한다.
이 문서의 주행 명령은 사용자가 실제 시뮬레이터에서 수행할 명령이다.
구현/테스트 중 MORAI 차량을 주행시키지 않았다.

### 터미널 1 — autonomy

기존 실행 중인 autonomy가 있으면 중복 실행하지 않는다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

### 터미널 2 — Editor와 RViz

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch edgenode_planning route_editor.launch.py
```

GUI 없이 Editor만 실행:

```bash
ros2 launch edgenode_planning route_editor.launch.py use_rviz:=false
```

Planning 없이 offline Preview를 할 때만 시작 링크를 직접 지정한다.
시작 링크를 캡처할 때 유효한 `/planning/current_link`가 있으면 그것이 우선이다.
첫 waypoint를 선택하면 그 시점의 시작 링크가 Draft에 고정된다. 이후 live link
변화만으로 Preview를 다시 계산하지 않는다. 링크가 없으면 waypoint를 유지하고
`WAITING_FOR_CURRENT_LINK`로 기다리다가 첫 유효 링크를 한 번 캡처한다.
Clear 또는 마지막 waypoint Undo는 lock을 해제하며, 다음 클릭에서 다시 캡처한다.

```bash
ros2 launch edgenode_planning route_editor.launch.py \
  start_link_id:=A219BS010380
```

이미 설치된 RViz2에서 설정 파일을 직접 열 수도 있다.

```bash
rviz2 -d /workspace/ros2_ws/src/edgenode_planning/rviz/kcity_route_editor.rviz
```

RViz에서 Fixed Frame이 `map`인지 확인하고 **Publish Point** 도구를 선택한다.
파란 node가 보일 때까지 확대하고, 원하는 도로의 node에서 5 m 이내를 클릭한다.
이 구현은 nearest-node snap이다. 긴 링크의 중간을 클릭해 node에서 5 m보다
멀면 거절한다. 교차로에서 원하는 직진/좌회전/우회전 도로 쪽으로 이동한 뒤
그 도로의 node를 찍는다. 화면의 번호가 통과 순서다. 일부러 같은 node를
나중에 다시 통과하는 코스는 허용하며, 연속 중복 클릭은 무시한다.

클릭할 때마다 자홍색 Preview를 확인한다. 녹색은 현재 활성 Planning Global
Path라서 Preview와 다를 수 있다. 실제 활성 경로를 가려 Preview가 안 보이면
잠시 `Active Planning Global Path` display를 끈다.

### 터미널 3 — 편집, 저장, 복원, 적용

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 편집/활성화 전 주행을 비활성화한다.
ros2 param set /planning_node enable_drive false

# 마지막 waypoint 제거 또는 draft 전체 초기화
ros2 service call /route_editor/undo std_srvs/srv/Trigger '{}'
ros2 service call /route_editor/clear std_srvs/srv/Trigger '{}'
```

Undo/Clear는 필요할 때만 호출한다. Clear 후에는 다시 waypoint를 클릭한다.
원하는 Preview를 확인한 후 저장한다.

```bash
ros2 param set /route_editor_node route_name kcity_demo_route
ros2 service call /route_editor/save std_srvs/srv/Trigger '{}'
```

저장 위치는 `/workspace/local_data/kcity_routes/kcity_demo_route.yaml`이다.
같은 이름으로 Save하면 해당 파일을 원자적으로 갱신한다. 다른 코스는 다른
route_name을 사용한다. 폴더는 Save 시 필요한 경우 생성된다.

나중에 복원할 때:

```bash
ros2 param set /route_editor_node route_name kcity_demo_route
ros2 service call /route_editor/load std_srvs/srv/Trigger '{}'
```

Load는 node 존재 여부, frame, 지도 경로와 파일 형식을 먼저 검사한다. 오류면
기존 draft와 lock을 유지한다. 성공하면 그 시점의 current link(없으면 offline
start_link_id)를 새로 캡처해 고정한다. 둘 다 없으면 waypoint만 복원하고 기다린다.
도달할 수 없는 코스는 marker는 보이지만 Preview를 비우고 `ROUTE_NOT_FOUND`를
표시한다. **Load는 Activate를 수행하지 않는다.**

Preview가 정상일 때 적용한다.

```bash
ros2 service call /route_editor/activate std_srvs/srv/Trigger '{}'
```

반드시 `success: true`, `ROUTE_ACTIVATED`를 확인한다. Activate는 다음을 검사한다.

1. waypoint가 하나 이상이고 모두 실제 node이며 Draft에 고정된 시작 링크에서 도달 가능함.
2. Planning 파라미터 서비스가 준비됨.
3. Planning의 `enable_drive`가 false임.
4. Planning이 같은 MGeo 디렉터리와 `map` frame을 사용함.
5. 검증 도중 Draft가 바뀌지 않음. live current_link 변화는 lock을 바꾸지 않음.
6. `enable_drive=false`와 `route_node_ids`를 한 atomic parameter 요청으로 적용해 성공함.

Activate 중에는 draft 편집을 잠근다. Activate는 차량을 enable하지 않는다.
Planning은 기존 timer에서 route_node_ids 변경을 감지해 경로와 진행 index를
초기화한다. 파라미터 적용 성공과 Global Path 생성은 서로 다른 단계이므로
다음 확인을 반드시 한다.

```bash
ros2 param get /planning_node route_node_ids
ros2 topic echo /planning/state --once
ros2 topic echo /planning/map_match_status --once
```

`GLOBAL_PATH_READY`, `MATCHED` 및 RViz의 녹색 Global Path 형상과 waypoint 순서를
확인한다. 편집 시작 이후 차량이 이동했다면 Preview와 활성 경로를 다시 비교한다.

그 뒤 현재 프로젝트 기준인 **10 km/h / 60초** 테스트를 수행한다.

```bash
ros2 param set /planning_node enable_drive false
ros2 param set /planning_node max_drive_speed_kmh 10.0
ros2 param set /planning_node drive_duration_sec 60.0
sleep 1
ros2 param set /planning_node enable_drive true
```

기존 YAML의 cruise speed는 10 km/h다. 기존 곡선 감속과 주행 lease는 유지된다.
즉 10 km/h는 직선 기준/상한이며 곡선에서 기존 속도 계획이 더 낮은 속도를
선택할 수 있다. 직접 정지하려면:

```bash
ros2 param set /planning_node enable_drive false
```

## 토픽

| 기본 이름 | 타입 | 역할 |
|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz 클릭 입력 |
| `/planning/current_link` | `std_msgs/msg/String` | 새 Draft/Load의 시작 링크 캡처 후보 |
| `/localization/odometry` | `nav_msgs/msg/Odometry` | 차량 위치 입력 |
| `/route_editor/mgeo_links` | `visualization_msgs/msg/MarkerArray` | 전체 도로망 |
| `/route_editor/mgeo_nodes` | `visualization_msgs/msg/MarkerArray` | 클릭할 수 있는 node 점 |
| `/route_editor/waypoint_markers` | `visualization_msgs/msg/MarkerArray` | 순번과 waypoint |
| `/route_editor/vehicle_marker` | `visualization_msgs/msg/Marker` | 차량 위치/방향 |
| `/route_editor/preview_path` | `nav_msgs/msg/Path` | 현재 draft의 Dijkstra 결과 |
| `/route_editor/status` | `std_msgs/msg/String` | 이벤트, Preview 상태, 비용, 링크 수 |

실제 Planning의 `/planning/global_path`, `/planning/local_path`,
`/planning/local_path_index`는 기존 발행자를 그대로 사용한다.

## 서비스

모두 `std_srvs/srv/Trigger`이며 `success`와 `message`를 반환한다.

| 이름 | 동작 |
|---|---|
| `/route_editor/undo` | 마지막 draft waypoint 제거, Preview 갱신 |
| `/route_editor/clear` | draft/Preview/marker 초기화, 실제 route 유지 |
| `/route_editor/save` | 현재 route_name의 YAML 저장 |
| `/route_editor/load` | 현재 route_name의 YAML을 draft로 복원 |
| `/route_editor/activate` | 검증 후 Planning 파라미터 적용, drive 비활성화 유지 |

## 파라미터

설정 파일: `config/route_editor.yaml`. route_name과 snap 거리는 runtime 변경이
가능하며, 나머지는 시작 시 설정한다. 토픽을 변경하면 RViz display도 맞춘다.

| 이름 | 기본값 |
|---|---|
| `mgeo_dir` | `/workspace/local_data/kcity_mgeo` |
| `clicked_point_topic` | `/clicked_point` |
| `current_link_topic` | `/planning/current_link` |
| `localization_topic` | `/localization/odometry` |
| `preview_path_topic` | `/route_editor/preview_path` |
| `status_topic` | `/route_editor/status` |
| `waypoint_snap_max_distance_m` | `5.0` |
| `route_name` | `kcity_demo_route` |
| `route_save_dir` | `/workspace/local_data/kcity_routes` |
| `planning_node_name` | `/planning_node` |
| `start_link_id` | 빈 문자열: live current_link 대기 |
| `show_mgeo_nodes` | `true` |
| `show_waypoint_node_ids` | `false` |
| `parameter_timeout_sec` | `3.0`, steady-clock 기준 |

## RViz 수동 설정과 확인 요소

설정 파일의 초기 카메라는 현재 K-City local 지도 범위를 내려다보도록 맞췄다.
다른 지도를 사용하면 화면을 이동/확대한다. Marker는 메시지 원본 XYZ를 사용한다.

- Fixed Frame: `map`.
- MarkerArray display: MGeo Links, MGeo Nodes, Selected Waypoints.
- Path display: Preview(자홍), Planning Global(녹색), Local(주황).
- Marker display: Vehicle(붉은 화살표).
- Tools에 `Publish Point`, Topic `/clicked_point`, Single click `false`.
- Editor 출력 display의 Durability는 `Transient Local`, Reliability는 `Reliable`.
- 기존 Planning Path display의 Durability는 `Volatile`.

## 문제 진단

```bash
ros2 topic echo /route_editor/status --qos-durability transient_local
ros2 topic echo /planning/current_link --once
ros2 topic echo /planning/local_path_index
ros2 topic echo /planning/drive_status
ros2 topic echo /control/pure_pursuit_steering
ros2 topic info /route_editor/mgeo_links --verbose
ros2 service list | rg 'route_editor|planning_node'
ros2 param get /planning_node enable_drive
ros2 param get /planning_node route_node_ids
```

| 상태/증상 | 확인할 내용 |
|---|---|
| `SNAP_TOO_FAR` | 파란 node 점 근처를 클릭. snap 거리 단위는 m |
| `DUPLICATE_WAYPOINT` | 직전과 같은 node이므로 추가하지 않음 |
| `FRAME_REJECTED` | Fixed Frame과 PointStamped frame이 map인지 확인 |
| `WAITING_FOR_CURRENT_LINK` | autonomy/Map Matching 상태 확인. offline이면 start_link_id 지정 |
| `ROUTE_NOT_FOUND` | directed graph에서 해당 순서로 갈 수 없음. Undo로 마지막 waypoint 수정 |
| `MAP_UNAVAILABLE` | mgeo_dir와 node_set/link_set 파일 확인 |
| `DRIVE_MUST_BE_DISABLED` | enable_drive false 적용 후 Activate 재시도 |
| `MAP_MISMATCH` | Editor/Planning의 mgeo_dir와 map frame 일치 여부 확인 |
| `PREVIEW_CHANGED` | 최신 Preview를 확인한 뒤 Activate 재시도 |
| `PLANNING_UNAVAILABLE` | planning_node_name과 파라미터 서비스 확인 |
| `ACTIVATION_FAILED` / timeout | 성공으로 취급하지 말고 Planning 파라미터와 Global Path 확인 |
| 지도/번호가 안 보임 | Fixed Frame, Topic QoS, display 활성화, 카메라 위치/확대 확인 |
| 차량이 60초 전에 정지 | drive_status, Map Match, 경로/속도 유효성 확인 |

파라미터 write 요청의 응답이 timeout되면 원격 적용 여부는 확정할 수 없다.
요청 자체에 enable_drive=false가 포함되어 있어 늦게 적용돼도 주행을 enable하지
않는다. 실제 route_node_ids와 Global Path를 읽고 다시 검증한다.

실제 MORAI에서는 원하는 교차로 방향, waypoint 순서, Local Path index 점프,
Pure Pursuit/최종 조향 급변, 직선 10 km/h 안정성과 60초 lease를 확인한다.
Clear/Load가 활성 경로를 바꾸지 않는 것과 Activate 후 Global Path가 바뀌는
것도 확인한다. 이 항목들은 코드/ROS 테스트와 구분되는 실제 시뮬레이터 검증이다.
