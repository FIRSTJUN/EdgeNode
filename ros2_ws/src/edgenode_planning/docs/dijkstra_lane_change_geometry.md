# K-City Global Path의 직선 chord 원인과 수정

## 결론과 변경 범위

`kcity_projection_validation_01`의 Global Path에는 MGeo의 lazy 차로 변경 link
두 개가 일반 주행 edge로 들어갔다. 이 link의 `points`는 각각 2개이며,
차로 centerline을 따르는 경로가 아니라 시작/끝을 직접 잇는 직선이다.
Dijkstra는 이 geometry를 그대로 Global Path로 내보냈다. 정상 곡선 centerline은
같은 지도에 존재한다. 원인은 차로 변경 추상 edge의 사용이며 link 연결 gap이 아니다.

`DijkstraPlanner.__init__()`에서 `lazy_init is True` 또는 비어 있지 않은
`lane_ch_link_path`를 가진 edge만 graph/경로 비용에서 제외했다. 원본 link 사전은
진단을 위해 보존한다. 정상 link의 좌/우 인접 차로 metadata는 제외 조건이 아니며,
일반 2점짜리 link도 계속 허용한다. ID, point 수, null road/type 또는 길이만으로
filter하지 않는다. 비용 가중치와 `link_length` 우선 규칙은 바꾸지 않는다.

이는 현 planner가 원본 points를 주행 경로로 사용하는 데 필요한 제한이다. 실제
차로 변경 궤적을 생성하는 기능은 추가하지 않았다. 추상 차로 변경 edge로만 갈 수
있는 목적지는 경로 없음으로 처리한다. 최초 matched start link가 추상 link인 경우도
잘못된 chord를 발행하는 대신 `None`을 반환한다. MapMatcher 자체는 변경하지 않았다.

Pure Pursuit, Local Path 2 m 보간/segment projection, PID/accel/steering,
SpeedPlanner 및 MapMatcher threshold, Lattice/FSM, 원본 map JSON은 모두 유지했다.

## 입력 자료와 재구성의 한계

- bag: `/workspace/logs/kcity_projection_validation_01` (140.644207초)
- dump: `/workspace/logs/kcity_projection_validation_01_global_path.txt`
- parameter: `/workspace/logs/kcity_projection_validation_01_planning.yaml`
- 원본 지도: `/workspace/local_data/kcity_mgeo/{node_set,link_set}.json`
- parameter의 실제 `route_node_ids`: `[A119BS010142]`.
  `goal_node_id=A119BS010245`는 비어 있지 않은 waypoint 배열 때문에 사용되지 않는다.
- 최초 기록된 `current_link`: `A219BS010380`.

bag에는 Global Path 전체가 없다. 별도 echo dump도 128개 숫자 pose 뒤에 `...`가 있어
전체 1,980점을 담지 못한다. 그 128점은 수정 전 코드로 재구성한 route의 prefix와
정확히 일치한다. dump의 header는 bag 이후에 해당한다. 따라서 전체 route link ID는
기록된 시작 link/waypoint/현재 지도/수정 전 알고리즘으로 **재구성한 값**이다.
추가로 bag의 2,786개 Local Path 양 끝점 5,572개를 재구성 route와 비교했으며 최대
XY 거리는 `2.39735e-13 m`였다. 기록된 progress에는 후퇴/reset이 없다.

아래 모든 시각은 bag 첫 메시지부터의 수신 상대 시각이다. 독립 topic을 가까운
수신 시각으로 대조했으므로 callback 내부의 완전한 동시 snapshot은 아니다.
수정 후 route/Global Path는 오프라인 산출물이며 아직 MORAI 주행 결과가 아니다.

## 실제 bag에서 확인한 주행

- `ACTIVE`/`NORMAL_DRIVE`: 6.9618초부터 약 126.982초까지. 마지막 정지는 120초
  drive lease 만료(`EXPIRED`)와 일치한다. `MATCHED`는 유지됐다.
- 38.982096초: `current_link= A219BS010402-A219BS010401`.
- 42.883286초: `current_link= A219BS010403-A219BS010404`.
- progress로 본 두 번째 chord 주행: 42.965103~57.931792초.
  raw index 528 구간의 arc 범위는 284.340679~369.623098 m다.
- 43~57초: index는 같은 긴 raw segment를 뜻하는 528이지만 arc는
  284.796~364.354 m로 증가한다. Local Path는 projection에서 매번 앞으로 이동한다.
  따라서 이전 고정 horizon 문제가 다시 발생한 현상은 아니다.
- 이 구간 420개 control sample에서 path error 평균 0.056154 m
  (0.038463~0.074591 m), target 거리 2.500296~2.501112 m,
  `front_steer=-0.014043~+0.015220`이었다. 잘못 공급된 직선을 정확히 따라간다.

| bag 시각 s | localization x, y (m) | progress m | PP / front_steer | target left / forward m | 정상 403 / 404까지 거리 m |
| --- | --- | --- | --- | --- | --- |
| 42.965 | 107.082, 1274.718 | 284.373 | +0.008399 / -0.008399 | +0.012377 / 2.50105 | 0.001 / 3.458 |
| 46.465 | 114.011, 1293.304 | 304.371 | +0.001786 / -0.001786 | +0.002631 / 2.50059 | 2.119 / 5.590 |
| 50.698 | 122.479, 1315.969 | 328.565 | -0.000562 / +0.000562 | -0.000828 / 2.50043 | 3.204 / 6.697 |

이 행들의 `current_link`는 모두 두 번째 lazy link다. 마지막 행에서 target을
localization 좌표계로 역변환하면 약 `(123.310684, 1318.326905)`다.
가까운 50.683984초 Local Path는 11점/20 m이며 첫 점
`(122.313045, 1315.659054)`, 마지막 점 `(129.318240, 1334.392106)`이다.
path 전체가 직선 chord 위에 놓인다. `path_error=0.046477 m`인 동시에 정상
404 centerline까지 6.696775 m 떨어져 있다. 정상 403과도 약 3.204 m 떨어졌다.

곡선 centerline에서 벗어나는 양은 약 43초부터 증가한다. MORAI ego 메시지의
`distance_left_lane_boundary`는 43.0059초 1.7937 m → 44.0071초 1.1399 m →
45.0087초 0.5267 m → 46.0103초 0.0220 m로 감소한다. 따라서 약 43~46초를
곡선 이탈/경계 접근 구간으로 특정할 수 있다. 영상, 차체 폭, lane boundary 원본은
없으므로 바퀴가 경계를 넘은 정확한 한 시각까지 단정하지 않는다. 이 값은 MORAI가
가장 가까운 link의 차선 경계에 대해 제공하는 거리다. MORAI의 `cross_track_error`는
좌/우 경계 거리 차이이므로 centerline 오차로 해석하지 않았다.
[공식 EgoVehicleStatus 정의](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/ros-2)

## 문제 link의 원본 metadata와 모든 points

두 link 모두 `lazy_init=true`, `max_speed=30`, 좌/우 lane-change dst는 null이고,
`link_type`, `road_id`, `ego_lane`, `lane_change_dir`, `hov`도 null이다.
둘 다 `link_length` 필드 자체가 없다. 아래는 원본 XYZ 값 전체다.

```text
A219BS010402-A219BS010401
  from_node_idx: A119BS010698
  to_node_idx:   A119BS010157
  lane_ch_link_path: [A219BS010402, A219BS010401]
  points:
    [99.29995859932387, 1253.3509978000075, -0.5369995269775387]
    [106.82295860990416, 1274.235997798387, -0.4940002593994137]
  XY point gap / polyline length: 22.198620544788422 m

A219BS010403-A219BS010404
  from_node_idx: A119BS010157
  to_node_idx:   A119BS010702
  lane_ch_link_path: [A219BS010403, A219BS010404]
  points:
    [106.82295860990416, 1274.235997798387, -0.4940002593994137]
    [136.69395864993567, 1354.1159978010692, -0.5049999389648434]
  XY point gap / polyline length: 85.28241931265829 m
```

각 link 내부가 이미 직선이다(A). metadata상 차로 변경 추상 link를 실제 주행
edge로 사용한 문제다(C). 수정 전/후 route의 모든 link join XYZ gap은 0이므로
서로 다른 link를 잘못 붙여 새 직선을 만든 경우(B)는 아니다.

## 정상 인접 geometry와 비용

다음 link는 `lazy_init=false`, `lane_ch_link_path=[]`, `link_type="6"`이다.
`road_id`는 여기에도 null이므로 동일 도로 여부는 road ID로 증명할 수 없지만,
좌/우 인접 metadata와 좌표로 같은 구간의 인접 차로 관계를 확인할 수 있다.

| link ID 끝자리 | from → to node 끝자리 | points | XY polyline m | 자체 시작/끝 chord에서 최대 이격 m | 인접 link |
| --- | --- | ---: | ---: | ---: | --- |
| 401 | 230 → 157 | 45 | 21.961664 | 0.124698 | right=402 |
| 402 | 698 → 699 | 45 | 21.985484 | 0.050338 | left=401 |
| 403 | 157 → 701 | 172 | 85.072811 | 5.101786 | right=404 |
| 404 | 699 → 702 | 175 | 86.673909 | 5.215713 | left=403, right=405 |
| 405 | 700 → 703 | 178 | 88.191918 | 5.952416 | left=404 |

따라서 곡선을 복원하기 위한 spline 추정은 필요 없다. 정상 402→404가 동일한
698 노드에서 출발해 동일한 702 노드로 연결된다. 기존 lazy 2개는 107.481040 m,
정상 402+404는 108.659393 m다. 단순 길이 비용에서는 lazy 경로가 **1.178354 m**
짧아 선택됐다. 이 지도 352개 link 모두 `link_length` 필드가 없으며 현재 코드는
XY polyline 길이 fallback을 사용한다. 잘못된 `link_length` 수치가 원인은 아니다.

## 수정 전/후 route와 Global Path

```text
before: ... A219BS010635
        → A219BS010402-A219BS010401 → A219BS010403-A219BS010404
        → A219BS010469 ...
after:  ... A219BS010635
        → A219BS010402 → A219BS010404
        → A219BS010469 ...
```

동일 시작 link `A219BS010380`, 동일 waypoint `A119BS010142`로 재계산했다.

| 항목 | 수정 전 재구성 | 수정 후 오프라인 |
| --- | ---: | ---: |
| link 수 | 27 | 27 |
| Global Path points | 1,980 | 2,196 |
| 전체 비용/XY 길이 m | 1089.956913 | 1091.135266 |
| 두 교체 link 외 prefix/suffix | 동일 | 동일 |
| 모든 join gap XYZ m | 0 | 0 |

지도 352개 raw link 중 명시적 추상 차로 변경 link 98개를 제외하고 254개를 graph에
넣는다. point 추가/보간으로 만든 곡선이 아니다. 수정 후 전체 Global Path가 선택된
원본 link의 XYZ 배열을 정확히 이어 붙인 결과임을 assertion으로 확인했다.

전체 27개 link ID, 수정 전/후 모든 Global Path XYZ, 관련 7개 원본 link와 모든
point 간 XY 거리, 시각별 CSV 및 비교 그림은
`/workspace/logs/kcity_projection_validation_01_analysis/`에 저장했다.

## SpeedPlanner 영향

실제 bag의 43~57초 `speed_curvature`는 `0~6.63913e-14 1/m`로 사실상 0이고,
`CRUISE`, target 50 km/h다. 직선 입력에서는 자연스러운 결과다. 57.5875~57.6365초의
단일 planning cycle만 SHARP_CURVE로 바뀌며 이는 chord 후반의 경로 전환 부근이다.
`speed_max_curvature`는 raw triplet 최대치라 후반 preview에 spike가 들어올 수 있고,
실제 mode 판단에는 median/persistence 처리 후 대표값을 쓴다.

추가로 정상 404의 각 원본 vertex 175곳에서 새 route를 20 m/최대 2 m로 추출해
기존 SpeedPlanner로 오프라인 계산했다. 모든 입력은 유효했지만 현재 대표 곡률은
0, mode는 여전히 CRUISE였다. raw 최대 곡률은 0.000575~0.231824 1/m,
공간 median 후 최대치는 0~0.033484 1/m이며 지속성 계산 후 대표값은 0이다.
즉 정상 centerline 사용이 CURVE 판정이나 25 km/h 감속을 자동 보장하지 않는다.
이번에는 SpeedPlanner 로직/threshold를 변경하지 않는다. 새 MORAI 속도 판정과
실제 경로 추종 성공은 후속 주행에서 별도로 확인해야 한다.

## 검증과 재실행

- 추가 회귀 테스트 6개: 짧은 lazy chord보다 정상 곡선 선택, 두 metadata marker,
  lazy start 거부, 정상 sparse/인접 metadata 허용, marker 없는 일반 link 허용,
  점 수가 많아도 추상 lane-change link 제외.
- 핵심 회귀 4개는 변경 전 실패를 확인했고 변경 후 통과했다.
- ROS 환경에서 Planning 313 + Control 48 = **361 passed**.
- 별도 `colcon test` Planning: **313 tests, 0 errors, 0 failures**.
- 분리 build: Control/Planning/Bringup **3 packages finished**, exit 0.
  기존 install의 morai_ros2_msgs/localization을 underlay로 사용한다는 경고만 있었다.
- build/test/log: `/tmp/edgenode-route-geometry.CsP3v7/{build,install,log}`.
- 금지된 변경에 해당하는 지도·제어·Local Path·설정·SpeedPlanner·MapMatcher·
  Lattice/FSM 12개 파일의 작업 시작 시점 SHA256과 최종 값 일치를 확인했다.

현재 실행 중인 autonomy를 종료하고 새 터미널에서 k-city workspace를 빌드/실행한다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select edgenode_control edgenode_planning edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

다른 ROS 환경 터미널에서 주행 비활성 상태로 동일 route/lease를 복원한다.
현재 config의 기본 waypoint는 비어 있고 lease는 15초라 실행 당시 dump와 다르다.
다음은 이번 bag과 동일한 기존 조건을 재설정하는 명령이며 source config 수정은 아니다.

```bash
ros2 param set /planning_node enable_drive false
ros2 param set /planning_node route_node_ids "['A119BS010142']"
ros2 param set /planning_node drive_duration_sec 120.0
ros2 param get /planning_node route_node_ids
ros2 topic echo /planning/global_path --once --full-length \
  > /workspace/logs/kcity_route_geometry_after_global_path.txt
```

시작 위치/link가 달라지면 전체 route는 달라질 수 있다. 동일 시작 위치에서 정상
402→404 centerline을 Global/Local Path가 따르는지 먼저 RViz로 확인한다. 새 bag에는
기존 토픽과 `/planning/global_path`, `/planning/local_path_progress_m`,
`/planning/speed_curvature`, `/planning/speed_max_curvature`를 포함한다.
실제 주행 시작은 기존 방식으로 사용자가 수행한다.

오프라인 비교를 다시 계산하려면 제공된 decoded 입력을 사용한다.

```bash
PYTHONPATH=/workspace/ros2_ws/src/edgenode_planning \
  python3 /workspace/logs/kcity_projection_validation_01_analysis/analyze_geometry.py
```

원본 bag 재디코딩용 `bag_audit.py`도 같은 디렉터리에 있다(ROS/messages 환경 필요).
