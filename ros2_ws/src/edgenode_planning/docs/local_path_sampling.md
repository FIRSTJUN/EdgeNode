# Local Path resampling 단계 검증 기록

이 문서는 resampling만 적용했던 단계의 분석 기록이다. 현재 progress 구현과
최신 재검증 절차는 [Local Path segment progress](local_path_progress.md)를 참조한다.
현재는 raw vertex 시작점 대신 제한된 segment window의 단조 projection부터 경로를 만든다.

## 확인한 원인

MGeo 링크 `A219BS010402-A219BS010401`은 두 원본 점만 가진다.

```text
A = (99.29995859932387, 1253.3509978000075)
B = (106.82295860990416, 1274.235997798387)
XY 길이 = 22.198620544788422 m
```

기존 LocalPathPlanner는 원본 점을 추가하다 누적 길이가 20 m 이상이면 멈췄다.
A에서 시작하면 A/B 두 점으로 끝나며 SpeedPlanner의 최소 3점 검사를 통과하지 못한다.
Planning은 GLOBAL_PATH_READY여도 speed_ready=false여서 WAIT_FOR_LOCAL_PATH가 된다.
local_path_length_m을 늘리는 방식은 더 긴 sparse segment에서 같은 문제가 반복된다.

`logs/kcity_accel_060`의 parameter dump는 이번 route_node_ids와 일치한다.
bag 시작 후 58.520594초에 index 527, 두 점 / 22.198621 m가 발행됐고,
58.521219초에 speed_mode=INVALID가 됐다. drive_status는 먼저 NOT_READY였다.
사용자가 이후 disable한 상태와 구분한다.

지도와 최초 local path / index를 대조해 재구성한 전역 경로는 시작 링크
`A219BS010380`, 목적지 `A119BS010142`, 총 1,980점이다. index 527부터 끝까지
XY 경로 길이는 약 827.814854 m다. 이 사례는 실제 route end가 아니다.
이는 기록·지도 기반 오프라인 재구성이며 bag에는 global_path 자체가 없다.

## 구현

`local_path_resample_spacing_m: 2.0`을 추가했다. 이는 최대 출력 점 간격이다.

1. 최근접 점과 previous_index / search_back / search_forward는 기존과 동일하게
   원본 global_points에서 선택한다. nearest_index의 의미는 바뀌지 않는다.
2. 선택된 원본 점부터 XY 누적 20 m에서 polyline을 정확히 자른다.
   원본 선분이 horizon을 넘으면 그 선분 위에 끝점을 보간한다.
3. 각 선분을 최대 2 m 간격으로 나눈다. 선분별 등분이므로 점 간격은 모두 같지는 않다.
   원본 꼭짓점과 기존의 더 촘촘한 점은 유지한다. 코너를 가로지르는 전역 등간격
   재샘플링이나 spline smoothing을 사용하지 않는다.
4. XYZ는 선형 보간한다. 연속 중복 XY / zero-length 선분은 건너뛰고 첫 Z를 유지한다.
5. 반환 path_length_m은 실제 출력 XY polyline 길이다. 입력 배열과 global path,
   Dijkstra의 geometry는 수정하지 않는다.

Planning은 이 출력을 기존 Lattice에 넘긴다. 그 이후 발행되는 local path와
SpeedPlanner 입력은 기존처럼 동일한 active points다. Pure Pursuit는 발행 경로를 사용한다.
Lattice/FSM/SpeedPlanner/PurePursuit 알고리즘은 변경하지 않았다.

## 2 m를 기본 상한으로 정한 근거

실제 지도 25,341개 선분의 간격 중앙값은 약 0.5 m이고, 98개 sparse 선분은 모두
20 m보다 길었다. 나머지 25,243개는 0.51 m 이하라서 기본 2 m 상한으로도
기존 점을 그대로 유지한다. 20 m 경로에는 최소 11점, 10 m speed preview에는
최소 6점이 확보된다. Pure Pursuit는 기존처럼 선분 위에서 목표점을 보간한다.

점 수를 무조건 늘리는 것이 항상 속도 판정에 유리하지는 않다. 기존 테스트의
반경 20 m, 원본 점 간격 약 2 m 곡선을 0.5 / 1 m로 더 나누면 직선 보간점들의
0 곡률이 현재 거리 median / 지속성 판정에 영향을 줘 CURVE에서 CRUISE로 바뀐다.
2 m 상한에서는 해당 곡선과 기존 급커브 판정이 유지됐다. 간격 값을 더 줄일 때는
속도 판정도 별도로 재검증해야 한다. 속도 threshold는 변경하지 않았다.

## route end 및 보장 조건

- 충분한 남은 길이가 있으면 20 m에서 자르고 각 출력 간격은 최대 2 m다.
- 중복 제거 후 실제 XY 길이가 2 m보다 길면 기본 간격에서 최소 3점이 생긴다.
  기존에 촘촘한 점이 있으면 그보다 짧아도 3점 이상일 수 있다.
- 실제 남은 경로가 horizon보다 짧으면 그 끝에서 멈추며 연장하거나 반복 점으로
  3점을 맞추지 않는다. 0.1 m 두 점이나 마지막 한 점은 그대로 짧은 경로이고
  SpeedPlanner INVALID / 기존 정지 gate가 유지된다. 실제 goal handling은 별도 문제다.
- 사용자 설정 시 충분한 SpeedPlanner preview에도 최소 3점이 들어가야 한다.
  기본 20 m horizon / 10 m preview / 2 m 상한은 이 조건을 만족한다.
- 비정상 spacing과 10,000점을 초과하는 출력 요청은 None으로 거부한다.
- 원본 최근접 vertex가 바뀔 때 local path의 시작점이 이동하는 기존 특성은 유지한다.
  보간점을 progress index로 사용하거나 별도 전역 경로 재탐색을 하지 않는다.

## 정지 직전 좌조향 분석

새 bag에서 정지 직전 55~58.6초 구간의 최저 front_steer는 58.405862초의
약 -0.136844다. 그때 target_left=+0.201814 m, target_forward=2.492017 m,
path_error=0.044679 m, target_speed=25 km/h였다. 이전 baseline 01의
목표점 거리 0.26 m / steering -0.70 포화와 다른 양상이다.

보고된 sparse 선분 직전 polyline heading은 약 60.459도, 해당 선분 heading은
약 70.190도로 왼쪽 방향 전환 약 9.731도가 존재한다. 보간은 이 꺾임을 보존한다.
58.539761초에는 target 0, accel 0, brake 0.65, front_steer 0이 기록돼
이전 명시적 정지 조향 수정의 동작과 일치한다.

같은 기록 pose / 경로로 수정 전후 Pure Pursuit를 2,009회 비교했을 때
정규화 조향 차이는 최대 약 1.11e-13, 목표 XY 차이는 약 2.32e-13 m였다.
2~3점이라는 개수 자체보다 원본 polyline 방향과 pose가 이 조향을 결정한다.
이번 보간 수정으로 좌조향이 줄어들었다고 판단하지 않는다.

## 오프라인 검증과 실주행 구분

- 기록된 Local Path 1,338개: 수정 전 INVALID 170개, 수정 후 0개.
- 문제 2점 입력: 11점 / 20 m, SpeedPlanner CRUISE, preview 6점으로 재계산.
- 기록 pose 3,295개에서 원본 global nearest_index 및 nearest_distance 차이 0.
- Global Path는 고정 geometry이며 보간은 Local Path 출력에만 적용한다.

입력 점 배치가 바뀌므로 기존 speed threshold가 같아도 mode가 달라질 수 있다.
이번 기록의 57.023~58.471초를 오프라인 재계산하면 SHARP_CURVE 5개와 CURVE
25개가 CRUISE로 바뀌었다. 추가로 기존 INVALID 170개도 CRUISE가 됐다.
이것은 수정 후 실제 주행 결과가 아니다. 곡률 추정과 공간 필터의 입력이 바뀐 결과이므로
같은 구간에서 mode / target_speed / 실제 속도 / 경로 추종을 다시 확인해야 한다.

## MORAI 재검증

동일 route와 시작 조건에서 `/planning/local_path`, local_path_index, speed_mode,
speed_curvature, recommended_speed, target_speed, drive_status, fsm_state,
`/control/target_left`, target_forward, pure_pursuit_steering, path_error,
`/ctrl_cmd`, `/ego_vehicle_status`, `/localization/odometry`를 기록한다.

index 527 근처에서 2점 대신 보간된 경로가 나오고 길이가 약 20 m인지,
progress index가 원본 기준으로 계속 진행하는지, INVALID / WAIT_FOR_LOCAL_PATH가
재발하는지 확인한다. 실제 route end의 정지 동작은 별도 항목으로 기록한다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select edgenode_planning edgenode_control edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

기존 autonomy를 종료한 뒤 develop 환경을 source하지 않은 새 터미널에서 실행한다.
재시작 후 route_node_ids를 다시 확인한다. YAML의 주행 비활성 기본값은 유지한다.
