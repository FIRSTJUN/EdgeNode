# Local Path segment progress

## 최신 MORAI 기록에서 확인한 문제

대상 bag은 `logs/kcity_resample_validation_01`, 길이는 54.405초다.
아래 시각은 bag 수신 시각 기준이며 노드 callback의 정확한 실행 시각은 아니다.

raw index 528과 529의 좌표는 다음과 같고 두 점 사이 XY 길이는 85.282419 m다.

```text
528: (106.82295860990416, 1274.235997798387)
529: (136.69395864993567, 1354.1159978010692)
```

기존 Local Path는 index 528에서 시작하는 20 m / 11점으로 고정됐다.
index는 39.775767~54.376213초, 관측상 최소 14.600445초 동안 528이었다.
차량이 raw 528보다 22.16 m 전방에 있어도 다음 raw 점까지는 약 63 m가 남아
최근접 vertex는 계속 528이었다. 출력 점 수를 늘려도 이 시작점은 이동하지 않았다.

| 시각(초) | 기록된 동작 |
| --- | --- |
| 39.783 | 이미 index 528로 넘어갔지만 차량은 그 시작점보다 약 10.61 m 뒤. PP target_forward 약 13.105 m |
| 44.750 | 고정 Local Path의 남은 길이 약 2.384 m, PP가 끝점을 목표로 사용하기 시작 |
| 45.117 | target_forward 0.297303 m, target_left +0.025510 m, PP +1.0, front_steer -0.70 |
| 45.150 | 목표점 직선거리 약 0.128746 m, front_steer -0.70 유지 |
| 45.169749 | 최초 기록 odometry에서 고정 horizon 끝을 통과. 시작점 기준 투영 거리 20.068871 m |
| 45.183 | 모든 Local Path 점이 차량 뒤. Control은 accel 0 / brake 0.8 / steering 0 안전정지 |
| 54.383 | 차량 투영 거리 약 22.163 m, 경로 끝보다 약 2.18 m 전방. yaw 약 115.13도 |

Planning의 ACTIVE / NORMAL_DRIVE / CRUISE / target_speed=50은 녹화 끝까지 유지됐다.
이번 정지는 Planning INVALID가 아니라 Pure Pursuit 입력이 차량 뒤에 놓여 생긴
Control 안전정지와 일치한다. PP 진단 토픽은 45.150초 이후 발행되지 않으므로
이후의 마지막 진단값을 새로운 steering 명령으로 해석하면 안 된다.

정확한 순서는 고정된 horizon → 남은 lookahead 감소 → 좌조향 포화 → horizon 추월
→ PP 거부 및 안전정지다. 추월 당시 yaw와 경로 방향의 차이는 약 5.6도였고,
정지 후 약 45.6도로 커졌다. 이 bag에는 map_match_heading_diff 자체가 없으므로
차이는 localization yaw와 기록된 polyline으로 계산했다. MapMatcher threshold는 유지한다.

## 수정 알고리즘

1. global polyline의 원본 raw segment와 XY 누적 arc를 사용한다. Global Path와
   Dijkstra geometry는 수정하지 않는다.
2. 초기화/경로 교체 때만 전체 경로에서 projection을 찾는다. 이후에는 이전 raw
   segment index 주변 `search_back=3`, `search_forward=10`의 inclusive window만 검색한다.
3. 차량 위치를 각 유효 선분에 투영하되 `s_new >= s_previous`를 강제한다.
   같은 긴 segment 안에서도 fraction과 arc가 계속 증가하며 pose 후퇴 잡음은 progress를
   되돌리지 않는다. 이때 현재 pose보다 조금 앞의 이전 projection이 유지될 수 있다.
4. 다음 segment의 연결점까지 남은 arc가 이전 projection에서 현재 pose까지의 거리
   범위 안에 있을 때만 handoff 후보가 된다. 현재 segment와 가깝게 되돌아오는
   인접 평행 구간으로 갑자기 건너뛰는 것도 막는다. 여러 segment를 한 번에
   건너뛰려면 차량의 투영이 각 생략 segment의 끝을 통과해야 한다.
   따라서 아직 끝내지 않은 segment 대신 가까운 미래 교차 leg를 고르는 것을 막는다.
   거리 동률은 작은 arc를 우선하고, 같은 꼭짓점/같은 arc는 window 안의 다음 segment를 우선한다.
5. 선택된 projection부터 원본 polyline을 따라 20 m를 추출하고 최대 2 m로 나눈다.
   원본 코너와 더 촘촘한 MGeo 점은 유지한다. XYZ 선형 보간과 zero-length 처리도 유지한다.
6. route/waypoint/goal 교체 때 index와 arc를 함께 reset한다. 일시적 map-match loss에는
   기존처럼 progress를 보존하며 매 cycle 전체 경로로 재탐색하지 않는다.

검증한 초기화 범위는 위치만으로 선택한 가장 가까운 polyline이다. 정확히 겹치는
여러 route leg에서의 최초 시작은 위치만으로 유일하게 판별할 수 없으며 초기 진행
선택 이후의 lock과 구분한다. 운행 중에는 위 window와 segment handoff 제약을 사용한다.

## Progress 표현 및 진단

`LocalPathPlanner.extract()`는 다음을 반환한다.

- `progress_segment_index`: 원본 global segment 시작 index
- `progress_fraction`: 해당 segment의 0~1 투영 비율
- `progress_arc_m`: global route 첫 점부터 투영점까지의 XY 누적 거리
- `nearest_index`: 호환용 raw segment 시작 index, 실제 route 끝에서는 마지막 raw vertex index
- `nearest_distance_m`: 현재 차량에서 제한된 projection까지의 거리

Planning은 `_progress_index`와 `_progress_arc_m`를 다음 cycle에 함께 전달한다.
보간점의 번호는 progress나 search window에 사용하지 않는다.

기존 `/planning/local_path_index`는 raw segment 시작 index를 표시한다.
긴 segment에서는 이 값이 계속 528이어도 정상이다. 중요한 것은 fraction / arc 및
Local Path 시작점이 이동하는지다. 새 진단 `/planning/local_path_progress_m`
(`std_msgs/Float64`)은 누적 arc를 20 Hz로 발행한다. 경로 입력이 무효인 cycle에는
NaN을 발행하고, 내부 progress의 보존/reset은 위 규칙에 따른다.

실제 남은 route가 20 m보다 짧으면 있는 부분만 반환한다. 마지막 1점이나 실제
극단적으로 짧은 경로를 외삽하거나 반복 점으로 채우지 않는다. SpeedPlanner의 기존
입력 검증과 정지 gate는 그대로이며 실제 goal handling은 별도 문제다.

## 기록 pose로 수행한 오프라인 비교

동일한 고정 global route를 지도와 기록으로 재구성해 1,087개 Planning 입력 및
1,631개 Control 시각의 pose를 재계산했다. 수정된 차량 궤적을 시뮬레이션한 결과는 아니다.

- 새 arc는 전체 재계산에서 후퇴하지 않았다.
- 새 Local Path 전체가 차량 뒤에 놓인 Control 시각: 0개.
- Pure Pursuit 무효: 기존 경로 277개 → 새 경로 0개.
- 새 PP 목표점 직선거리 최솟값: 약 2.496 m.
- 45.116619초의 같은 pose에서 기존 PP는 +1.0 / front -0.70이었다.
  새 경로로 계산하면 PP 약 -0.06519 / front +0.06519, 목표점 거리는 약 2.50035 m다.

이미 이탈한 후반 pose에서도 새 경로의 전방 목표점이 계산되는 것은 확인했지만,
실제 복귀 안정성이나 수정 후 MORAI 주행 성공을 의미하지 않는다. preview 시작점이
이동하므로 같은 SpeedPlanner threshold에서도 mode는 달라질 수 있다.

## 테스트 범위

40 m 및 실제 85 m sparse segment 중간 진행, projection 시작점과 20 m horizon,
fraction/arc 단조성, 정상 0.5 m 구간, window 밖 교차점, window 안의 가까운 미래 교차점,
인접한 평행 복귀 segment,
급커브 인접 segment handoff, route reset, map-match loss 후 progress 보존,
실제 짧은 route end, SpeedPlanner 유효 입력, Pure Pursuit 전방 목표점 및 기존 Control
정지/watchdog 회귀를 검증한다. 실제 Pure Pursuit를 사용하는 통합 테스트를 위해
Planning에 Control test dependency를 명시했다.

## MORAI 재실행 및 기록

기존 autonomy를 종료하고 develop 환경을 source하지 않은 새 터미널에서 실행한다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select edgenode_control edgenode_planning edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

주행 비활성 상태에서 동일 route_node_ids를 다시 설정/확인한 뒤 기록한다.
기존 제어·Planning 토픽에 `/planning/local_path_progress_m`를 추가한다.
index 528 구간에서 arc가 증가하며 path 첫 점이 차량 근처를 따라 이동하는지,
target_forward와 목표점 거리가 끝점 접근 때문에 0으로 붕괴하지 않는지 확인한다.

`local_path_resample_spacing_m=2.0`, horizon 20 m, window 3/10 및 모든 제어값·속도
threshold·Map Matching·Lattice/FSM은 유지한다. 새 코드의 실제 MORAI 결과는 아직 없다.
