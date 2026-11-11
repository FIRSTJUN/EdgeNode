> 과거 구현의 실험 기록입니다. 현재 단계형 FSM과 실행 방법은 `static_obstacle_avoidance.md`를 따릅니다. 별도 도로 경계 검사는 삭제되었습니다.

# S4 Stage 5A 합성 폐루프 검증

이 검증은 실제 MORAI 주행 승인이나 K-City 지도·센서 승인으로 사용할 수 없다.
수치, 실행별 FSM, 제동 기록, 원본 보존 사본과 Git diff는
`/workspace/logs/s4_stage5a_resume_20261109/REPORT.md`와 같은 폴더의 JSON/CSV/PNG에 저장한다.

2026-11-09 최종 행렬 44개는 모두 기대 결과를 충족했다. 3km/h 주 회피 8개와 초기
10/20km/h 감속 분석 4개는 전체 FSM 복귀·2초 NORMAL 유지·제동 정지를 확인했다.
나머지 32개는 안전 조건 부족으로 BLOCKED 후 정지했다. 모든 실행에서 모델 충돌과
도로 이탈이 없었고, 정지 뒤 1초 유지를 확인했다. 기존 783개와 신규 18개, 총 801개
테스트 및 Planning/Bringup 2개 패키지 빌드가 통과했다.

대표 직선/반경 300m 곡선의 최소 장애물 간격은 2.056/2.029m, 경계 여유는
0.492/0.464m, 최대 추종 오차는 0.0258/0.0261m였다. 실제 호스트 계산시간을 적용한
별도 freshness 게이트 재생에서는 곡선 4개가 0.6초 제한으로 거부됐다. 따라서 완료
범위는 합성 알고리즘 폐루프 검증이며 실제 시간의 ROS/MORAI 운용 검증은 남아 있다.

## 중단된 작업에서 이어진 수정

시작 브랜치는 `s4-finish`, HEAD는 `d24569262e020509dbdf6222b47297f416af5959`이다.
시작 당시 수정된 `static_obstacle_planner.py`와 미추적 `synthetic_s4_loop.py`를
`logs/s4_stage5a_resume_20261109/before/`에 복사하고 Git diff도 보존했다.
기존 차량 모델과 운영 `ControlNode.control_loop()` 연결을 확장했다.
중단된 코드의 `_rollout_distance()`와 복귀 여유 검사의 목적을 유지했다.
Preview 증가를 기다리며 전진하는 임시 로직은 실제 지도 suffix의 안전 검증으로 대체했다.

## SHORT_CONTROL_HORIZON의 원인과 수정

기준 커밋을 메모리에서만 불러와 동일한 합성 하네스에 연결하면 직선과 반경 300m 곡선
모두 RETURN 후반에 실패한다. 고정 복귀 경로의 끝이 최초 80m Preview 끝으로 정해졌고,
복귀 종료점이 그 안에 있어도 종료 전까지 Control의 최소 경로 여유는 보장하지 않았다.

요구 길이는 기존 검사 그대로
`2 * max(measured_speed, 3 km/h) / 3.6 * 2 s + lookahead + 2 * 0.30 m`이다.
3km/h에서는 약 6.433m이다. 최초 실패 시 실제 Control에 추출된 경로는 직선 6.360m,
곡선 6.382m였으며 아직 차선 취득이 끝나지 않았다. 이 실패는 충돌이나 도로 이탈을
뜻하지 않지만, 계속 추종할 검증 경로가 부족하므로 BLOCKED가 맞다.

후보 생성은 이제 복귀 종료점 뒤에도 이 요구 길이를 확보한다.
연결·차선 변경 허가가 확인된 LanePair의 실제 원래 차선 데이터가 부족하면
`RETURN_CONTROL_SUPPORT`로 후보를 거부한다. 충분한 데이터가 있고 Preview만 짧으면
`center_suffix()`로 원래 지도 선분을 추출해 복귀 경로에 붙인다. 전체 추가 경로도
차량 형상, 장애물 간격, 과거 도로 경계 검사, 곡률, 횡가속도 검사를 통과해야 한다.
지도를 외삽하거나 가상 차선을 생성하지 않는다. 커밋 이후 고정 경로는 재피팅하지 않는다.
비용 비교는 기존 공통 Preview 범위를 유지하며, 후보마다 길이가 다른 안전 여유를
추가 진행량으로 보상하지 않는다. 실제 Control 입력은 계속 20m이다.
`SHORT_CONTROL_HORIZON`, 속도·센서·거리·추종·조향 게이트와 BLOCKED 래치는 유지한다.

## 실험 A와 B

| 조건 | A: 합성 알고리즘 | B: 현재 운영 제약 |
|---|---|---|
| 관측 | 가상 near-face 50m, 전체 형상 raw 60m | ROI 상한 21.19m, 검증 증거 없음 |
| 도로 경계 | 장면에서 명시한 합성 과거 도로 경계 검사 | 승인 없음 |
| 차선 변경 | 합성 메타데이터의 명시적 양방향 허가 | 허가 없음 |
| 센서 주기/지연 | 10Hz / 0.05s, footprint와 상태 stamp/digest 일치 | 동일 메시지 인터페이스, 승인 증거는 주입하지 않음 |
| Planning/Control | 가상 계산 지연 0.02s / 10Hz, Control 30Hz | 같은 안전 모듈, 승인 부족으로 BLOCKED |
| 운영 설정 | 변경 없음 | 변경 없음 |

A의 합성 `ObservationEvidence`는 메모리 내 테스트 객체다. 운영 로더를 통과하는
센서·지도 승인 파일을 만들지 않는다. B는 evidence=None, 과거 경계 입력 없음,
차선 변경 승인 필드 없음으로 시작한다. 첫 실패는 관측 증거 없음이며 뒤 게이트를
통과했다는 의미가 아니다. A의 경계 누락/허가 누락 사례로 해당 거부 동작도 별도 검증한다.

센서 freshness, 계산 예산, STOP-before-search, fresh pose 3회, 거리 모델 및 PlanningFSM을
직접 연결하지만, 전체 `PlanningNode`의 ROS transport/drive lease를 재현하는 시험은 아니다.
ROS 노드·DDS·차량 명령 발행 없이 publisher는 메모리 sink를 사용한다.
오프라인 FSM의 ACTIVE 입력은 실제 enable_drive/drive lease와 연결되지 않는다.

## 차량 모델과 Control

운영 `ControlNode.control_loop()`, Pure Pursuit, PID 및 YAML 값을 그대로 사용한다.
Dijkstra, PP, PID 소스는 수정하지 않는다. 경로 추종 중 PID가 가속·감속 페달을 계산한다.
목표 속도 0의 운영 Control 분기는 PID를 reset하고 브레이크 0.65·조향 0을 명령한다.
BLOCKED의 빈 경로는 기존 `publish_safe_stop()`의 브레이크 0.8·조향 0을 사용한다.
정지 결과를 PID가 계산한 제동으로 해석하지 않는다.

차량은 no-slip rear-axle bicycle 모델이다. 명령 지연 0.1s, 페달 1차 지연 0.08s,
조향 1차 지연 0.04s, 조향 물리 변화율 상한 0.6rad/s, 가속 gain 3m/s²,
제동 gain 5m/s², 속도 비례 저항 0.04/s를 가정한다. 운영 Sonata 형상을 사용해
매 Control tick 사이 swept body와 장애물·도로 외곽 경계 간 거리를 확인한다.
위치나 속도를 정답으로 대입하지 않으며 음의 차량 속도만 물리 정지점 0에서 제한한다.
정지 후 1초 유지도 검증한다. 10·20km/h는 초기 속도 모델 분석이고 회피 목표는 계속 3km/h다.
10·20km/h 유지 주행/회피를 승인하는 시험이 아니다.

## 계산시간과 실제 운용 한계

가상 시간의 0.02s 계산 지연과 호스트 wall-clock 실행시간을 별도로 기록한다.
둘 중 하나라도 기존 1.5s Planning 예산을 넘으면 결과를 승인하지 않는다.
busy callback 동안 센서 갱신은 정지하며 0.6s freshness 검사도 그대로 수행한다.
1.6s 계산 고장에서는 1.5s 예산 이전/동시에 센서 stale이 먼저 거부 사유가 될 수 있다.

정상 후보 검색도 실제 호스트에서 센서 freshness 여유 0.6s를 넘을 수 있다.
`cpu_freshness_budget_exceedances`로 별도 기록한다. A의 가상 시간 성공은 이 실제
지연을 해결했다는 뜻이 아니다. 실제 ROS 실행에서는 기존 stale 게이트가 계속 STOP을
요구한다. 모델의 정상 조향/제동 지연, 무슬립, 관측 완전성, worst-case CPU,
곡선·교차로·가림과 실제 측방 관측은 MORAI/실차 보정 및 계측이 남아 있다.

## 오프라인 재현

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  MPLCONFIGDIR=/tmp/edgenode_stage5a_mpl \
  python3 tools/s4_closed_loop_report.py --out /tmp/s4_stage5a_matrix

ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q -p no:cacheprovider \
  ros2_ws/src/edgenode_perception/test ros2_ws/src/edgenode_planning/test \
  ros2_ws/src/edgenode_control/test

cd /workspace/ros2_ws
colcon build --packages-select edgenode_planning edgenode_bringup --symlink-install
```

`--match A_straight_left_3,A_curve_left_3`로 선택 실행할 수 있다.
`--resume`는 장면 값과 소스 hash가 동일한 저장 결과만 재사용한다.
각 실행 폴더에는 원본 result.json, trajectory.csv, planning.csv, fsm.json,
trajectory.png가 생긴다. summary.json/summary.csv에 성공/거부/정지 결과를 구분한다.

## 안전한 노드/launch 재시작

이전 실행은 해당 터미널에서 Ctrl+C로 중지한다. 이 검증 작업은 차량 노드를 시작하지 않는다.
별도 도메인의 disarmed 검증 launch는 Perception과 Planning만 시작한다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
ros2 launch edgenode_bringup s4_validation.launch.py use_sim_time:=false
```

개별 재시작은 통합 launch를 먼저 중지한 뒤 아래 두 명령을 서로 다른 터미널에서 실행한다.

```bash
ros2 launch edgenode_perception lidar_obstacle.launch.py
ros2 run edgenode_planning planning_node --ros-args \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/planning.yaml \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/s4_sensor_validation.yaml \
  -p enable_drive:=false

ros2 param get /planning_node enable_drive
ros2 topic echo --once /planning/s4_sensor_state
timeout 5s ros2 topic echo --once /planning/s4_distance_state
ros2 topic echo --once /planning/avoidance_debug
ros2 topic echo --once /planning/target_speed
```

실제 관측·경계·허가 승인 없이 운용 게이트는 BLOCKED를 유지한다.
LiDAR ROI, 운영 YAML, 정상 글로벌 패스와 Git HEAD는 보존한다. commit/push는 수행하지 않는다.
