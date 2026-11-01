# 명시적 정지 수정과 MORAI 속도 제한 분리 검증

## 적용한 변경

대상은 `/workspace`의 k-city 브랜치다. `.worktrees/develop`는 별도 작업 경로다.

`control_node.py`의 `target_speed_kmh <= 0.05` 분기에서
`front_steer = 0.0`을 지정한다. 기존 accel 0, brake 0.65, PID reset을 유지한다.
watchdog / 입력 무효 / Pure Pursuit 실패 시의 `publish_safe_stop()`은 변경하지 않는다.
이 경우 기존처럼 accel 0, brake 0.8, front_steer 0을 발행한다.

Pure Pursuit 진단 토픽은 계속 계산 결과를 나타낸다. 따라서 명시적 정지 중
`/control/pure_pursuit_steering`이 0이 아니어도 최종 `/ctrl_cmd.front_steer`는 0이다.
양수 목표 중에서도 0.05 km/h 이하는 기존 명시적 정지 범위에 해당한다.
0.05 km/h 초과 시 기존 조향 및 종방향 PID를 유지한다.

이 변경은 정지 전환 직후의 non-zero steering을 고친다. 목표 속도가 아직
50 km/h인 경로 끝에서의 조향 급증은 별도 route-end 처리 전까지 남는다.
lookahead, 조향 상한, PID gain, Planning 설정은 변경하지 않았다.

## baseline 01에서 확인한 사실

- 목표 속도는 50 km/h였고 실제 최고 속도는 약 20.5206 km/h였다.
- 목표가 50인 제어 명령 855개 모두 accel 0.60, brake 0이었다.
- 경로가 3점 / 0.775922 m에서 2점 / 0.276460 m로 줄 때 SpeedPlanner가 INVALID가 됐다.
- bag 시작 후 49.345초에는 target 50, accel 0.60, front_steer -0.70이었다.
- 49.379초에는 target 0, accel 0, brake 0.65, front_steer -0.70이었다.
- 49.412초부터 brake 0.8, front_steer 0인 안전정지 명령이 나왔다.

새 코드의 MORAI 실주행 결과는 아직 없다. 테스트와 오프라인 재계산은
실제 차량 응답을 검증한 결과로 취급하지 않는다.

## Route end: 분석 및 권고, 미구현

| 방안 | 장점 | 한계 | 역할 |
| --- | --- | --- | --- |
| Control에서 짧은 target distance 거부 | 작은 분모에 따른 급조향을 차단할 수 있음 | 정지거리나 실제 임무 종료를 알 수 없고 일시적 path 절단과 구분하기 어려움 | 보조 입력 유효성 보호 |
| Planning에서 전역 경로 남은 거리로 접근 속도 제한 | 경로가 짧아지기 전에 감속 가능 | 실제 제동 성능·지연·정지 허용오차가 필요 | 주된 route-end 처리 |
| 실제 종료를 기존 GOAL_REACHED에 연결 | 상태와 target_speed=0을 일관되게 표현 | 신호 연결만으로는 현재 readiness 우선순위와 충돌 | Planning 접근 감속과 함께 사용 |

권고는 전역 경로 기반 접근 감속과 기존 GOAL_REACHED 연결을 함께 구현하는 것이다.
임의의 target distance threshold나 조향 clamp로 임무 종료를 대신하지 않는다.

최소 설계 범위:

1. Dijkstra 결과의 `points`로 XY 누적 길이를 경로 생성 시 한 번 계산한다.
   경로 비용 `cost`나 최대 20 m인 local path 길이를 남은 전역 거리로 사용하지 않는다.
2. 기존 `_progress_index` 주변의 전역 선분으로 차량을 투영해 남은 거리를 계산한다.
   local path 첫 점이 차량 앞에 선택될 수 있으므로 점 인덱스만으로 도착을 판정하지 않는다.
   전체 경로에서 매번 최근접점을 찾으면 교차·재방문 구간에서 다른 leg로 뛸 수 있다.
3. 실제 속도와 측정한 제동 성능으로 접근 속도 상한을 산출해 기존 목표 속도에
   추가 상한으로 적용한다. 제동 시작 거리는 `v * delay + v^2 / (2 * decel)`와
   검증된 여유 거리를 근거로 정한다. 여기서 v는 m/s이며 값은 아직 정하지 않는다.
   목표점이 퇴화하는 영역에 도달하기 전에 차량이 정지할 수 있어야 한다.
4. 최종 leg의 진행 위치, 끝점과의 거리 및 횡방향 오차, 실제 저속/정지를 확인해
   goal reached를 latch한다. 좌표상 끝점이 가까워도 아직 중간 leg면 도착이 아니다.
   경로 교체 또는 명시적 새 임무 때 latch를 초기화한다.
5. 현재 `planning_node.py`는 `goal_reached=False`를 전달한다. 현재 FSM은
   `not local_path_ready or not speed_ready`를 GOAL_REACHED보다 먼저 검사한다.
   따라서 2점 경로에서 goal 신호만 True로 연결하면 WAIT_FOR_LOCAL_PATH가 된다.
   실제로 검증된 도착을 이 일반 geometry readiness보다 먼저 처리하도록 조정해야 한다.
   emergency / INVALID_CONFIG / EXPIRED와 localization / map / global route 검사는
   계속 우선한다. 비종단 경로 오류를 goal로 처리하거나 `speed_ready=True`로 위장하지 않는다.

새 APPROACHING FSM 상태나 Dijkstra/Localization/Lattice 변경은 첫 구현에 필요하지 않다.
Planning 안의 접근 속도 상한과 완료 신호, FSM 완료 판정 순서 조정으로 범위를 제한한다.
실제 속도 입력이 추가되면 그 입력의 유효성도 검증해야 한다.

후속 route-end 검증에는 긴/짧은 경로, 끝점 부근 횡오차, 끝점을 지나친 경우,
끝점이 가까운 중간 leg, 일시적인 local path 절단, 경로 교체 및 장애물 정지와의
우선순위를 포함한다. route-end 처리 코드는 이번 변경에 포함하지 않았다.

## 설치 인터페이스와 종방향 모드

설치된 `morai_ros2_msgs/msg/CtrlCmd`에는 `longl_cmd_type`, `accel`, `brake`,
`front_steer`, `rear_steer`, `velocity`, `acceleration`이 있다.
accel/brake는 float64이며 메시지 정의 자체에는 범위나 enum 검증이 없다.
현재 Control은 longl_cmd_type=1, velocity=0, acceleration=0으로 발행하고
PID 출력을 accel [0, max_accel_cmd] 또는 brake [0, max_brake_cmd]로 나눈다.
max_accel_cmd는 제어 루프마다 읽으므로 재시작 없이 시험값 변경이 가능하다.

[MORAI 26.R1 공식 ROS 문서](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/ros-2)는
1=Throttle, 2=Velocity, 3=Acceleration, accel/brake 범위 0~1을 명시한다.
따라서 0.60/0.80/1.00은 페달 명령이며 m/s² 또는 목표 속도가 아니다.
설치 필드명과 문서 표기는 일부 다르므로 실제 wire 필드는 설치 인터페이스를 기준으로 한다.

설치된 `EgoVehicleStatus`에는 gear/control mode가 없다. `EventInfo`/`EventCmd`에는
관련 필드가 있지만 이벤트 변경 인터페이스다. 실험에서는 MORAI Vehicle Info의
실제 gear 및 control mode를 기록한다. `VehicleSpec`은 크기·wheelbase·최대 조향각 등
geometry만 제공하며 최고속도·모터·기어비는 제공하지 않는다.

## accel 0.60 → 0.80 → 필요 시 1.00 실험

이 절차는 아직 실행하지 않았다. 기본 control.yaml의 max_accel_cmd는 0.60을 유지한다.

### 공통 조건

- 수정한 Control을 빌드·재시작하고, 정지 중 front_steer=0부터 확인한다.
- 재시작하면 runtime route 설정이 YAML 초기값으로 돌아갈 수 있으므로 긴 직선 route를
  다시 선택하고 `route_node_ids`를 dump해 세 실험이 같은 경로인지 확인한다.
- 모든 실험에서 같은 긴 직선 route, 시작 pose, 초기 정지 상태, 차량 모델, gear,
  control mode, 노면 및 차량 설정을 사용한다. gear/control mode는 숫자를 추정하지 말고
  시뮬레이터에 표시되는 D 및 실제 외부 제어 모드를 확인한다.
- 목표 속도는 기존 50 km/h, PID와 lookahead/조향 상한은 고정한다.
- 세션 시간은 세 실험에서 같은 유한 값으로 고정하고 enable 전에 dump한다.
  baseline dump의 15초와 실제 ACTIVE 약 28.5초는 불일치하므로 현재 값을 다시 확인한다.
  15초 동안 50 km/h로 이동하는 거리는 약 208.3 m다. route에는 가속 구간과 관측 시간,
  측정한 정지거리 및 경로 끝을 피할 여유를 확보한다.
- 경로 끝 이전에 정지한다. 이번 수정만으로 target 50 상태의 끝점 급조향이 해결되지는 않았다.
- 속도가 계속 증가하는 중 종료되면 최고속도 제한으로 판정하지 않는다. 관측 시간을
  늘려야 한다면 모든 비교 실험에서 같은 시간/경로 조건으로 다시 측정한다.
- `/ctrl_cmd` publisher가 하나인지 확인한다. 새로운 명령 publisher를 추가하지 않는다.

### 각 실험 준비: 예시는 0.60

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 param set /planning_node enable_drive false
ros2 param set /control_node max_accel_cmd 0.60
ros2 param get /control_node max_accel_cmd
ros2 param get /planning_node drive_duration_sec
ros2 topic info /ctrl_cmd --verbose
ros2 param dump /control_node > /workspace/logs/kcity_accel_060_control.yaml
ros2 param dump /planning_node > /workspace/logs/kcity_accel_060_planning.yaml
ros2 bag record -o /workspace/logs/kcity_accel_060 \
  /ctrl_cmd /ego_vehicle_status /localization/odometry \
  /planning/target_speed /planning/recommended_speed /planning/speed_mode \
  /planning/drive_status /planning/fsm_state /planning/local_path \
  /planning/local_path_index /planning/lattice_offset \
  /control/path_error /control/target_left /control/target_forward \
  /control/pure_pursuit_steering /parameter_events
```

다른 터미널에서, 준비 상태 및 정지 출력 확인 후 시작한다.

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 param set /planning_node enable_drive true
```

같은 터미널에서 경로 끝 이전에 정지한다.

```bash
ros2 param set /planning_node enable_drive false
```

완전 정지와 정지 명령을 기록한 뒤 녹화를 종료한다. 시뮬레이터 시작 pose 및 정지
상태를 동일하게 복원하고 0.80 실험을 수행한다. 위 준비 명령의 accel만 0.80으로,
산출물 이름을 `080`으로 바꾼다. 필요 시 같은 방식으로 1.00 / `100`을 수행한다.
한 실험 중에는 상한을 바꾸지 않는다. 종료 후 기본값 0.60을 복원한다.

### 비교할 값

| 비교 항목 | 기록과 판정 |
| --- | --- |
| 유효 비교 구간 | target=50, drive ACTIVE, FSM NORMAL_DRIVE, 경로가 충분한 구간만 비교 |
| 명령 전달 | `/ctrl_cmd.longl_cmd_type=1`, accel이 해당 상한에 실제 도달했는지, brake=0인지 |
| simulator 수신 | Ego status accel/brake가 명령에 대응하는지, 응답 지연이 달라졌는지 |
| 가속 응답 | 0→10/20/30/40/50 km/h 도달 시간, 같은 속도 구간의 dv/dt 및 Ego acceleration |
| 최종 속도 | 순간 최대값과 평형 구간 평균·변동을 분리하고 마지막 구간 dv/dt 확인 |
| saturation | 각 accel 상한에 걸린 시간 비율과 지속시간, 속도 오차 |
| 실험 종료 원인 | enable 해제 / lease EXPIRED / 경로 INVALID / watchdog을 분리 |
| 정지 수정 | target<=0.05일 때 최종 cmd accel=0, front_steer=0, 유효 입력이면 brake=0.65 |

실제 속도는 기존 Control과 동일하게 velocity 벡터 크기 × 3.6으로 계산하고
Ego 위치 변화로 교차 확인한다. bag 시작 대신 target=50 상승 시점을 가속 시간 0으로 맞춘다.
50 km/h 목표 실험으로 확인하는 것은 50 부근 도달 가능성이며 그 이상의 지속 최고속도는 아니다.

### 결과별 다음 조사

- 상한을 높이자 평형 속도도 상승: throttle 상한/차량 부하 응답이 속도에 영향을 준다.
  이 결과만으로 PID gain을 바꾸지 않는다.
- 가속만 빨라지고 평형은 계속 약 20.5: 차량 모델의 최고속도/내부 limiter/구동계 상한을
  우선 조사한다. 이것은 가설이며 모델 설정을 확인해야 확정할 수 있다.
- cmd는 0.8/1.0인데 Ego accel이 0.6 등에 머묾: 외부 제어 모드, 명령 전달/변환,
  다른 입력의 개입 또는 simulator 입력 clamp를 확인한다.
- cmd와 Ego accel 모두 증가했는데 속도는 동일: 선택 차량의 정확한 모델/버전,
  최대속도·모터 토크/회전수·기어비·질량/저항, D 및 실제 기어 상태, 적용 Dynamics preset,
  시나리오/차량의 속도 제한을 MORAI 설정에서 확인한다.
- 명령 accel 자체가 상승하지 않음: runtime max_accel_cmd, target_speed, PID 출력과
  brake 개입부터 확인한다. 비교 중 PID gain은 변경하지 않는다.

공식 문서의 [Vehicle Info 안내](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/basic-tutorial)는
Control Mode, Current Speed, Accel, Brake, Gear 표시를 설명한다.
해당 UI/차량 설정과 로그를 함께 확인해야 simulator-side 원인을 분리할 수 있다.

## 빌드와 autonomy 재실행

기존 autonomy 터미널을 종료하고, develop 환경을 source하지 않은 새 터미널에서 실행한다.
현재 YAML의 enable_drive=false를 유지하므로 launch만으로 주행을 시작하지 않는다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select edgenode_control edgenode_planning edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
python3 -m pytest -q src/edgenode_control/test
ros2 launch edgenode_bringup autonomy.launch.py
```

Control의 기존 colcon 테스트 설정은 테스트를 자동 수집하지 않아 pytest를 직접 실행한다.
