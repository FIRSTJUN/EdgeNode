# Sonata Pure Pursuit, Dynamic Lookahead와 조향 연속성

`ControlNode.status_cb()`는 `/ego_vehicle_status`의 `velocity.x/y/z`로부터
`sqrt(vx² + vy² + vz²) * 3.6`을 계산해 `current_speed_kmh`에 저장한다.
기존 속도 계산은 그대로 사용한다. `control_loop()`는 watchdog/입력 검증 후
`dynamic_lookahead.calculate_lookahead_distance()`를 호출해 매 제어 주기마다
다음 arc-length lookahead를 Pure Pursuit에 전달한다.

```text
v_mps = current_speed_kmh / 3.6
lookahead_m = clamp(v_mps * lookahead_preview_time_sec,
                    lookahead_min_m, lookahead_max_m)
```

`target_speed_kmh`는 이 계산에 사용하지 않는다.

| measured speed (km/h) | lookahead (m) |
| ---: | ---: |
| 0 | 3.0 |
| 20 | 3.0 |
| 25 | 3.125 |
| 30 | 3.75 |
| 40 | 5.0 |
| 50 | 6.25 |
| 60 | 7.5 |
| 80 이상 | 10.0 |

`config/control.yaml`에 다음 파라미터를 노출한다.

| parameter | 기본값 | 의미 |
| --- | ---: | --- |
| `lookahead_min_m` | 3.0 | 최소 거리 (m) |
| `lookahead_max_m` | 10.0 | 최대 거리 (m) |
| `lookahead_preview_time_sec` | 0.45 | 실제 속도에 곱하는 preview 시간 (s) |
| `lookahead_distance_m` | 2.5 | 기존 설정 호환용 시작 시 최소값 |

이전 config만 사용할 때 `lookahead_distance_m`를 `lookahead_min_m`의 선언
기본값으로 사용한다. 새 `lookahead_min_m`가 명시되어 있으면 새 값이 우선한다.
시작 후에는 새 세 파라미터를 매 주기 읽는다. 이전 파라미터의 런타임 변경은
최소값을 갱신하지 않는다.

속도는 유한한 0 이상 값이어야 하며, 설정은
`0 < lookahead_min_m <= lookahead_max_m`, `lookahead_preview_time_sec > 0`이어야 한다.
NaN/Inf/음수 속도 또는 잘못된 설정이면 기존 safe stop
(`accel=0`, `brake=0.8`, `front_steer=0`, PID reset)을 사용한다.
`EgoVehicleStatus.velocity`는 벡터이므로 음수 성분은 기존처럼 크기로 계산된다.
정상 입력의 explicit stop은 기존 `accel=0`, `brake=0.65`, `front_steer=0`,
PID reset 동작을 유지한다.

`/control/lookahead_distance` (`std_msgs/msg/Float32`, 단위 m)는
Pure Pursuit 계산이 성공한 주기에 전달한 lookahead를 발행한다.
이는 경로를 따라 전진하도록 요청한 거리이며, 차량과 target 사이 직선거리나
남은 경로 길이로 축소된 거리가 아니다. 예를 들어 50 km/h에서 경로가 1 m만
남으면 진단값은 6.25 m이고 target은 기존 알고리즘에 따라 경로 끝으로 제한된다.
watchdog/입력 오류 또는 Pure Pursuit 실패 시 새 진단 표본을 발행하지 않는다.
explicit stop 중에도 경로 계산이 성공하면 진단값은 발행된다.

Pure Pursuit의 target selection, curvature/정규화와 Sonata geometry
(`wheelbase_m=2.84`, `max_wheel_angle_rad=0.610865`)를 유지한다.
MORAI sign은 -1, 현재 YAML의 상한은 ±0.85다. 변환된 명령에
`steering_filter_time_constant_sec=0.08`과 `steering_max_rate_per_sec=1.5`를 적용한다.
필터 출력 하나만 상태로 저장하고, 스케줄러 공백의 허용 이동 시간은 100ms로 제한한다.
명시적 정지와 watchdog 정지는 필터를 즉시 우회하고 조향 상태를 0으로 초기화한다.
PID는 기존값을 유지하며 Local Path horizon은 20m다.

코드/API 호환 기본값은 최소 lookahead 2.5m, steering 상한 .70, 필터/변화율 0(비활성)이다.
일반 Bringup은 `control.yaml`을 읽어 위 검증 설정을 적용한다.
실제 적용값은 `/control_node/get_parameters` 또는 `ros2 param get`으로 확인한다.

## 자동 검증

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/edgenode_control:$PWD/src/edgenode_planning:$PYTHONPATH" \
  python3 -m pytest -q src/edgenode_control/test src/edgenode_planning/test
colcon build --symlink-install
git -C /workspace diff --check
```

속도별 계산, 상한 clamp, invalid/NaN/negative speed, 설정 유효성/호환성,
실제 status 속도와 target speed의 독립성, 진단 토픽, 경로 끝 처리,
기존/고속 target selection과 steering sign, explicit stop/PID reset을 검증한다.

초기 Dynamic Lookahead 도입 당시 기록: Control 85개 + Planning 313개 = **398개 통과**
(변경 전 361개, 신규 37개). 전체 `colcon build --symlink-install`은
**7 packages finished**로 성공했고 `git diff --check`도 통과했다.
빌드된 설치 패키지로 격리 ROS domain/토픽을 사용한 실제 ROS 전송 smoke도
기본값, legacy 최소값, 새 config 우선순위 **3/3 시나리오 통과**했다.

## 속도 PID의 반복 제동 진단

20Hz 목표속도와 30Hz 제어 주기의 조합에서 필터 없는 오차 미분은 목표값의
계단 변화와 MORAI 실제속도의 순간 변화를 증폭한다. 감속 중 큰 양의 D항이
가속으로 바뀌는 경우를 raw/filtered D항으로 구분할 수 있다.

`speed_derivative_filter_time_constant_sec`는 D항에만 적용한다. 계수는
`dt / (tau + dt)`이며 P/I와 Planning 목표값은 그대로 사용한다. 코드의 호환
기본값 0은 기존 PID 계산과 같다. 목표속도 감소와 정지 명령을 평활화하지 않는다.
현재 K-City YAML의 선택값은 `0.10`초와 `speed_pid_anti_windup_enabled=true`다.

`speed_pid_anti_windup_enabled`를 켜면 실제 페달 상한에 도달한 상태에서
같은 방향의 적분 누적을 중단한다. 반대 부호의 기존 적분은 0까지 해소할 수 있다.
상한은 기존 `max_accel_cmd`와 `max_brake_cmd`에서 가져온다. 코드 기본값은 false다.
명시적 정지와 watchdog 정지는 적분 및 미분 상태를 초기화하고 즉시 제동한다.

`/control/speed_pid_d_raw`, `/control/speed_pid_d_filtered`, `/control/speed_pid_i`는
정규화된 PID 노력 성분이며 `/control/speed_pid_effort`는 페달 clamp 전의 합이다.
이들은 브레이크 압력이나 물리적인 차량 가속도 값이 아니다.
각 시나리오에서 50 km/h status 입력 → Float32 6.25 m 진단값,
조향 부호 및 explicit stop 명령 전송을 확인했다. MORAI 주행은 수행하지 않았다.

## 현재 MORAI 검증

2026-11-11 K-City 검증은 60/40/30 정책에서 수행했다.
Control/Planning/Localization 및 기존·신규 실험 도구 회귀는 672개 통과했고,
Localization/Planning/Control/Bringup 4개 패키지를 빌드했다.
실측 구간·그래프·한계는 `/workspace/local_data/kcity_604030_20261111/report_ko.md`에 있다.
Planning 설정과 물리 감속 envelope는 [속도 프로파일 문서](../edgenode_planning/docs/kcity_speed_profile.md)를 따른다.

기존 노드가 실행 중이면 중복 실행하지 않는다. 변경 코드 반영이 필요한 노드만
`enable_drive=false`와 차량 정지를 확인한 뒤 개별 재시작한다.
아래 도구는 기존 Planning lease/gate만 사용하며 CtrlCmd를 직접 발행하지 않는다.
준비와 주행에 같은 상한을 지정하고, 준비 실패 뒤에는 주행하지 않는다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tools/kcity/live_test.py --prepare --max-speed-kmh 50
python3 tools/kcity/live_test.py --run --seconds 120 --max-speed-kmh 50
```

50km/h에서 회전교차로 검증 후 55, 60으로 단계적으로 확장한다.
`--prepare`는 주행을 시작하지 않고, 생략한 상한은 20km/h다.
종료·실패 모두 drive=false, target=0, 차량 정지와 제동을 확인한다.
정지 후 진행률이 앞에 남아 준비가 거부되면 오차 기준을 완화하지 말고
정지 상태에서 경로 진행률과 Localization 위치를 확인한다.
