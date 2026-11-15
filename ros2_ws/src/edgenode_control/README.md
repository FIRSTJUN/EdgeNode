# Sonata Dynamic Lookahead

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
| 0 | 2.5 |
| 20 | 2.5 |
| 25 | 3.125 |
| 30 | 3.75 |
| 40 | 5.0 |
| 50 | 6.25 |
| 80 이상 | 10.0 |

`config/control.yaml`에 다음 파라미터를 노출한다.

| parameter | 기본값 | 의미 |
| --- | ---: | --- |
| `lookahead_min_m` | 2.5 | 최소 거리 (m) |
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

Pure Pursuit의 target selection, curvature/정규화, Sonata geometry
(`wheelbase_m=2.84`, `max_wheel_angle_rad=0.610865`), MORAI sign/±0.70 제한,
PID와 Planning/Localization 코드는 변경하지 않았다. Local Path horizon은 20 m다.

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

구현 후 검증 결과: Control 85개 + Planning 313개 = **398개 통과**
(변경 전 361개, 신규 37개). 전체 `colcon build --symlink-install`은
**7 packages finished**로 성공했고 `git diff --check`도 통과했다.
빌드된 설치 패키지로 격리 ROS domain/토픽을 사용한 실제 ROS 전송 smoke도
기본값, legacy 최소값, 새 config 우선순위 **3/3 시나리오 통과**했다.
각 시나리오에서 50 km/h status 입력 → Float32 6.25 m 진단값,
조향 부호 및 explicit stop 명령 전송을 확인했다. MORAI 주행은 수행하지 않았다.

## MORAI 재실행

현재 실행 절차는 [수동 MORAI 매뉴얼](../../../docs/morai_manual.md)을 사용한다.
2026-11-15부터 NORMAL(직선·일반 커브) 60 / SHARP_CURVE 30 / S4 10km/h가
기본 목표이며 `cruise_speed_kmh`, `sharp_curve_speed_kmh`로 조정한다.
`max_drive_speed_kmh`와 별도 CURVE 모드는 삭제됐다. 위 검증 집계는 당시 구현의 기록이다.
기존 성공 속도 실험을 세션 시작마다 다시 반복하지 않는다.
