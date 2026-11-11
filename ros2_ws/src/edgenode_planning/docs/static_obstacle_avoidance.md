# S4 단계별 정지 장애물 회피

`NORMAL → DETECT → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL`

지원 범위는 현재 시험 위치처럼 같은 방향 차선이 두 개 이상인 구간과 고속도로 형태의 긴 직선 구간이다.

Planning 한 호출에서 상태 전환은 최대 한 번이다. 각 분기는 결과를 반환하며 다음 단계의 경로 생성 함수를 호출하지 않는다.

- `NORMAL/DETECT`: 기존 LiDAR footprint로 진행 경로의 장애물을 감지한다. 위치·차체와의 거리·관측 크기를 debug에 기록하고 감속 가능한 글로벌 경로 구간만 발행한다. Lattice는 생성하지 않는다.
- `AVOID_ENTRY`: MGeo 현재 링크의 바로 왼쪽·오른쪽 링크 중 허용된 차선을 선택한다. `_generate_entry()`는 진입 경로와 진입 완료를 위한 짧은 PP 제어 지지 구간만 생성한다. 미래 복귀 공간·복귀 허가는 요구하지 않는다. 위치와 방향이 연속 확인된 뒤 다음 상태로 전환한다.
- `AVOID_HOLD`: 진입 완료 후에만 인접 MGeo 중심선 경로를 구성한다. 저속 유지 시간, 기존 장애물의 후방 통과, 주변 장애물과 차체 간격을 모두 확인한다. 복귀 경로는 생성하지 않는다.
- `RETURN`: 이 상태의 다음 호출에서 `_generate_return()`이 저장한 Global Path의 전방 합류점을 선택한다. 현재 위치·방향·곡률로 복귀 경로를 생성하고 해당 구간의 복귀 방향 허가, 차체 충돌, 곡률, PP 조향을 검사한다. 복귀 불가 시 인접 차선을 유지하며 재시도하고, 인접 경로도 위험하면 정지한다. 복귀 위치·방향 확인 후 `NORMAL`로 전환한다.

별도 도로 경계 파일·승인 해시·포함 검사는 제거했다. LiDAR 충돌, 차량 차체, 제동거리, 조향 검사는 유지한다. 기본 글로벌 경로, Pure Pursuit 및 PID 제어는 그대로 사용한다.

기본 파라미터: `avoidance_avoid_speed_kmh=3.0`, `avoidance_hold_duration_s=3.0`, `avoidance_hold_clearance_m=1.0`, `avoidance_obstacle_margin_m=0.25`. `avoidance_observation_padding_m=0.05`는 처음부터 검증할 장애물 기억 외곽에 더하는 여유이며, 그 안의 재관측은 동일한 장애물로 유지한다. 속도와 유지 시간은 시작 시 변경할 수 있다.

## MORAI 실행

기존 autonomy 노드가 종료됐는지 확인한 뒤 한 번만 실행한다. 실제 차량용 실행 설정이 아니다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_morai.launch.py speed_kmh:=3.0 hold_duration_s:=3.0
```

이 launch는 후방을 포함한 LiDAR ROI와 관측 포인트를 모두 포함하는 방향성 8점 외곽, 10 Hz Planning, 0.1 m 보수적 관측 외곽, 0.6 rad/s 조향 명령 변화율 제한을 사용한다. 경로 예측도 같은 조향 제한을 사용한다. MORAI의 실제 바퀴 각도(도 단위)를 곡률로 변환하므로 저속 yaw-rate/속도 비율을 사용하지 않는다. 기본 autonomy 설정은 기존 20 Hz·조향 출력과 PID를 유지한다. 센서 신선도 검사는 켜고, 이전 관측 증거 파일 기반 거리 승인 기능은 이 시뮬레이션 설정에서만 분리한다. MGeo의 방향별 인접 링크를 사용하며 명시적 차선 변경 금지는 계속 적용한다.

센서가 준비된 뒤 주행 비활성 상태에서 초기 래치를 재설정한다. 장애물과 인접 차선을 확인한 다음 주행을 활성화한다.

```bash
ros2 topic echo /planning/s4_sensor_state
ros2 service call /planning/reset_avoidance std_srvs/srv/Trigger '{}'
ros2 param set /planning_node enable_drive true
# 즉시 정지
ros2 param set /planning_node enable_drive false
```

실행 단계: `/planning/avoidance_stage`. 기존 `/planning/avoidance_state`는 `VALIDATING` 같은 안전 대기 상태를 유지하고, `/planning/avoidance_debug`의 `stage`는 실제 실행 단계를 보존한다. RViz: `/planning/lattice_candidate_markers`, `/planning/avoidance_selected_path`, `/planning/avoidance_return_path`. 복귀 경로 토픽은 실제 STEP 4 생성 전까지 비어 있다. 터미널은 상태·사유 변경 시 한 줄을 출력한다.

문법·단위 테스트 통과는 실제 MORAI 주행 성공이나 주행 안전성 확인을 의미하지 않는다. 실험 결과는 실행 로그로 별도 확인한다.
