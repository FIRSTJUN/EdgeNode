# EdgeNode control 브랜치: 한성이 코드 통합

2026-10-03, 로컬 `/workspace`와 읽기 전용 `/reference/ProgrammingProject/ros2_ws/src`만 비교했다. 원격 저장소를 조회하지 않았으며 참조 mount를 수정하지 않았다. 기존 Dockerfile, host network/IPC, ROS_DOMAIN_ID, Fast DDS 설정은 유지했다. compose.yaml에 이미 있던 참조 mount 변경도 유지했다.

| 영역 | 기존 EdgeNode | 한성이 참조 | 통합 결과 |
|---|---|---|---|
| 인지 | 영상 디코딩만 수행; 별도 OpenCV prototype | 밝기/gradient mask, 다중 paint 후보, 2차 곡선, 단일 차선 방향/폭 기억 | 참조 perception_node와 lane_tracker 사용. 기존 두 OpenCV 구현은 주석으로 보존 |
| 인지 ROI | 차선 알고리즘 미연결 | top=0.55, lookahead=0.60 | 실제 640×480 영상에서 차선이 y≈300 전에 화면 옆으로 나가므로 top=0.40, lookahead=0.48로 변경 |
| 장애물 | 미연결 | 넓은 ROI의 DBSCAN, centroid 거리; 잘못된 cloud도 clear | ground z≈-0.56 관측에 맞춘 z 필터, 예측 주행 곡선을 따르는 폭 2.1m corridor, 가장 가까운 표면 거리, invalid cloud STOP |
| 계획 | WAIT_FOR_MAP, 항상 속도 0 | 차선 신뢰도 hysteresis + FSM + lane-loss grace | 참조 입력 adapter/FSM의 STOP 우선순위와 hysteresis 사용. 실제 연결된 MGeo 링크에 pure pursuit 적용 |
| 제어 | steering PID + speed PID | 거의 동일하며 종료 시 브레이크 개선 | 기존 MORAI 메시지/속도 PID 유지. 조향 filter/rate limit, 개별 명령 watchdog, finite 값 검증, 종료 브레이크, 적분 anti-windup, 감속 목표 변경 시 적분 초기화 추가 |
| 실행 | 계획·제어만 launch | 인지·계획만 launch | 인지·계획·제어 전체 launch; 기본값은 정지 preview |

## 실제 주행 기준

- 로컬 지도: `/workspace/local_data/c_track_mgeo/link_set.json`.
- 위치·heading: 모라이 `/ego_vehicle_status`의 **시뮬레이터 local ENU ground truth**. GPS/IMU EKF는 아직 연결하지 않았다. 실차용 위치 추정 검증으로 해석하면 안 된다.
- 참조 지도 origin과 collision_data의 global offset은 동일하며 첫 위치의 지도 오차는 약 0.038m였다.
- 처음 경로를 잡은 뒤에는 연결된 outgoing link로만 진행한다. 교차로에서 전체 지도의 nearest link를 계속 바꾸지 않는다.
- 기본 갈림길 선택은 현재 진행 방향과 가장 가까운 `straight`. 목적지가 지정된 mission planner는 아니다.
- `turn_preference:=left` / `right`는 선택 가능한 갈림길에서 그 방향을 우선하며, 없는 경우 진행각이 가장 작은 연결을 선택한다. 시작 링크는 차량 위치와 방향으로 결정한다.
- 지도 경로가 유효하면 차선이 안 보이는 교차로도 현재 geometry를 따라 저속 진행한다. 오래된 영상 조향을 유지해서 교차로를 통과하지 않는다.
- 좋은 카메라 pair(confidence ≥0.65)는 직선에서 최대 0.06의 작은 보정만 더한다. 단일/불확실한 paint 후보는 지도 경로를 바꾸지 않는다.
- cruise 6km/h, curve 3.5km/h, 급곡선 2.5km/h, camera degraded 2km/h. 횡오차 >0.5m 또는 heading 오차 >0.4rad는 1km/h로 감속한다.
- 카메라/LiDAR/ego 또는 각 제어 명령이 0.6초 이상 오래되면 정지. camera confidence=0과 camera topic timeout은 구분한다.
- 장애물이 3.5m 이내면 참조 FSM의 STOP. 속도에 따른 제동 거리도 검사한다. 인접 차로의 여유를 검증할 정보가 없으므로 임의 회피/차선 변경은 하지 않는다.
- 지도 횡오차 1.5m 초과, heading 불일치, 경로 종료, invalid 센서 입력에서 정지한다. 실제 collision 메시지는 STOP을 latch하며 scene 확인 후 planner 재시작이 필요하다.
- **조향 부호:** planner는 좌측을 양수로 계산하고, 실측된 MORAI ERP42 `/ctrl_cmd.front_steer`는 우측이 양수다. control의 `steering_sign: -1.0`에서 변환한다. 첫 시험에서 반대 부호를 확인했고 watchdog 정지 및 사용자 재배치 후 수정했다.

## 실행

기존 컨테이너에서:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select edgenode_perception edgenode_planning edgenode_control edgenode_bringup
source install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

기본 launch는 `enable_drive=false`이며 정지 상태로 센서/지도 결과를 확인한다. 차선 중앙의 적절한 시작 자세에서 주행하려면:

```bash
ros2 launch edgenode_bringup autonomy.launch.py enable_drive:=true turn_preference:=straight
```

이미 실행 중이면 중복 launch를 만들지 말고 다음을 사용한다:

```bash
ros2 param set /planning_node enable_drive true
# 정지
ros2 param set /planning_node enable_drive false
```

저장된 실행 래퍼도 사용할 수 있다. 중복 controller/perception publisher가 있으면 시작을 거절한다.

```bash
bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true
```

갈림길 선택은 tracker 생성 시 읽으므로 `turn_preference` 변경은 정지 후 launch 재시작한다. 같은 topic을 발행하는 legacy perception / rosbag / 두 번째 controller를 동시에 실행하지 않는다. 기존 `lane_opencv_pkg lane_detector` 진입점은 비활성 안내를 출력하며 원본은 해당 파일의 주석에 보존돼 있다.

```bash
ros2 topic echo /planning/diagnostics
ros2 topic echo /planning/state
ros2 topic echo /ctrl_cmd
```

기존 `/planning/target_error`, `/planning/target_speed`에 `/planning/target_steering`과 `/planning/curvature`를 추가했다. control은 세 target stream을 모두 요구한다. `/perception/obstacle`은 기존 5-element 배열을 유지하지만 distance는 centroid 대신 가장 가까운 표면 거리다. exact clear sentinel은 `[nan,nan,nan,inf,0]`, invalid cloud는 `[nan]*5`다.

## 검증 및 한계

참조 회귀 테스트에 실제 MGeo 지도의 직진/좌회전/우회전 bicycle 폐루프, 연결성·이탈 검사, LiDAR row padding/endian/ground/벽 검사, 제어 watchdog·NaN/Inf·조향 부호·브레이크 검증을 추가했다. 테스트는 actuator를 발행하지 않는다.

```bash
python3 -m pytest -q src/edgenode_perception/test src/edgenode_control/test src/edgenode_planning/test
```

실제 센서 캡처와 관측 로그는 `/workspace/logs/control_integration/`에 있다(Git 제외). `preview/report.json`, 첫 부호 확인 `drive_first/report.json`, 수정 후 `drive_corrected/report.json`, 연속 주행 `drive_extended/report.json`을 구분한다. 첫 잘못된 부호의 시험에서는 횡오차 watchdog이 정지시켰고 충돌은 없었다. 사용자가 차량을 재배치한 후 수정된 부호로 주행했다. 모델 테스트와 실제 주행 결과는 별개이며 모든 C-track 코스·모든 시작 자세·모든 장애물 조합의 검증을 의미하지 않는다.


추가 실제 정지 검증은 `watchdog_live.json`에 기록했다. 차를 정지시킨 상태에서 perception을 3초 멈추면 `CAMERA_TIMEOUT`, accel=0/brake=0.75가 관측됐다. planning을 3초 멈추면 command watchdog에서 accel=0/brake=0.80이 관측됐다. 각 프로세스는 즉시 복원했다.


최종 구성은 `drive_duration_sec`로 주행 세션을 제한한다(기본 60초, 최대 300초). 시간 만료 시 `DRIVE_SESSION_COMPLETE`, target_speed=0으로 정지한다. 다음 세션은 `enable_drive false` → `enable_drive true` 순서로 재설정한다. 중간의 센서 끊김·충돌·경로 이탈 정지는 시간 제한과 독립적이다.

```bash
bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true drive_duration_sec:=60.0
```


통합 후 주요 실제 관측은 45+120+100=265초, 1,294개 샘플이었다. 관측된 collision_object는 0건, 지도 기준 |횡오차| 최대 0.336m/평균 0.083m였다. 관측 위치를 이어 계산한 거리는 약 309m다(관측 구간 사이 공백이 있어 정밀 주행거리계가 아니다). 직선·좌측 교차로·좌우로 휘는 곡선·오르막을 관측했다. 모든 갈림길의 좌/우 90도 회전 실증을 완료한 것은 아니다. 모델 폐루프에서는 straight/left/right를 모두 검증했다. 최종 단위/회귀 테스트는 **125 passed**.

최종 가속 제어는 saturation 시 적분을 더 쌓지 않고, 목표 속도 감소 시 기존 적분을 초기화한다. 목표 속도를 0.5km/h 이상 초과하면 남은 적분이 가속을 계속하지 못하게 제동을 적용한다. 오르막 시험에서 상한 0.25가 부족했기 때문에 기존 상한 0.60을 유지한다.


최종 PID/주행시간 제한 수정 뒤에는 35초 주행 및 40초 관측을 추가했다(`drive_final/report.json`). 연결된 곡선 및 다음 링크를 통과하고 시간 만료 시 실제 accel=0/brake=0.75, 속도≈0으로 정지했다. 이 추가 관측에서도 collision 0건이었다. 총 주행 세션은 300초이며, 현재는 `enable_drive=false`, 다음 주행 세션은 60초로 설정돼 있다. 인지·계획·제어 노드는 실행 중이다. `/workspace/logs/control_integration/autonomy.pid`에 현재 launch PID를 기록했다. ROS 로그는 `/workspace/logs/autonomy/`에 있다.

```bash
# 현재 파이프라인에서 다음 60초 주행 세션 시작
ros2 param set /planning_node enable_drive true
# 즉시 정지
ros2 param set /planning_node enable_drive false
```

경로 경계를 3m로 넓힌 복귀 주행과 무기한 background drive 요청은 자동 승인 검토에서 거절돼 실행하지 않았다. 차량 재배치 후 기존 1.5m 경계를 유지했고 최종 주행에 자동 정지 시간을 적용했다.


재실행 무반응 수정: 기존 시험 노드가 유지한 경로와 재배치된 차량 위치가 달라 MAP_OR_POSE_INVALID가 발생했다. 주행 비활성 상태에서만 기존 경로가 무효인 경우 재획득하며, 활성 주행에서는 기존 경로 고정을 유지한다. 재배치 회귀 테스트 2개를 추가했다.


재실행 명령은 `bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true drive_duration_sec:=60.0`으로 통일한다. 기존 로컬 planner를 disable하고 종료한 뒤 새로운 pipeline을 실행한다. 이 작업공간의 정확한 launch/node 경로만 종료 대상이며, planner 비활성화에 실패하거나 기존 프로세스가 종료되지 않으면 재실행을 중단한다.
