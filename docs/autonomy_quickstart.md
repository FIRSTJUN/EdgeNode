# MORAI C-track 자율주행 실행 순서

현재 통합 코드를 기존 EdgeNode Docker/ROS2 환경에서 실행하는 방법이다. 일상적인 실행에는 재빌드가 필요 없다.

1. MORAI에서 주행을 확인했던 C-track 맵과 ERP42를 불러온다. 차량을 차선 중앙에 진행 방향으로 놓고, 시뮬레이션을 재생한다. 기존 ROS2 연결 및 외부 제어 설정을 켜고 카메라·LiDAR·차량 상태 송출을 유지한다. ROS_DOMAIN_ID는 기존 EdgeNode 컨테이너와 동일하게 맞춘다(현재 compose 기본값은 0).

2. VS Code의 EdgeNode Dev Container 터미널을 연다. 프로젝트 경로는 `/workspace`다. PC의 일반 터미널에서 컨테이너를 열 경우에는 PC의 EdgeNode 프로젝트 폴더에서 다음을 실행한다.

   ```bash
   docker compose up -d autonomous
   docker compose exec autonomous bash
   ```

3. 컨테이너 터미널에서 다음 명령을 실행한다. 최초 실행과 재실행에 같은 명령을 사용한다.

   ```bash
   cd /workspace
   bash tools/autonomy/run_autonomy.sh --restart enable_drive:=true drive_duration_sec:=60.0
   ```

   `--restart`는 기존 EdgeNode 노드가 있으면 먼저 주행을 비활성화하고 정상 종료한 뒤 인지·계획·제어를 새로 실행한다. 기존 노드가 없어도 그대로 실행된다. 스크립트가 ROS2 환경을 자동으로 불러온다. 터미널을 열린 상태로 유지한다. 60초 후 차량은 자동 정지하지만 노드는 계속 실행된다. 기본 갈림길 선택은 `straight`이며, 현재 방향과 가장 가까운 연결 경로를 선택한다. 반드시 도로 전체가 직선이라는 의미는 아니다.

4. 실행 중인 차량을 정지하려면 **같은 컨테이너의 두 번째 터미널**에서 다음을 실행한다.

   ```bash
   source /opt/ros/humble/setup.bash
   source /workspace/ros2_ws/install/setup.bash
   ros2 param set /planning_node enable_drive false
   ```

   전체 노드 종료는 처음 launch를 실행한 터미널에서 `Ctrl+C`를 누른다. 시간 만료 후에도 전체 노드는 자동 종료되지 않는다.

5. 시간 만료나 수동 정지 후 다음 주행 세션을 시작하려면 두 번째 터미널에서 다음을 실행한다. 현재 노드가 이미 실행 중인 경우에도 이 방법을 사용한다.

   ```bash
   source /opt/ros/humble/setup.bash
   source /workspace/ros2_ws/install/setup.bash
   ros2 param set /planning_node enable_drive false
   ros2 param set /planning_node drive_duration_sec 60.0
   ros2 param set /planning_node enable_drive true
   ```

   같은 설정으로 다음 세션만 시작할 때는 위 parameter 명령을 사용한다. 전체 파이프라인을 다시 실행하려면 3번의 `--restart` 명령을 사용한다. `--restart` 없이 기존 노드가 있을 때 실행하면 `Already running`으로 차단된다. 주행 세션은 0초 초과, 최대 300초다. 5분 세션을 선택하려면 최초 실행에서 `drive_duration_sec:=300.0`, 실행 중에는 `drive_duration_sec 300.0`으로 설정한다.

상태 확인은 두 번째 터미널에서 ROS2 환경을 불러온 뒤 실행한다. `Ctrl+C`로 조회를 종료한다.

```bash
ros2 topic echo /planning/state
# 상세 경로 오차·속도·차선 신뢰도
ros2 topic echo /planning/diagnostics
```

| 상태 | 의미와 확인할 내용 |
|---|---|
| `MAP_LANE_FOLLOW` | 지도 경로와 카메라를 사용해 주행 |
| `MAP_FOLLOW_CAMERA_DEGRADED` | 카메라 신뢰도가 낮아 지도 경로를 저속으로 추종 |
| `PREVIEW_STOP` | 주행 비활성 상태; enable_drive로 주행 시작 가능 |
| `DRIVE_SESSION_COMPLETE` | 설정한 시간이 끝나 자동 정지; false → true로 다음 세션 시작 |
| `WAIT_FOR_EGO` | MORAI 차량 상태 송출 확인 |
| `CAMERA_TIMEOUT` / `LIDAR_TIMEOUT` | 해당 센서 송출과 시뮬레이션 재생 상태 확인 |
| `MAP_OR_POSE_INVALID` | C-track 지도와 차량 위치·방향 확인. 다른 위치로 재배치했다면 enable_drive=false로 두어 이전 경로가 유효하지 않을 때 현재 위치에서 다시 획득하게 함 |
| `OBSTACLE_STOP` | 진행 경로에 가까운 장애물 감지 |
| `COLLISION_STOP` | 충돌 정지 유지. 장면 확인·재배치 후 launch를 재시작해야 해제됨 |

갈림길의 좌회전 또는 우회전을 우선하려면 전체 노드를 종료한 뒤 아래 중 하나로 실행한다. 해당 방향의 연결이 없으면 진행각이 가장 작은 연결을 선택한다. 경로 선택은 tracker 생성 때 결정되므로 실행 중 parameter 변경만으로 전환되지 않는다.

```bash
bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true turn_preference:=left
bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true turn_preference:=right
```

소스를 수정하거나 install 폴더가 없다면 정지·전체 노드 종료 후 한 번 빌드한다.

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select edgenode_perception edgenode_planning edgenode_control edgenode_bringup
```

ROS 로그는 `/workspace/logs/autonomy/`에 저장된다. 알고리즘과 검증 상세는 `docs/control_integration.md`에 있다. 현재 위치 추정은 MORAI EgoVehicleStatus local ENU를 사용한다.


2026-10-03 운용 수정: `--restart`로 기존 로컬 시험 launch를 비활성화·정상 종료하고 다시 실행한다. 다른 작업공간의 프로세스는 종료하지 않는다. 정지 명령은 출발 순서에 포함하지 않는다. 정지 상태에서 차량이 다른 위치로 재배치되어 이전 경로가 무효가 되면 planner가 현재 위치의 경로를 다시 획득하도록 수정했다. 주행 활성 상태에서는 경로를 임의로 다시 선택하지 않는다.
