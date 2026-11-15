# MORAI 수동 실행·주행·정지 매뉴얼

2026-11-15 KST. 브랜치 `s4-finish`. MORAI 전용 개발 절차다.
MORAI는 Ubuntu 호스트, ROS 노드는 VS Code Dev Container, RViz는 호스트에서 실행한다.
아래 호스트 저장소 경로는 현재 컨테이너 mountinfo로 확인했다.

현재 목표는 **NORMAL 60 / SHARP_CURVE 30 / S4 10km/h**다.
NORMAL은 직선과 일반 커브를 모두 포함하며 `cruise_speed_kmh` 한 값으로 설정한다.
60/30은 조정 가능한 목표다. 별도 승인 속도·최대 운행 속도 파라미터는 없다.
60/30 실제 주행 성공을 검증한 상태는 아니다. 현재 새 장애물 3번은 진입 구간의
경계 근거가 부족해 실제 접근 시 정지했다. 상세 근거와 다음 위치는
[s4_progress_20261115.md](s4_progress_20261115.md)를 참고한다.

## 1. Ubuntu 호스트 터미널 H1 — Docker와 VS Code

MORAI K-City와 기존 ROS2 통신 설정만 켜져 있다고 가정한다.

```bash
cd '/home/a/프프/EdgeNode'
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
export ROS_DOMAIN_ID=0
docker compose up -d --build autonomous
docker compose ps autonomous
```

이미 같은 Compose 서비스가 켜져 있으면 같은 컨테이너를 사용한다.
VS Code에 Dev Containers 확장이 설치된 상태에서 연결한다.

```bash
container_id="$(docker compose ps -q autonomous)"
test -n "$container_id"
container_authority="$(printf '%s' "$container_id" | od -An -tx1 | tr -d ' \n')"
code --folder-uri "vscode-remote://attached-container+${container_authority}/workspace"
```

VS Code 하단이 Dev Container 연결 상태이고 탐색기의 루트가 `/workspace`인지 확인한다.
CLI 연결이 지원되지 않는 VS Code 구성에서는 `code .` 실행 후
`Dev Containers: Reopen in Container`를 선택한다. 프로젝트의 `.devcontainer/devcontainer.json`은
Compose `autonomous`, 사용자 `dev`, 작업 디렉터리 `/workspace`를 지정한다.

## 2. Dev Container 터미널 D1 — 빌드와 주행 스택

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --symlink-install --packages-up-to edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
```

노드가 이미 있으면 중복으로 실행하지 않는다. 다음 블록은 기존 노드가 있으면
목록만 보여 주고, 없으면 Localization/LiDAR/Planning/Control 네 노드를 실행한다.

```bash
if ros2 node list | rg '^/(planning_node|control_node|localization_node|lidar_obstacle_node)$'; then
  echo '기존 주행 스택이 있습니다. D3에서 상태를 확인하세요.'
else
  ros2 launch edgenode_bringup s4_morai.launch.py enable_drive:=false \
    speed_kmh:=60.0 sharp_curve_speed_kmh:=30.0 avoidance_speed_kmh:=10.0 \
    route_file:=/workspace/local_data/kcity_routes/kcity_demo_route.yaml
fi
```

이 터미널은 실행 중 유지한다. LiDAR ROI 100m는 관측 범위 설정이며,
100m의 장애물 탐지 성공을 의미하지 않는다. 센서 유효성·충돌·경로 검사와 수동 정지는 유지된다.

## 3. Dev Container 터미널 D2 — Route Editor와 Planning Visualization

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if ros2 node list | rg '^/(route_editor_node|planning_visualization_node)$'; then
  echo '기존 편집·시각화 노드가 있습니다. 중복 실행하지 않습니다.'
else
  ros2 launch edgenode_planning route_editor.launch.py use_rviz:=false
fi
```

이 Launch가 **두 노드 모두** 실행한다. 각각을 추가로 `ros2 run` 하지 않는다.
둘 중 하나만 있는 비정상 구성은 해당 Launch를 종료한 뒤 이 블록으로 함께 다시 시작한다.

## 4. Ubuntu 호스트 터미널 H2 — 기존 K-City RViz 설정

```bash
cd '/home/a/프프/EdgeNode'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d "$PWD/ros2_ws/src/edgenode_planning/rviz/kcity_route_editor.rviz"
```

호스트에 ROS Humble/RViz가 설치된 기존 환경을 사용한다. 컨테이너의 install overlay는
`/workspace` 절대 경로를 사용하므로 호스트에서 source하지 않는다.
Fixed Frame은 `map`이다. 기존 RViz가 열려 있으면 그 창에서 위 설정을 연다.

## 5. Dev Container 터미널 D3 — 연결과 저장 경로 확인

이후 운영 명령은 모두 같은 D3에서 실행한다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node list
ros2 topic info /ctrl_cmd --verbose
python3 tools/morai_drive.py status
```

네 주행 노드와 `/route_editor_node`, `/planning_visualization_node`, MORAI 노드가
보여야 한다. `/ctrl_cmd` 발행자는 정확히 하나다. 상태 출력에서 각 `ages_s`가
신선해야 하며 센서 `ready=true`, 지도 매칭 `MATCHED`를 확인한다.
`EMERGENCY_STOP`이면 `/planning/avoidance_debug`의 이유를 먼저 확인한다.

```bash
ls -l /workspace/local_data/kcity_routes/kcity_demo_route.yaml
cat /workspace/local_data/kcity_routes/kcity_demo_route.yaml
ros2 param get /planning_node route_node_ids
ros2 param get /planning_node route_start_link_id
```

Launch는 저장된 18-waypoint 경로를 읽는다. 이미 이동한 위치에서 같은 경로를
이어갈 때는 정지 상태에서 아래 도구로 검증된 저장 경로의 나머지 부분을 적용한다.

```bash
python3 tools/s4_morai_prepare.py --saved-start-link A219BS010380
```

RViz에 편집용 저장 경로도 표시하려면 다음 명령을 사용한다.

```bash
ros2 param set /route_editor_node route_name kcity_demo_route
ros2 service call /route_editor/load std_srvs/srv/Trigger '{}'
```

`load`는 편집용 Preview를 복원한다. 현재 경로를 명시적으로 다시 적용할 때만
다음 명령을 실행하고 `success: true`를 확인한다. 주행 중에는 적용하지 않는다.

```bash
ros2 service call /route_editor/activate std_srvs/srv/Trigger '{}'
```

RViz에서 새 경로를 편집한 경우 다른 `route_name`으로 저장해 기존 파일을 보존한다.

```bash
ros2 param set /route_editor_node route_name my_kcity_route
ros2 service call /route_editor/save std_srvs/srv/Trigger '{}'
```

저장 위치는 `/workspace/local_data/kcity_routes/my_kcity_route.yaml`이다.
`morai_drive.py start/run`은 기본 저장 미션을 검증하는 도구다. 다른 미션은 정지 상태에서
`/route_editor/activate`로 적용하고 준비 상태를 확인한 뒤 아래 직접 시작 명령을 사용한다.

## 6. Dev Container 터미널 D3 — 목표 속도 설정

```bash
ros2 param set /planning_node cruise_speed_kmh 60.0
ros2 param set /planning_node sharp_curve_speed_kmh 30.0
ros2 param get /planning_node cruise_speed_kmh
ros2 param get /planning_node sharp_curve_speed_kmh
```

향후 70/40은 이 두 값만 바꾼다. `max_drive_speed_kmh`, `curve_speed_kmh`,
`morai_validated_speed_cap_kmh`는 삭제됐으므로 설정하지 않는다.
S4 속도는 기하 검증과 함께 시작 시 고정된다. 변경할 때는 정지 후 D1 Launch를
종료하고 `avoidance_speed_kmh:=원하는값`으로 다시 실행한다.
20km/h **접근** 시험은 NORMAL 목표 20, S4 목표 10인 별도 시나리오다.

## 7. Dev Container 터미널 D3 — 주행 시작

기본 저장 미션은 다음 명령이 정지·센서·경로·중복 제어 발행자를 검사하고 시작한다.

```bash
python3 tools/morai_drive.py start
```

센서 정상 복구 후 정지 latch를 해제할 필요가 있을 때만 사용한다.

```bash
ros2 service call /planning/reset_avoidance std_srvs/srv/Trigger '{}'
```

다른 미션을 직접 적용했다면 `READY`, 신선한 센서와 유효한 local path를 확인하고 시작한다.

```bash
ros2 param set /planning_node drive_duration_sec 3600.0
ros2 param set /planning_node enable_drive true
```

기본 미션의 한정된 새 S4 실험을 기록하며 실행할 때는 다음 도구를 사용할 수 있다.
예시는 승인된 별도 접근 시나리오의 20→10 목표다. 같은 성공 위치를 반복하지 않는다.
현재 3번 장애물 배치는 이 명령에서도 회피가 거부되므로 검증 가능한 배치에서 사용한다.

```bash
python3 tools/morai_drive.py run --normal-kmh 20 --sharp-kmh 30 \
  --seconds 90 --expect-return --output logs/my_new_s4_trial
```

`run`은 오류·시간 종료·Ctrl+C 후 주행을 해제하고 실제 정지를 확인한다.
종료 후 임시 시험 속도와 session 시간을 원래 값으로 복원한다.

## 8. Dev Container 터미널 D3 — 주행 중 상태·속도

```bash
python3 tools/morai_drive.py watch --seconds 600
```

`/ego_vehicle_status`의 실제 속도(`speed_kmh`), 목표 속도, FSM, 회피 debug와
차체 경계 거리, 경로 오차가 출력된다. watch의 Ctrl+C는 감시만 끝낸다.
RViz의 Global/Local/선택/복귀 경로와 함께 확인한다.

## 9. Dev Container 터미널 D3 — 정지와 실제 정지 확인

감시 중이면 Ctrl+C로 프롬프트를 되찾은 뒤 실행한다.

```bash
python3 tools/morai_drive.py stop
```

이 명령은 `enable_drive=false` 후 **신선한 실제 속도 <0.2km/h, 목표 0,
가속 0, 브레이크 ≥0.6이 0.5초 지속**돼야 `stop_confirmed: true`를 출력한다.
서비스 실패나 확인 시간 초과는 성공으로 표시하지 않는다. 직접 정지하는 동일 인터페이스:

```bash
ros2 param set /planning_node enable_drive false
python3 tools/morai_drive.py status
ros2 param get /planning_node enable_drive
```

2026-11-15 실제 MORAI에서 시작 목표 발행·차량 이동과 정지 명령을 확인했다.
새 장애물 접근 최고 14.45km/h, 경로 거부 후 실제 정지와 브레이크를 확인했다.
이는 20km/h 도달 또는 S4 회피·복귀 성공 기록이 아니다.

## 10. 정상 종료 — D3 → D1 → D2 → H2 → H1

1. **D3**: `python3 tools/morai_drive.py stop`의 정지 확인 후 진행한다.
2. **D1**: Ctrl+C로 네 주행 노드를 정상 종료한다.
3. **D2**: Ctrl+C로 Route Editor/Planning Visualization을 종료한다.
4. **H2**: RViz를 닫거나 Ctrl+C로 종료한다.
5. **H1**: 아래 명령으로 해당 Compose 서비스를 종료한다.

```bash
cd '/home/a/프프/EdgeNode'
docker compose stop autonomous
docker compose ps -a autonomous
```

`docker compose down -v`, 프로세스 전체 kill, 다른 컨테이너 종료는 필요하지 않다.
호스트 Docker·VS Code 재접속 명령은 저장소 설정과 mount 경로를 대조했으며,
이번 세션에서 호스트의 Docker를 다시 내렸다 올리는 시험은 수행하지 않았다.
