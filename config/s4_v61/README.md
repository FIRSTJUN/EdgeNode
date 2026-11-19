# 검증한 S4 v61 재현 설정

이 프로필은 실제 MORAI 자격 시험의 원래 YAML을 byte 단위로 보존한 파일이다. 소스 기본 `lattice`는 유지된다. 사용자 복원 스택은 기본 `lattice`로 정지 중이며 아래 명령을 현재 스택에 중복 실행하지 않는다.

검증 소스 Planning SHA: `c3ecbc84fd667e47e7ea125ac4f5c69b021a5f64ebd5ded0af2ca14c69d19e52`

검증 Perception SHA: `e6129c13bfbc6a128c3d64c3d4d77bd89c8e242f4dc25016bda328aa57937334`

별도 재현 환경 또는 기존 스택이 종료된 상태에서:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_morai.launch.py \
  enable_drive:=false speed_kmh:=60.0 sharp_curve_speed_kmh:=30.0 \
  avoidance_speed_kmh:=10.0 static_avoidance_method:=lane_candidates \
  planning_profile_file:=/workspace/config/s4_v61/planning_10.yaml \
  lidar_tiled_minimum_area_bounds:=true lidar_native_geometry:=true \
  route_file:=/workspace/local_data/kcity_routes/kcity_demo_route.yaml
```

출발은 별도의 사용자 운행 결정이다. 이 명령은 주행 비활성으로 시작한다. 15km/h 보존 배치 재현은 `planning_15.yaml`과 `avoidance_speed_kmh:=15.0`를 함께 사용한다. 전체 2444m 미션의 회피 설정은 10km/h였으며 15km/h 전체 미션 검증으로 확대 해석하지 않는다. `perception.yaml`은 실제 검증 파라미터 덤프이며 위 두 LiDAR 옵션과 함께 사용한 launch 설정과 일치한다.

측량 도로 `/workspace/local_data/s4_final_completion_20261118/surveyed-roads-v37.json`의 SHA는 `7bead1955dc8adc6c343ed42b902234202c3fbac843e7caa6dda0b28ddfff0e0`다. 원본 지도·경로와 이 산출물은 결과 루트의 `final-v61-package`에 별도 보존한다. 원본 지도·대용량 bag·빌드 출력은 Git에 포함하지 않는다.

ROS2 Humble / Python3.10 / Ubuntu22.04의 검증 네이티브 `.so`는 위 패키지 `source-and-tools.tar.gz` 안의 원래 source 경로에 보존한다. 원본 SDK client·Python 의존성·프로토콜은 `morai-trial-runtime.tar.gz`를 `/tmp`에 풀면 복원된다. 기존 작업 트리는 초기화하지 않는다. 다른 환경에서의 빌드는 다음과 같으며, ABI·도구 체인을 바꾼 바이너리는 지문과 검증 결과를 다시 확인해야 한다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select edgenode_perception edgenode_planning edgenode_bringup
```

연결만 읽어서 확인하려면, sourced shell에서 다음 도구를 사용한다. 노드 재기동·파라미터 변경·주행 명령은 보내지 않는다.

```bash
python3 /workspace/tools/s4_verify_live_reception.py --seconds 20 --output /tmp/s4-live-reception-new.json
```

| 파일 | SHA-256 |
|---|---|
| `perception.yaml` | `f41a15e822e2fe5ca5b9317bcdd4437bfcadb386a7f7272a0363bb992ed2100f` |
| `planning_10.yaml` | `d399d09332d09dbc791f30c30368cafb2c3b0d4fe1b10b357ea16952d6e55167` |
| `planning_15.yaml` | `77d60d694c6c2c5d868b2b7f5cdc49918aa34b214269c2fa86e306e928cc4564` |
