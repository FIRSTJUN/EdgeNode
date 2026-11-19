# 정적 Lattice 및 K-City 전체 미션 최종 실제 검증

판정: **검증한 MORAI 조건에서 완료**. 최종 v61 생산 소스의 정적 배치 21개(기존·연속·새 무작위), 전체 저장 경로 일반 주행 2회, 장애물 포함 전체 미션 1회, 실제 프레임 생산 중단과 물리 차단 안전 정지를 통과했다. 임의의 모든 장애물·지도·차량에 대한 보장은 아니다. 후속 Phase A~F와 별도 지붕·기둥 3D 인식은 미착수다.

## 현재 사용자 복원 스택과 검증 프로필의 구분

현재 프로세스·전체 파라미터를 읽고 노드를 재기동하거나 모드를 바꾸지 않았다. 현재 `static_avoidance_method=lattice`, `selected_strategy=LEGACY`, `vehicle_geometry_profile=sonata_nominal`은 기본 launch로 복원된 설정이다. 실행 소스 지문은 v61과 같으며 LEGACY를 구버전 소스로 판정하지 않는다. 전체 3회 및 정적 캠페인의 자격은 명시적 `lane_candidates` 프로필에 적용된다.

현재 20초 연속 수신은 [live-reception-qualified.json](/workspace/local_data/s4_final_completion_20261118/user-restored-final-runtime/live-reception-qualified.json)에 보존했다. 원시 LiDAR/GPS/IMU/Ego/Localization·원자 프레임의 시각 역행 0, 최대 입력 나이 0.157초, 센서 READY 및 빈 latch를 확인했다. `enable_drive=false`, 실제 속도 약 0, accel=0, brake=0.65, READY/DISABLED/MATCHED, 제어·경로·원자 프레임 publisher 각 1개다.

현재 원자 전달 `s4_atomic_sensor_frames=true`와 0.6초 신선도 감시는 유지됐다. 현재 Perception의 native/tiled 옵션은 false이며, 검증 때는 둘 다 true다. 지도 원본 경로는 같지만 측량 도로 artifact, 병합·중심 보정과 통합 회피 옵션은 현재 기본값으로 복원되어 있다.

전체 설정 차이와 실제 실행 인자는 [qualified-profile-comparison.json](/workspace/local_data/s4_final_completion_20261118/user-restored-final-runtime/qualified-profile-comparison.json) 및 [processes.json](/workspace/local_data/s4_final_completion_20261118/user-restored-final-runtime/processes.json)에 있다. 아래 표는 차이를 모두 나열한다.

| 노드·파라미터 | 현재 사용자 기본 실행 | v61 실제 검증 |
|---|---|---|
| `planning_node.avoidance_continuous_lane_follow` | `False` | `True` |
| `planning_node.avoidance_deceleration_mps2` | `2.0` | `4.0` |
| `planning_node.avoidance_integrated_strategy` | `False` | `True` |
| `planning_node.avoidance_native_obstacle_geometry` | `False` | `True` |
| `planning_node.avoidance_native_reference_rollout` | `False` | `True` |
| `planning_node.avoidance_search_budget_sec` | `6.0` | `12.0` |
| `planning_node.avoidance_tracking_limit_m` | `0.3` | `0.15` |
| `planning_node.goal_node_id` | `A119BS010245` | `A119BS010184` |
| `planning_node.local_path_search_forward` | `10` | `200` |
| `planning_node.morai_surface_offset_experiment` | `False` | `True` |
| `planning_node.morai_transient_sensor_recovery` | `False` | `True` |
| `planning_node.morai_unmarked_junction_experiment` | `False` | `True` |
| `planning_node.normal_merge_correction_enabled` | `False` | `True` |
| `planning_node.normal_occlusion_speed_enabled` | `False` | `True` |
| `planning_node.normal_surveyed_center_correction_enabled` | `False` | `True` |
| `planning_node.static_avoidance_method` | `lattice` | `lane_candidates` |
| `planning_node.surveyed_road_file` | `` | `/workspace/local_data/s4_final_completion_20261118/surveyed-roads-v37.json` |
| `planning_node.surveyed_road_sha256` | `` | `7bead1955dc8adc6c343ed42b902234202c3fbac843e7caa6dda0b28ddfff0e0` |
| `planning_node.traffic_tracking_diagnostics_enabled` | `True` | `False` |
| `planning_node.vehicle_geometry_profile` | `sonata_nominal` | `morai_sonata_collision_envelope` |
| `lidar_obstacle_node.native_geometry` | `False` | `True` |
| `lidar_obstacle_node.tiled_minimum_area_bounds` | `False` | `True` |

현재 장면은 사용자 복원 상태를 그대로 보존했으며 Ego 이외 시험 NPC가 없고 실제 브레이크·정지를 확인했다. 사용자가 재설정한 현재 A380 위치와 과거 시험 후 원래 좌표 복원은 서로 다른 시점의 기록이다. 현재 차량을 과거 좌표로 옮기지 않았다. 과거 복원 성공은 `v61-original-scene-restoration.json`, 현재 장면은 `user-restored-final-runtime/source-and-scene.json`을 따른다.

네트워크 재연결 확인을 이유로 전체 경로나 완료된 캠페인을 추가 실행하지 않았다. 수신 확인 도구의 최초 실행은 원자 프레임 시각 필드를 루트에서 찾는 오류로 해당 항목만 거부됐다. `status.source_stamp_ns`로 도구만 정정한 뒤 20초 최종 검사를 통과했다. 생산 코드·센서 설정은 바꾸지 않았으며 최초 결과와 정정 기록도 보존했다.

## 보존·실행 지문

- 브랜치 `s4-finish`, 기준 HEAD `d449993b021bd96fd29781d7759b99a18970e4cf`. 기존 v15와 착수 시 미커밋 변경을 보존했다. reset·clean·강제 checkout은 수행하지 않았다. 2026-11-19 사용자가 최종 로컬 보존 커밋을 승인했으며 푸시는 사용자 호스트에서 수행한다. 이 문서를 포함한 보존 커밋의 SHA는 결과 루트의 `v61-commit-receipt.json`에 별도로 기록한다.
- 최종 Planning SHA-256: `c3ecbc84fd667e47e7ea125ac4f5c69b021a5f64ebd5ded0af2ca14c69d19e52`.
- Perception SHA-256: `e6129c13bfbc6a128c3d64c3d4d77bd89c8e242f4dc25016bda328aa57937334`.
- 측량 도로: `surveyed-roads-v37.json`, SHA-256 `7bead1955dc8adc6c343ed42b902234202c3fbac843e7caa6dda0b28ddfff0e0`.
- 실제 명령·동결 파일·검사 순서: [v61-qualification-sequence.json](/workspace/local_data/s4_final_completion_20261118/v61-qualification-sequence.json), [v61-final-fault-sequence.json](/workspace/local_data/s4_final_completion_20261118/v61-final-fault-sequence.json). Python·C++·실행 `.so`를 함께 지문에 포함한다.

- 최종 보존 묶음: [final-v61-package](/workspace/local_data/s4_final_completion_20261118/final-v61-package) 및 [SHA와 실제 시험 증거 인덱스](/workspace/local_data/s4_final_completion_20261118/final-v61-package/evidence-index.json). 대용량 rosbag·JSONL은 인덱스가 가리키는 원래 하위 디렉터리에 그대로 보존한다.

## 사용한 알고리즘과 기본 주행 보호

기존 차선 후보와 측량 도로 안의 완결형 Offset를 통합했다. LiDAR 관측 → 방향·차선 허가와 도로 근거 → 측정 위치·곡률에서 연속 진입 → 통과·복귀의 전체 경로와 실제 Pure Pursuit rollout 증명 → 고정 궤적 실행 → 최신 센서·측정 차체·전체 경계 재검사 → 원래 Global Path 및 NORMAL 복귀 순서다. 경유지 1m 조건을 넓히지 않고 복귀 전에 필요한 미션 노드를 함께 검증한다.

기본 선택 `lattice`는 보존했다. 최종 실제 검증과 권장 실행 프로필은 명시적인 `lane_candidates`다. Control·Localization 소스와 파라미터는 보존했으며, 원본 약 2444m Global Path 및 18개 경유지 파일은 덮어쓰지 않았다. A303 병합과 A073 작은 중심 보정은 별도의 실행 시 경로로 적용한다.

Planning 차체는 폭 2.12m·전방 3.81m·후방 1.12m다. 실제 Sonata collider 약 2.10345×4.91453m와 grid convergence를 사용해 모든 원시 Ego 표본에 측정 collider의 2D yaw 차체·회전 sweep 독립 감사를 수행했다. roll·pitch를 포함한 완전한 3D 충돌 증명과는 구분한다. 실제 차체와 계획 차체, 조향·tracking·PP 지연·도로 여유를 함께 확인한다. 장애물 0.25m·기준점 1.20m·센서 0.6초 및 모든 금지 B2/물리 C3 검사를 유지했다.

센서는 원자 상태·장애물 프레임과 원본 수신 시각을 사용한다. 동일한 점·반경 그래프·연결 성분·외곽을 네이티브 계산으로 최적화했다. 최소 면적 관측 외곽은 관측점을 보존하며 기존 bounded convex memory를 사용한다. 시험에서 제외한 union memory와 실패 프로토타입도 보존했다. Actor API는 배치·복원·사후 감사에만 사용하며 Planning 장애물 입력에 공급하지 않는다.

명목 완결 계획은 횡가속도 1.2m/s²로 여유를 두고 실제 실행 한도 1.5m/s²와 10/15km/h 목표는 유지한다. Offset는 안전한 후단 통과 복귀를 먼저 증명하며 필요할 때만 공간상 여유를 증명한 조기 복귀를 검토한다. 준비 곡선의 끝 곡률을 다음 연결부에 일치시킨다. 실행 중 검사 창은 원래 궤적의 차체 자세와 sweep을 유지하며, 최신 장애물·경계 검사 범위만 제한한다.

## 전체 경로와 목적지 정지

| 실제 시험 | 최고 km/h | 최대 제어 경로 오차 m | 원시 Ego 표본 | 경유지 | 독립 정지 유지 s | 목표 오차 m |
|---|---:|---:|---:|---:|---:|---:|
| [full_v61_static_same_placement](/workspace/local_data/s4_final_completion_20261118/full_v61_static_same_placement) | 59.765 | 0.419 | 17419 | 18/18 | 6.450 | 0.400 |
| [full_v61_qualified_normal_01](/workspace/local_data/s4_final_completion_20261118/full_v61_qualified_normal_01) | 59.814 | 0.440 | 13215 | 18/18 | 5.476 | 0.370 |
| [full_v61_qualified_normal_02](/workspace/local_data/s4_final_completion_20261118/full_v61_qualified_normal_02) | 59.814 | 0.419 | 13253 | 18/18 | 5.714 | 0.387 |

각 전체 시험에서 충돌·불필요 SENSOR HOLD·비정상 Local Path 소실·금지 B2·물리 C3·도로 미증명·목적지 전 1초 이상 정지는 모두 0이다. 허용 방향의 혼합선 통과 원본은 별도 보존했다. 유한 경로의 남은 거리로 미리 감속하고 실제 위치·방향·원시 속도 및 정지 유지로 도착을 판정한다. 표의 정지는 감독 스크립트의 정지 요청 전에 관측됐다.

장애물 포함 전체 미션의 배치는 A656 16m/yaw9°/scale0.55, A404 53.666m/yaw19.154°/lateral−0.288m/scale1.069, A632 42m, A301 50m다. 4개 회피 이후 NORMAL 주행과 자율 목적지 정지를 연결했다. 원본 정상 목표 60km/h·급커브 30km/h·회피 10km/h를 사용했다.

정식 자격 26건은 회피·전체 완주 24건(정적 21 + 전체 3)과 기대된 안전 정지 2건이다. 별도의 동일 실패 배치 재현 1건을 더해 최신 소스의 실제 실행 증거는 총 27건이며 중복 배치를 독립 무작위 성공으로 세지 않는다.

전체 주행의 센서 source age 최대는 static/normal01/normal02 각각 0.258/0.270/0.303초다. Planning p95는 102.119/82.403/82.589ms, 최대는 250.693/192.896/166.631ms다. 모든 콜백이 100ms 이내였다고 주장하지 않으며 원본 source 신선도와 주행 중 HOLD 0으로 실제 결과를 판정한다.

## 기존·연속·새 무작위 배치 회귀

| 배치 | 실제 최고 km/h | 회피·복귀·NORMAL | 독립 감사 |
|---|---:|---|---|
| [preserved_lane15_yaw12](/workspace/local_data/s4_final_completion_20261118/preserved15-v61/preserved_lane15_yaw12) | 15.141 | 통과 | 6개 금지 항목 모두 0 |
| [preserved_lane15_long_approach](/workspace/local_data/s4_final_completion_20261118/preserved15-v61/preserved_lane15_long_approach) | 15.162 | 통과 | 6개 금지 항목 모두 0 |
| [final_regression_lane10_40](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/final_regression_lane10_40) | 10.281 | 통과 | 6개 금지 항목 모두 0 |
| [native_verified_offset10_small](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/native_verified_offset10_small) | 10.326 | 통과 | 6개 금지 항목 모두 0 |
| [offset_edge_object10](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/offset_edge_object10) | 10.266 | 통과 | 6개 금지 항목 모두 0 |
| [right_lane10](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/right_lane10) | 10.315 | 통과 | 6개 금지 항목 모두 0 |
| [multi_lane10](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/multi_lane10) | 10.293 | 통과 | 6개 금지 항목 모두 0 |
| [sequential_lane10](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/sequential_lane10) | 14.945 | 통과 | 6개 금지 항목 모두 0 |
| [integrated_offset_lane10](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/integrated_offset_lane10) | 10.302 | 통과 | 6개 금지 항목 모두 0 |
| [random_20261118_05](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/random_20261118_05) | 10.294 | 통과 | 6개 금지 항목 모두 0 |
| [a632_map_evidence_stop](/workspace/local_data/s4_final_completion_20261118/regression-v61-all/a632_map_evidence_stop) | 10.298 | 통과 | 6개 금지 항목 모두 0 |
| [same_lane_two_50_110](/workspace/local_data/s4_final_completion_20261118/static-sequence-v61/same_lane_two_50_110) | 10.273 | 통과 | 6개 금지 항목 모두 0 |
| [late_second_adjacent110](/workspace/local_data/s4_final_completion_20261118/static-sequence-v61/late_second_adjacent110) | 15.119 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_01](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_01) | 10.224 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_02](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_02) | 10.291 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_03](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_03) | 10.287 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_04](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_04) | 10.291 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_05](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_05) | 10.331 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_06](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_06) | 10.250 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_07](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_07) | 10.275 | 통과 | 6개 금지 항목 모두 0 |
| [random_2026111802_08](/workspace/local_data/s4_final_completion_20261118/random-2026111802-v61/random_2026111802_08) | 10.286 | 통과 | 6개 금지 항목 모두 0 |

무작위 8개는 고정 seed `2026111802`의 위치·방향·횡위치·크기를 그대로 사용했다. 실패를 제외하거나 성공을 기준으로 다시 추출하지 않았다. 기존 seed 배치와 새 seed 배치는 구분해 보존했다. 연속 배치의 두 번째 인접 차선 장애물이 AVOID_HOLD 중 새로 관측됐다는 원본 프레임 증거는 [late_second_adjacent110의 continuous-observation-evidence-v2.json](/workspace/local_data/s4_final_completion_20261118/static-sequence-v61/late_second_adjacent110/continuous-observation-evidence-v2.json)에 있다.

동일 실패 배치 4번의 추가 재현 [random_2026111802_04](/workspace/local_data/s4_final_completion_20261118/random-reproduction-v61/random_2026111802_04)도 통과했다. 이 실행은 위 고유 21배치와 중복 계산하지 않는다.

역사적 동일 A404 배치의 실제 차체·계획·경유지 전후 그림은 [a404-waypoint-comparison-v55.png](/workspace/local_data/s4_final_completion_20261118/a404-waypoint-comparison-v55.png) 및 원본 SVG/JSON이다. v54의 3개 경유지 거리 3.520/3.500/3.462m가 v55에서 0.020/0.026/0.036m로 줄었다. 이 그림은 당시 변경의 통제된 비교이며 최신 v61의 자격은 위 현재 소스 실제 결과로 판단한다.

## 실패 원인·동일 조건 수정 전후

| 현상 | 원인·분류 | 수정과 검증 |
|---|---|---|
| 약 2.1km 센서 비상 정지 | 분리 토픽 DDS 짝짓기·Planning 콜백 지연, 이후 별도 A378 생산 지연. 센서 계층 | 원자 프레임·계산 전 수신·동치 기하 최적화. 원본 최초 HOLD 226.151809s와 source age·FSM·bag 보존. 최종 반복 전체 주행 HOLD 0 |
| A303 금지선 접촉 | 원본 경로 자체의 차체 여유 부족. 작은 실제 추종 오차로도 발생하는 일반 경로 기하 문제 | 실제 소멸 차선과 허용 방향 혼합선을 이용한 C2 병합. 동일 시작 조건 금지 sweep 115→0, 최종 전체 주행도 0 |
| 목적지 Local Path 소실 | 유한 경로 끝의 일반 미션 종료 처리 | 사전 감속·실제 정지·도착 유지. 강제 goal이나 감독 정지로 성공을 만들지 않음 |
| A404 | 실제 지도 증명 공백, 복귀 탐색 한계, 평행 차선에서 경유지 우회, global/지역 거리 기준 혼용을 구분 | C3/A2 근거 복원, 완결 복귀·원본 노드 제약·세계 좌표를 통한 station 정합. 동일 배치 및 전체 18개 노드 방문으로 확인 |
| A632 | 양쪽 근거 부족과 mask 누락은 지도 처리 오류. 이후 새 관측에 대한 0.2076m 또는 조기 복귀 0.24617m 여유는 계획 문제 | 실제 C3/A2만으로 증명 복원. 추가 관측 여유의 후보와 후단 통과 복귀 우선. 0.25m 검사는 유지하고 동일 배치 재통과 |
| 굽은 첫 진입의 반복 예산 초과 | 준비가 끝난 뒤 초기 측정 곡률을 재주입하여 약 0.304m의 불필요한 굽힘 생성. 알고리즘 연결 계약 | 준비 끝 곡률로 연결. 같은 입력 12초/907후보 실패→약 1.9초/첫 후보 전체 증명. 전체 동일 4장애물 실제 완주 |
| 무작위 4번 B2 거부 | 검사 창 절단·재샘플링으로 다른 차체 방향 생성. 고정 궤적은 안전하지만 한 절단 위상에서만 거부 | 원래 이웃 점을 사용한 같은 body sweep에 검사 범위만 적용. 최신 실제 관측·PP 재생 및 동일 배치 실제 재시험 통과 |
| 호스트 주소 변경 | DHCP .51→.149 뒤 ROS/MORAI DDS가 이전 주소 유지. 회피와 별개인 외부 실행 환경 문제 | 설정·센서 4개·장면 보존 후 ROS와 같은 MORAI 버전 재기동. 세 노드 파라미터 동일·fresh source 약 0.11초·단일 제어 복구 후 최종 시험 수행 |

역사적 성공과 실패는 원래 버전 지문에 묶어 보존한다. 잘못된 조향 부호·유한 제어 tail 밖까지 진행한 초기 오프라인 진단은 자격에서 제외하고 정정 기록을 남겼다. 계산 한도 초과를 물리 불가능으로 표시하지 않는다.

## 센서 중단과 실제 물리 차단

- [sensor_interruption_v61_actual](/workspace/local_data/s4_final_completion_20261118/sensor_interruption_v61_actual): 단일 실제 Perception 생산자를 SIGSTOP해 유효 원본 프레임 출력을 중단했다. 0.6초 검사를 유지한 채 자율 비상 정지하고 원시 정지를 감독 요청 전에 2.029초 확인했다. 주행 해제·정지 후 생산자를 재개했다. raw LiDAR 하드웨어 단선 시험과는 구분한다.
- [physical_wall_v61_a073](/workspace/local_data/s4_final_completion_20261118/physical_wall_v61_a073): 실제 NPC와 양쪽 원시 C3 사이의 독립 단면 증명이 `certified_blocked=true`다. 양쪽 남은 0.573072/0.568975m 틈은 실제 Ego 폭 2.10345m보다 작다. 실제 FSM 정지 사유는 `ENTRY_SEARCH_BUDGET_EXHAUSTED`이며 계산 예산에 따른 안전 거부다. 계획기가 물리 불가능을 직접 분류했다고 해석하지 않는다. 별도 독립 단면 증명이 차단을 입증한다. 센서 READY·자율 정지 2.081초·충돌 0·복원 확인. 전역 우회 불가능이라는 주장은 하지 않는다.
- 한쪽 경계가 없었던 과거 배치는 지도 미확인으로 남겼다. 물리 불가능 성공 사례로 바꾸지 않았다. NGII 매뉴얼 `ngii-manual.pdf`(SHA-256 `93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541`) p88/p101–104와 원본 지도 SHA를 보존했으며 Type6 중앙분리대 개구부를 물리 벽으로 계산하지 않는다.

## 검사·실행·복원

최종 선정 회귀 559개를 실행했고 실패·오류는 0이다. XML은 [broad-regression-v61.xml](/workspace/local_data/s4_final_completion_20261118/broad-regression-v61.xml)다. 이는 저장소 모든 테스트 통과라는 뜻이 아니다. 과거 Mock 기반 `test_path_following_node.py`의 기준선 실패 기록은 보존한다. 실제 자격은 소스 동결·기존 배치·새 무작위·전체 반복·독립 감사·실제 중단·물리 차단을 함께 통과해야 완료로 계산했다.

최종 검증 종료 당시에는 검증한 10km/h 회피 프로필과 원래 Actor/Ego/18노드 미션으로 복원했다. 이후 사용자가 네트워크를 복구하면서 시작한 현재 기본 스택과의 차이는 위에 구분했다. 실행 프로필은 `v38-native-c3-fast10.yaml`, 15km/h는 `v41-native-c3-15.yaml`, Perception은 `v32-perception-native.yaml`이다. 이름은 보존한 최초 작성 버전이며 최종 실제 소스는 위 지문을 따른다.

기존 스택을 종료한 별도 재현 환경에서의 실제 Launch:

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

위 명령은 검증한 설정을 재현하는 기록이다. 현재 정상 수신 중인 기본 스택에 중복 실행하지 않는다. 검증 YAML은 원래 프로필과 byte 단위로 같은 파일을 `/workspace/config/s4_v61/`에 Git 보존했다. 명시적 모드만 바꾸는 것으로는 측량 도로·차체·목표·추종·센서 옵션까지 재현되지 않는다.

실제 각 시험의 정확한 명령은 `summary.json/execution_argv`, 모든 노드 파라미터는 최종 패키지 YAML에 있다. 정지한 기존 스택에서 Planning 하나만 교체하는 명령:

```bash
python3 /workspace/local_data/s4_final_completion_20261118/reload-planning.py /workspace/local_data/s4_final_completion_20261118 replay-load-01 v38-native-c3-fast10.yaml
```

기존 네이티브 빌드는 Planning+Bringup 2개 패키지 성공([v47-native-body-build.log](/workspace/local_data/s4_final_completion_20261118/v47-native-body-build.log)), Perception 1개 성공([v32b-native-build.log](/workspace/local_data/s4_final_completion_20261118/v32b-native-build.log))이며 `setup.py`의 C++17/O3/ffp-contract=off 설정을 보존했다. 이후 Python 변경은 symlink install로 반영됐고 최종 559검사와 모든 실제 자격이 같은 지문을 사용했다. 이번 수신 확인 때문에 실행 바이너리를 재빌드하지 않았다.

소스·도구·C++·실행 `.so`, 원본 지도·경로, 보존 v15·착수 시 변경, 고정 manifest, SDK 프로토콜과 Python 의존성, 최종 파라미터·SHA·명령을 최종 패키지로 고정한다. `.so`는 Git에서 제외하고 SHA가 고정된 source archive에 보존한다. 대용량 지도·bag·빌드는 Git에 추가하지 않는다. 별도 복구 디렉터리에서 기준 HEAD에 소스 tar와 지도 tar를 복원하고, 같은 Ubuntu22.04/ROS2Humble/Python3.10 ABI에서 보존 `.so`를 사용한다. SDK runtime tar는 `/tmp`의 원래 상대 경로로 복원한다. ABI나 컴파일러를 바꾸어 재빌드하면 새 실행 지문을 확인하고 다시 자격 검증해야 한다. 기존 작업 트리 초기화용 명령은 제공하지 않는다.

범위는 검증한 K-City 지도·Sonata·MORAI·정적 배치다. 도로 근거가 없거나 실제 통과 불가능한 상황까지 100% 회피를 보장하지 않는다. 센서 원본 시각·실제 제동·모든 안전 거부는 계속 유효하다.

작은 자격 요약과 summary·감사 SHA는 [Git 보존 증거 요약](validation/s4_v61_evidence.json)에 있다. 과거 실패 원본과 수정 전후 증거는 결과 루트에 그대로 남기며 [센서·경계 후속 기록](s4_sensor_boundary_followup_20261118.md), `v61-prepared-curvature-root-cause.json`, `v61-fixed-window-live-replay.json`, `v60-fixed-window-boundary-forensics.json`을 함께 보존한다. 최종 증거 인덱스의 실제 실행 목록은 최신 소스 27건에 해당한다.

## 수정 소스 절대 경로와 SHA

실주행 소스의 동결 당시 81개 변경 목록은 [v61-source-change-inventory.json](/workspace/local_data/s4_final_completion_20261118/v61-source-change-inventory.json)에, 마감용 설정·수신 감사 도구를 포함한 최종 목록은 [v61-final-preservation-change-inventory.json](/workspace/local_data/s4_final_completion_20261118/v61-final-preservation-change-inventory.json)에 있다. 실행에 사용한 파일 집합은 최종 증거 인덱스의 `planning_files`·`perception_files`다. 보존한 미채택 실험 파일은 활성 프로필과 구분한다.

- `/workspace/.gitignore`
- `/workspace/config/s4_v61/README.md`
- `/workspace/config/s4_v61/perception.yaml`
- `/workspace/config/s4_v61/planning_10.yaml`
- `/workspace/config/s4_v61/planning_15.yaml`
- `/workspace/ros2_ws/src/edgenode_bringup/launch/s4_morai.launch.py`
- `/workspace/ros2_ws/src/edgenode_perception/config/lidar_obstacle.yaml`
- `/workspace/ros2_ws/src/edgenode_perception/edgenode_perception/_geometry_native.cpp`
- `/workspace/ros2_ws/src/edgenode_perception/edgenode_perception/_geometry_native.cpython-310-x86_64-linux-gnu.so`
- `/workspace/ros2_ws/src/edgenode_perception/edgenode_perception/lidar_obstacle_node.py`
- `/workspace/ros2_ws/src/edgenode_perception/edgenode_perception/lidar_obstacles.py`
- `/workspace/ros2_ws/src/edgenode_perception/edgenode_perception/provenance.py`
- `/workspace/ros2_ws/src/edgenode_perception/package.xml`
- `/workspace/ros2_ws/src/edgenode_perception/setup.py`
- `/workspace/ros2_ws/src/edgenode_perception/test/test_native_geometry.py`
- `/workspace/ros2_ws/src/edgenode_perception/test/test_tiled_minimum_area.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/_geometry_native.cpp`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/_geometry_native.cpython-310-x86_64-linux-gnu.so`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/avoidance_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/entry_search.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/goal_approach.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/hdmap_boundaries.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/lane_candidate_planner.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/native_obstacle_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/normal_occlusion.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/observed_union_memory.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/planning_node.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/planning_provenance.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/return_search.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/road_surface.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/route_clearance.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/search_budget.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/static_obstacle_planner.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/surveyed_road.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/surveyed_road_artifact.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/surveyed_road_patches.py`
- `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/vehicle_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/package.xml`
- `/workspace/ros2_ws/src/edgenode_planning/setup.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_atomic_sensor_transport.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_complete_return_validation.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_curved_entry_adoption.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_dense_normal_progress.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_directed_merge.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_entry_proof_origin.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_entry_search.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_fixed_maneuver_window_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_goal_approach.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_lane_candidate_bank.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_lateral_prediction_evidence.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_local_offset_proposals.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_matched_entry_prefix.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_mission_return_waypoints.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_native_body_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_native_controller_reference.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_native_obstacle_geometry.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_normal_occlusion.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_observed_union_memory.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_offset_observation_headroom.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_road_origin_pruning.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_road_surface.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_s4_performance_regressions.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_search_budget.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_static_obstacle_planner.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_surveyed_road.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_surveyed_road_artifact.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_surveyed_road_patches.py`
- `/workspace/ros2_ws/src/edgenode_planning/test/test_vehicle_geometry.py`
- `/workspace/tools/morai_drive.py`
- `/workspace/tools/s4_async_jsonl.py`
- `/workspace/tools/s4_build_holey_roads.py`
- `/workspace/tools/s4_build_partitioned_roads.py`
- `/workspace/tools/s4_build_surveyed_roads.py`
- `/workspace/tools/s4_extend_original_a2_survey.py`
- `/workspace/tools/s4_failure_forensics.py`
- `/workspace/tools/s4_final_mission_audit.py`
- `/workspace/tools/s4_morai_generalized_trial.py`
- `/workspace/tools/s4_partitioned_surveyed_roads.py`
- `/workspace/tools/s4_random_static_campaign.py`
- `/workspace/tools/s4_refine_holey_roads.py`
- `/workspace/tools/s4_refine_partitioned_roads.py`
- `/workspace/tools/s4_refine_surveyed_roads.py`
- `/workspace/tools/s4_reload_perception.py`
- `/workspace/tools/s4_static_barrier_certificate.py`
- `/workspace/tools/s4_survey_islands.py`
- `/workspace/tools/s4_verify_live_reception.py`
- `/workspace/tools/tests/test_s4_async_jsonl.py`
- `/workspace/tools/tests/test_s4_empty_path_audit.py`
- `/workspace/tools/tests/test_s4_surveyed_partitions.py`
