# Phase A — NORMAL S자 고속 추종 완료

기준 `29eef7ed2eb04d592521367b9ca90aaa004763b0`, 브랜치 `s4-finish`, 작업 트리 `/workspace`. 새 브랜치·worktree·amend·reset·clean·push는 사용하지 않았다. **검증한 MORAI 조건에서 A 완료**다. 이 문서를 포함하는 `Phase A: qualify NORMAL-only S-curve tracking` 커밋과 결과 루트의 커밋 영수증을 함께 확인한다. B는 A 완료 커밋 뒤 별도 결과 폴더에서 시작한다. C~F와 별도 하이패스 연구는 미착수다.

결과 루트: `/workspace/local_data/s4_a_scurve_20261119`. 기계 판독 근거는 [s4_a_evidence.json](/workspace/docs/validation/s4_a_evidence.json)다. v61 원본 소스·기본 Launch·`config/s4_v61`·완료 보고서·검증 증거·지도·경로·보존 묶음은 변경하지 않았다.

## 범위와 A378 원인

A는 장애물이 없는 NORMAL 주행의 S자 추종 시험이다. 최초 v61 기준 A073 30·40·60×3은 모두 NORMAL, 회피 활성·진입·복귀 0으로 통과했다. 최초 전체 주행의 A378 정지는 FSM NORMAL_DRIVE→EMERGENCY_STOP, Planner NORMAL→BLOCKED였고, 장애물 확인 0·진입/복귀/후보 0이었다. `BRAKING_DISTANCE`는 **회피 전 NORMAL 장애물 안전 판단**이며 Lattice 실행 중 정지가 아니다.

실패 직전 속도 59.679km/h, 원본 Local Path 80m, 장애물 front station 40.874m, 차체 전방 3.812m와 0.25m 여유를 뺀 가용 거리 36.811m < 반응/제동 필요 거리 39.325m였다. 이 입력을 받아 정지한 안전 조건은 정상이다. 정지 후 속도 0과 함께 발행한 빈 Path를 일반 경로 소실로 세지 않았다. 센서는 READY였고 해당 실패에 armed HOLD는 없었다. 최초 실행 Planning·Perception·Control·Localization 설정은 v61 정상 전체 주행 성공 설정과 같았다.

현재 관측만 쓰는 가시성 속도 제한과 과거에 보존된 점유를 쓰는 충돌 판단이 서로 달랐고, 일시적 관측을 정적 점유처럼 보존하여 경로를 막았다. 정확한 관측 합집합만 사용해도 원래 실패 경로와 교차했으므로 convex fill만 원인이라고 단정하지 않았다. 기존 평면 필터의 재생에서는 지면으로 증명된 patch가 0개였으므로 지면 임계값도 완화하지 않았다. 개별 원시 반사의 시설/노면 정체나 센서 하드웨어 오류는 확정하지 않는다.

원본 근거: [a-scope-and-baseline-comparison.json](/workspace/local_data/s4_a_scurve_20261119/a-scope-and-baseline-comparison.json), [a378-first-distance-context.json](/workspace/local_data/s4_a_scurve_20261119/a378-first-distance-context.json), [a378-union-versus-convex-path-clearance.json](/workspace/local_data/s4_a_scurve_20261119/a378-union-versus-convex-path-clearance.json), [a378-local-ground-support-replay.json](/workspace/local_data/s4_a_scurve_20261119/a378-local-ground-support-replay.json).

## 독립 A 구현

`edgenode_normal_experiment`는 기존 센서/FSM/미션/전방 속도·PP·Control·Localization·Perception을 사용하고, 정적 횡경로 계획기만 NORMAL 안전 어댑터로 교체한다. 탐색 작업자는 닫으며 차선 변경·Offset 진입/복귀 메서드는 호출되면 거부한다. 원래 NORMAL 경로를 그대로 추종한다.

현재 관측은 즉시 위험 검사에 포함하고, 서로 다른 source frame 2회로 확인한 정적 점유는 가려져도 유지한다. 미확인 단발 관측은 0.6초 이상 및 앞을 가리는 현재 관측이 없는 신선한 3개 프레임 이후에만 만료한다. 결측/반복 timestamp는 만료를 진행하지 않는다. 점유 합집합의 convex 공백을 채우지 않으며 공간 색인으로 실제 도형 간 거리와 교차를 검사한다. 동일 점유를 가시성 속도 제한에도 사용한다. 최초 전차체 sweep 충돌까지 실제 주행 거리를 계산하고, 이미 정지거리가 부족하면 즉시 정지한다. Actor 정답은 Planning 입력으로 보내지 않는다.

차체 폭 2.12m·전방 3.81m·후방 1.12m, 장애물 0.25m·기준점 1.20m·padding 0.10m·센서/Control 0.6초·반응 0.3초·제동 4m/s²를 유지했다. 실제 차체 도로 포함, 금지 B2와 물리 C3 sweep을 검사하며 기존 혼합선 허용 증명도 다시 확인한다. PP/기본 속도 계획의 물리 한도와 정상 주행 알고리즘을 바꾸지 않았다.

가져온 v61 기반 SHA는 `c3ecbc84fd667e47e7ea125ac4f5c69b021a5f64ebd5ded0af2ca14c69d19e52`, 실제 A 확장 SHA는 `0f494fff2a6c0c78db257a55c56a5edf1d1cfcb4deb705427adc8923eebad735`다. `static_avoidance_method=lattice`는 보존된 노드 초기화용 값이고 실행 전 A 어댑터로 교체된다. 실제 실행은 `selected_strategy=NORMAL_SAFETY_ONLY`, `static_avoidance_method=NONE_PHASE_A`, `/planning/phase_a_runtime`로 확인한다. 기본 v61 Launch의 선택은 변경하지 않았다. 최종 A 전용 Launch의 기본 프로필 경로만 내용이 동일한 v1→v3 이름으로 정리했으며, 실제 자격 시험은 v3 경로를 명시했다.

전체 설정 차이는 [normal-only-v3-versus-v61-profile-diff.json](/workspace/local_data/s4_a_scurve_20261119/normal-only-v3-versus-v61-profile-diff.json)의 초기화 선택과 `phase_a_motion_mode` 두 항목이다. A의 추가 안전 정책 상수와 동작은 [README.md](/workspace/ros2_ws/src/edgenode_normal_experiment/README.md)에 기록했다.

## 실제 검증

최종 동일 A 코드로 S자 5회 + A378 재현 1회 + 전체 정상 미션 1회 = 주행 성공 7회다. 별도 실제 NPC 안전 정지 1회는 정상 무장애 주행 성공 수에 포함하지 않는다. 이전 기준 S자 성공과 실패/중간 버전은 합산하지 않는다.

| 접근 조건 | 실제 접근 최고 km/h | 요청−2 이상 유지 s | S 횡오차 p95 / 최대 m | 차체 B2 / C3 최소 여유 m | 출구 회복 km/h |
|---|---:|---:|---:|---:|---:|
| 30 | 29.688 | 6.679 | 0.056 / 0.080 | 0.793 / 1.188 | 30.003 |
| 40 | 39.458 | 5.041 | 0.123 / 0.153 | 0.739 / 1.194 | 39.892 |
| 60-1 | 59.011 | 2.237 | 0.151 / 0.191 | 0.762 / 1.172 | 50.864 |
| 60-2 | 59.020 | 2.239 | 0.182 / 0.214 | 0.738 / 1.155 | 49.865 |
| 60-3 | 59.041 | 2.231 | 0.211 / 0.235 | 0.712 / 1.144 | 53.092 |

기준은 60 접근에서 실제 58km/h 이상 2초, S 횡오차 p95≤0.25m·최대≤0.6m 및 조건부 출구 회복≥45km/h다. 설정은 NORMAL 60 / SHARP_CURVE 30km/h이며 S 내부 무조건 60 주행을 요구하지 않았다. 최종 5회 모두 NORMAL·정적 회피 실행 0, 독립 금지 항목 6개 모두 0이다. 구간 시험의 마지막 감독 감속은 목적지 자율 정지로 계산하지 않는다.

전체 약 2444m는 18/18 경유지, 원시 Ego 목적지 정지 5.454초, 위치 오차 최대 0.409m다. 충돌·금지 B2·물리 C3·도로 미증명·비정상 경로 소실·armed HOLD 및 목적지 전 1초 이상 불필요 정지는 전부 0이다. Planning p95 141.681ms / 최대 328.004ms, 최대 source age 0.276초다. 전체 경로 내 S자도 실제 접근 59.762km/h, 58 이상 3.301초, 횡오차 p95 0.230m·최대 0.266m, 출구 회복 52.557km/h로 통과했다.

실제 전방 NPC는 횡회피 없이 최소 차체 여유 3.068m에서 브레이크 0.65로 59.922초 정지 유지했다. 목표 약 0.0088km/h는 기존 Control의 명시적 정지 기준 ≤0.05km/h 안이다. 원본 주행 스크립트의 중단 오류는 보존하고 원시 bag의 별도 안전 정지 감사로만 통과 판정했다. 이 시험은 물리적 회피 불가능성 증명이나 Lattice 회피 시험이 아니다.

각 성공 폴더의 `summary.json`, `runtime-stage-manifest.json`, `bag/metadata.yaml`, `final-mission-audit-normal-v3.json`, `scurve-analysis.json`, `scurve-control-detail.json` 및 PNG/SVG를 확인한다. 보조 PP bag은 `-control-detail/bag`다. 횡가속도는 측정 속도/실제 앞바퀴 각도의 bicycle 추정, 종가속도는 0.25초 차분 추정이며 IMU 힘 측정과 구분한다. 실제 collider 감사는 grid convergence를 적용한 2D yaw 회전 sweep이며 완전한 3D roll/pitch 충돌 증명은 아니다.

## 실패와 도구 수정

NORMAL-only v1은 과거 점유 정지, v2는 약 495ms 계산 지연과 센서 감시 정지로 실패했다. 삭제하거나 성공으로 바꾸지 않았다. 최종 v3가 관련 재현·회귀와 전체 경로를 통과했다. 40 시험의 RPC 실패와 복원 미완료도 별도 보존했다. 기존 4초 대기 뒤 활성화 요청이 늦게 적용된 FSM 기록이 있고, 실패 bag 종료 뒤 구간은 원시 Ego로 증명하지 않는다. 후속 SDK에서는 정지 상태와 약 0.04m 위치 차이를 확인했다. 미완료 복원으로 A632 미션/실제 A380 위치가 불일치한 상태는 원래 미션과 파라미터를 응답 확인·재조회 대조하여 복구했다.

A 시험 도구만 RPC 응답을 최대 15초 기다리고 지연/응답을 기록한다. 준비 중에는 READY·신선도·지도·Path·SHA가 함께 안정되어야 한다. 실제 관측된 최장 응답 8.54초를 수용하며 차량의 0.6초 센서 기준은 바꾸지 않는다. 준비 실패를 S자 통과로 계산하지 않는다. 소스/SHA 불일치 오류의 한 경우는 실제 SHA 변경이 아니라 미완료 경로의 임시 결과에 SHA가 없었던 상황이었다.

감사: [40-arming-acknowledgement-failure-raw-audit.json](/workspace/local_data/s4_a_scurve_20261119/40-arming-acknowledgement-failure-raw-audit.json), [stale-route-after-scene-restore.json](/workspace/local_data/s4_a_scurve_20261119/stale-route-after-scene-restore.json), [incomplete-trial-restoration-repaired.json](/workspace/local_data/s4_a_scurve_20261119/incomplete-trial-restoration-repaired.json).

## 빌드·회귀·현재 정지 상태

소스를 수정하기 전에 실행 노드와 탐색 작업자를 종료했다. 실제 시험 전후의 Planning·Control·Perception·Localization·A 전체 파일 지문은 동일하다. 최종 정지 빌드는 2개 패키지 성공, 선정 회귀는 기존 559+A 9=568개 실패/오류 0이다. [a-final-build.log](/workspace/local_data/s4_a_scurve_20261119/a-final-build.log), [a-final-regression.xml](/workspace/local_data/s4_a_scurve_20261119/a-final-regression.xml), [a-final-regression-command.json](/workspace/local_data/s4_a_scurve_20261119/a-final-regression-command.json)에 명령과 결과가 있다. 전체 저장소 모든 테스트 통과라는 의미는 아니며 과거 Mock 기준선 실패 기록은 유지한다.

착수 장면과 Ego 위치를 복원했다([a-completion-scene-restoration.json](/workspace/local_data/s4_a_scurve_20261119/a-completion-scene-restoration.json)). 빌드 후 같은 A 코드를 재기동하여 주행 비활성·실제 정지·제어 발행자 1개·READY·연속 원시 센서와 새 source timestamp를 확인했다: [a-final-stopped-runtime-receipt.json](/workspace/local_data/s4_a_scurve_20261119/a-final-stopped-runtime-receipt.json). B 전환 전 다시 정지 상태에서 소유 노드만 교체한다.

## 재현 명령과 보존 자료

Dev Container에서 기존 스택을 정지하고 제어 발행자 0개를 확인한 뒤 실행한다. 기본 Launch나 v61 프로필은 덮어쓰지 않는다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select edgenode_normal_experiment edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_a_scurve.launch.py enable_drive:=false \
  planning_profile_file:=/workspace/config/s4_a/planning-normal-only-v3.yaml
```

Ubuntu 호스트의 동일 Dev Container:

```bash
cd '/home/a/프프/EdgeNode'
docker compose exec autonomous bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && ros2 launch edgenode_bringup s4_a_scurve.launch.py enable_drive:=false planning_profile_file:=/workspace/config/s4_a/planning-normal-only-v3.yaml'
```

시험 명령은 각 `summary.json`의 `execution_argv`와 시퀀스 JSON에 보존했다. 대표 60 시험의 일반형은 다음과 같다. 명령은 실제 출발·구간 정지·장면 복원을 수행한다.

```bash
cd /workspace
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/s4_stage_trial.py --stage A -- \
  --link A219BS010632 --start 5 --follow-distance-m 360 \
  --approach-speed-kmh 60 --seconds 180 --record-bag \
  --output /workspace/local_data/UNUSED_NEW_A_TRIAL_DIRECTORY
```

이전 버전으로 전환할 때는 먼저 현재 변경과 결과를 커밋/별도 보존하고 차량 정지→현재 launch 및 자식 종료→제어 발행자 0 확인 순서를 따른다. A는 추가 모듈이므로 원본 v61 소스를 되돌릴 필요가 없다. 아래 보존된 검증 프로필로 기동하고 실제 모드·지문·READY·발행자 1개를 확인한다.

```bash
ros2 launch edgenode_bringup s4_morai.launch.py \
  enable_drive:=false speed_kmh:=60.0 sharp_curve_speed_kmh:=30.0 avoidance_speed_kmh:=10.0 \
  static_avoidance_method:=lane_candidates \
  planning_profile_file:=/workspace/config/s4_v61/planning_10.yaml \
  lidar_tiled_minimum_area_bounds:=true lidar_native_geometry:=true \
  route_file:=/workspace/local_data/kcity_routes/kcity_demo_route.yaml
```

지도·원본 경로·v37 도로와 네이티브 ABI는 기존 v61 보존 묶음을 그대로 참조한다. `/workspace/local_data/s4_final_completion_20261118/final-v61-package/`의 `original-maps-and-route.tar.gz`, `source-and-tools.tar.gz`, `morai-trial-runtime.tar.gz` 및 `evidence-index.json`을 함께 보존한다. SDK runtime tar는 `/tmp` 경로를 재구성하는 자료이며 현재 임시 파일만을 유일한 사본으로 사용하지 않는다. A는 Python 3.10.12, Shapely 1.8.5.post1, NumPy 1.21.5, SciPy 1.8.0, pyproj 3.3.0에서 검증했다. A 추가 의존성 계약은 `config/s4_a/runtime-requirements.txt`다. 네이티브 ABI/구현이 달라지면 지문 재확인이 필요하다.

A의 bag·지도·빌드를 무차별 Git 추가하지 않는다. 소스·설정·도구·보고서·간결한 근거를 커밋하고 전체 외부 증거의 SHA-256 목록을 결과 폴더에 보존한다. A 완료 커밋 뒤의 B는 `/workspace/local_data/s4_b_static20_20261119/`를 사용한다.

최종 외부 보존 묶음은 `/workspace/local_data/s4_a_scurve_20261119/final-a-package/`다. `evidence-index.json`은 닫힌 bag·로그·분석·중간 실패 및 외부 v61 의존성의 파일별 SHA-256을, `commit-receipt.json`과 `phase-a-completion.bundle`은 A 완료 커밋을 보존한다. 실행 중 Launch 로그는 별도 고정 사본으로 보존하여 이후의 경고 출력과 혼동하지 않는다.
