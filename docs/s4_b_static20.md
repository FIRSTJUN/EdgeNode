# Phase B — 정적 회피 20km/h 검증 완료

검증한 유한 MORAI 조건에서 B를 완료했다. A 완료 `fc81ffbebbd10b088f0250c96bb410b0db73a96d`와 v61 `29eef7ed2eb04d592521367b9ca90aaa004763b0`을 보존하고 같은 `s4-finish`에서 개발했다. 완료 커밋의 실제 SHA는 결과 묶음 `commit-receipt.json`에 기록한다. C→F는 이 완료 커밋 이후 실제 검증을 거쳐 순서대로 진행한다. 별도 하이패스 연구는 범위 밖이다.

B 소스는 `ros2_ws/src/edgenode_static20_experiment/`, 프로필은 `config/s4_b/`, 전용 Launch는 `s4_b_static20.launch.py`다. 실제 최종 지문은 `3ffe175553c2bc6bf1dbb60f10c19ec212b7c2d541d6dbefc8fb8c1caca2a4fe`이며 모든 최종 시험의 실행 전후 패키지 파일 일치를 확인했다. v61 Planning·A 안전 모듈 지문은 [기계 판독 근거](validation/s4_b_evidence.json)에 별도로 연결한다.

## 실제 검증과 속도 판정

최종 동일 v12 소스로 정적 고정7개를 통과했다. v11의 무장애 NORMAL 통합2개는 정상 경로·Localization·NORMAL guard·노드 소스가 최종과 동일함을 파일 해시로 확인하여 회귀 근거로 재사용했다. aggregate 지문은 버전별로 구분하며 v11 정상 결과를 v12 횡회피 성공으로 세지 않는다. 이전 버전의 성공을 최종 성공 수에 합산하지 않는다. 충돌 메시지·실차체 금지 B2·물리 C3·도로 포함 미증명·비정상 Local Path 소실·armed 센서 HOLD는 각 시험에서 모두0이다. 정상 목표60/급커브30/회피 요청20이며10/15 회귀는 각각 독립 프로필을 기동했다.

| 최종 실제 시험 | 진입 / 복귀 / 자연 합류 | 중간 불필요 정지 | 최소 실제 장애물 여유 m |
|---|---:|---:|---:|
| b_v12_lane_10 | 1 / 1 / 0 | 0 | 1.671 |
| b_v12_lane_15 | 1 / 1 / 0 | 0 | 1.672 |
| b_v12_lane_20 | 1 / 1 / 0 | 0 | 1.664 |
| b_v12_natural_20_retry_02 | 1 / 0 / 1 | 0 | 1.672 |
| b_v12_offset_20 | 1 / 1 / 0 | 0 | 0.951 |
| b_v12_next_adjacent_20 | 1 / 1 / 0 | 0 | 0.673 |
| b_v12_full_static_20 | 4 / 4 / 0 | 0 | 0.367 |

| A301 요청20 구간 | 실제 속도 p05 / p50 / p95 km/h | 19 이상 최장 유지 s |
|---|---:|---:|
| AVOID_ENTRY | 5.518 / 19.262 / 19.489 | 5.472 |
| AVOID_HOLD | 19.521 / 19.562 / 19.602 | 2.164 |
| RETURN | 19.620 / 19.735 / 19.773 | 5.313 |

이는 순간 최고값만의 통과가 아니다. 진입·통과·복귀 각각19km/h 이상2초 유지를 확인했다. 좁은 Offset 및 전체 미션은 완결 증명이 가능한 낮은 속도를 허용하며 이를20 성능 통과로 세지 않는다. 각 시험의 실제 요청/선택 속도·전략 변화는 근거 JSON의 `selected_speed_timeline`, 원시 bag 및 속도 분포에 있다. 전체 저장 약2444m/18경유지+4NPC는18/18 방문, 자율 목적지 정지 5.596초를 확인했다. 감독 거리 종료 시험과 자율 목적지 시험을 구분한다.

실패했던 A439는 같은 출발/30km/h/무장애 조건에서105m를 NORMAL/회피 진입0으로 통과했다. B 모드 무장애 S60도 NORMAL만으로 통과했고 접근58 이상 2.245초,곡선 횡오차p95 0.176m를 확인했다. 두 시험은 정적 회피 성공과 분리한다.

## 바꾼 알고리즘과 보존

기존 v61/A 소스·프로필·기본 Launch·지도·저장 경로와18경유지는 변경하지 않았다. B에서만 다음 모듈을 선택한다.

- `static20_planner.py`: NORMAL은 완료된 A의 즉시 위험/확인된 점유/가시성 속도/전차체 검사를 사용한다. 실제 위험 또는 확인된 정적 점유는 원래 서로 다른 source2회 확인으로 정적 계획에 인계한다. 확인된 점유는 trigger 직전 일시 가림으로 지우지 않고 회피 완료 시 episode를 닫는다.
- `natural_join.py`: 실제 directed native join과 합류 전 필수 경유지의 순서/거리 증명이 있을 때만 자연 합류한다. 아니면 완결 복귀를 유지한다.
- `static20_planner.py`/`entry_search.py`: 같은 총12초에서 요청 속도의 native→Offset을 우선하고 하위15/10 tier도 native 완결 복귀→Offset을 검토한다. 후보별 조건부 초기 속도/예약 접근 원점을 사용하되 실제 선택 속도 이하까지 기다리고 최신 자세·장애물·지도·차체·PP 증명을 다시 확인한다. 예약0m면 제동한다. NORMAL에서 요청 상한을 복원한다.
- `search_pacing.py`: 처음2초 가정으로 예약을 조기 소진하던 계산 대기를 남은 전체 탐색 시간에 맞춘다. 같은12초 예산/반응·제동 상한/예약0m 제동을 유지한다.
- `fair_schedule.py`: 기존 직렬 callback group 안에서 정상100ms 시작 간격을 유지하고 계산 초과 때 최소20ms를 비워 서비스 기아를 막는다. 센서/Control0.6초 한도는 유지한다.
- `source_time_ekf.py`/`source_time_localization.py`: 기존 EKF 식을 재사용해 지연 GPS를0.8초 이력에서 재처리하고 현재 ROS 차량 속도를 융합한다. 위치는 GPS/IMU이며 Actor API 입력은 없다. GPS/IMU/차량 속도 각각0.6초,미래0.05초를 검사하고 실제 필터 원시 시각으로 Odometry를 발행한다. 별도 프로필 GPS std0.05m/속도 std0.1m/s다.
- `normal_clearance.py`: 기존 성공 A cache를 유지하고 실패/미생성 정상 reference만 작은 C2 보정으로 검토한다. 전체 링크 차체 sweep+15cm가 원래 B2/C3/실측 road를 통과해야 한다. 끝점·접선·곡률·노드/링크/경유지 순서를 보존한다. 곡률은 원래 최대값(기본 선호0.08/m 이상일 때)을 증가시키지 않고 차량 조향 상한 안에 둔다. 원래 정상 속도/횡가속도 제한을 유지하며 특정 링크 분기나 장애물 정답을 넣지 않는다.

A439는 원래 reference의 차체 여유 약5.5cm<허용 추종15cm가 보호 정지의 잔여 원인이었다. B 후보의 요청 보정−0.2m/실제 최대0.2354m/최대 곡률0.081924/m는 원래0.237423/m 및 차량0.160467/m보다 작다. A393/A438도 같은 일반 조건으로 선정했다. 지도 공백을 채우거나 차체/여유를 줄이지 않았다. 증명하지 못한 링크는 원래 경로와 live 안전 정지를 유지한다.

## 실패 재현과 최종 범위

모든 실패/중간 소스/원시 입력을 결과 루트에 보존했다. v2는 계산 중 예약0m 접근으로 TRACKING_ERROR 인계 거부 후 BRAKING_DISTANCE, v4/v5는 확인된 점유를 trigger 직전에 잃어 계산 대기 정지, v6는 전체 Offset의 조건부 초기 속도/시간 배분, v7은 A404 하위 native를 탐색하지 않는 선택 오류였다. v8 전체3회피 뒤 NORMAL A439는 시간 정합/차체 reference 여유 문제였다. 이를 정적 횡회피 실패나 지도 부족으로 바꾸지 않았다. source-time 보완 후에도 v9 동일 무장애 A439가 정지하여 reference 여유 원인을 추가로 확인하고 v11에서 같은 구간을 통과했다. v11 전체의 안전/자율 목적지는 통과했지만 첫 Offset 계산 대기2.396초로 성능 실패했으며, v12의 남은 episode 시간 기반 접근 속도로 같은 전체를 재검증했다. 첫 worker 전체 경과는 v11 7707.301ms/v12 7406.448ms이고 속도 probe 합은7344/7075ms다. 이를 구분한 시간표는 v11-v12-first-offset-time-accounting.json에 있다. 요청20인 전체의 실제 선택은 Offset10→차선15→Offset15→차선20이었다(v12-full-maneuver-selection-accounting.json).

A632/A404는 핀 고정한 실측 C3/A2 도로 자료를 사용한다. 실제 지도 근거 부족과 탐색 예산/선택 오류를 구분한다. 제한 시간 소진은 물리 불가능의 증명이 아니다. 실제 위험·물리 차단·센서 입력 단절·완결 경로 미증명에서 안전 정지는 유지한다. v61의 실제 센서 중단 정지와 물리 차단 ENTRY_SEARCH_BUDGET_EXHAUSTED 시험은 이전 완료 근거로 별도 보존하며 B의 주행 성공 수에 합산하지 않는다.

v11 첫 A439 준비는 observer가 새 정상 경로를 지원하지 않아 출발 전 거부했다. B 전용 verifier가 같은 핀 지도와 실제 파라미터로 전체 보정을 독립 재계산하도록 추가하고 원래 exact-path 일치 검사는 유지했다. 자연 합류 첫 준비에서도 도구가 시작 시 저장 미션 대신 대체 경로까지 보정한 범위 오류로 출발 전 거부했다. 실제 bag 경로1479점은 시작 미션 준비 범위 재계산과 정확히 일치했고 잘못 확대한 도구 경로1592점과 달랐다. 도구만 같은 시작 미션 범위로 수정해 재시험했으며 런타임 v12는 변경하지 않았다. B 추가 정상 보정은 시작 시 설정한 미션에 준비하며 새 경로의 미준비 링크는 원래 보정 및 live 안전 검사를 유지한다. Actor 정답은 배치·복원·사후 감사에만 사용한다.

상세 과정은 `/workspace/local_data/s4_b_static20_20261119/b-development-progress-v0-v12.md`, `entry-captured-inputs-v1~v12/`, 버전별 frozen tar, `qualification-b-v12-full.json`, `qualification-b-v12-sequence.json`이다. 실제 bag은 각 시험 `bag/`, 별도 Control/Localizer bag은 `*-control-detail/`에 있다. summary/runtime manifest/독립 감사/속도·조향 분석/PNG·SVG/명령을 연결했다. 실제 collider 감사는2D yaw sweep이며 완전한3D 접촉 증명과 구분한다.

## 회귀와 재현

선정 회귀는 기존559+A9+B39=607개,실패/오류0이다. 동일 소스2패키지 빌드 성공이다. 전체 저장소 모든 테스트를 통과했다는 뜻은 아니며 기존 Mock 기준선 실패 자료는 유지한다. 최종 검사 전에 소유 노드/작업자를 종료했고, 원래 장면 복원 후 같은 최종 B 프로필을 재기동해 실제 파라미터·실행 코드·원시 센서 source 시각 연속 수신·READY·주행 비활성·실제 정지·제어 발행자1개를 확인했다.

최종 정지 수신 영수증은2초 준비+6초 확인 동안 LiDAR75개(새 시각75), GPS41개(새 시각41), IMU392개(새 시각332/중복60)를 기록했고 역행0이다. 중복 IMU 시각은 새 관측으로 재사용하지 않으며 각 원시 source age 최대는0.008초 미만이었다. 최종 실제 속도 약0.000011km/h/가속0/브레이크0.65, READY/DISABLED·센서 ready/latched reason 없음·제어 발행자1개다. 시작 중 비활성 상태의 과거 HOLD 사건은 JSON에 보존하며 armed 주행 HOLD0과 구분한다.

기존 스택을 정지하고 소유 launch/자식 작업자 종료·제어 발행자0개를 확인한 뒤 Dev Container에서:

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select edgenode_static20_experiment edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_b_static20.launch.py enable_drive:=false \
  source_time_localization:=true \
  localization_profile_file:=/workspace/config/s4_b/localization_source_time.yaml \
  planning_profile_file:=/workspace/config/s4_b/planning_20.yaml
```

같은 지도/저장 미션에 차량 위치·방향이 정합하고 READY/신선도/Path/발행자1개 확인 후 별도 터미널에서:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
python3 tools/morai_drive.py status
python3 tools/morai_drive.py start --normal-kmh 60 --sharp-kmh 30
# 정지
python3 tools/morai_drive.py stop
```

10/15 회귀는 `planning_10.yaml`/`planning_15.yaml`을 명시한다. 실행 중 speed 파라미터만 바꾸어 고정 planner 구성까지 바뀌었다고 가정하지 않는다. `source_time_localization:=false` 또는 `use_v61_baseline:=true`는 원래 Localizer를 선택하지만 B 최종 프로필 재현 명령과 다르다. v61/A 원상 재현은 각각 [v61 완료](s4_static_final_completion.md), [A 완료](s4_a_scurve.md)의 정확한 전용 명령을 사용한다. 현재 변경/결과를 보존하고 정지→종료→발행자0을 확인하며 reset/clean은 필요 없다.

소스/프로필/지도 산출물/도구의 해시는 근거 JSON과 `/workspace/local_data/s4_b_static20_20261119/final-b-package/evidence-index.json`, 재현 소스 tar와 실제 완료 커밋 bundle/영수증에 있다. 원본 지도/경로/네이티브 ABI/SDK는 v61 보존 묶음의 tar·해시를 재사용한다. `/tmp`만 유일한 사본으로 두지 않았고 대용량 bag·지도·빌드를 Git에 추가하지 않았다. push는 사용자가 Ubuntu 호스트에서 수행한다.

검증하지 않은 지도·배치·미관측 영역 전체의 목적지 도달을 보장하지 않는다. 검증한 조건의 성공과 안전상 필요한 정지를 구분한다.
