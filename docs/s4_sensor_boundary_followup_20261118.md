# S4 센서 지연·금지 경계 후속 검증 — 2026-11-18 KST

**정적 Lattice와 전체 미션의 최종 완료 판정은 보류한다. A~F는 미착수다.**
센서 입력 전달·반복 계산·시험 종료 정지를 수정했고 동일 실패 구간에서 실제 검증했다.
원래 Global Path의 차체 경계 여유, 목적지 정지, A404/A632 증명 공백과 실패 탐색 지연은 별도 미해결 항목이다.

## 기준선과 변경 범위

실제 착수 HEAD는 `d449993b021bd96fd29781d7759b99a18970e4cf` (`s4-finish`)이며 작업 트리는 깨끗했다.
사용자가 언급한 `363c4c9` 다음에 이미 v15 보존 커밋이 있었다. 기존 자료·실패·성공 코드·거부된 v12~v14 자료를 초기화하지 않았다.
이번 변경은 미커밋이다. 커밋·푸시·reset·clean을 수행하지 않았다.

결과 루트: [`local_data/s4_followup_20261118`](../local_data/s4_followup_20261118/).
`preserved-v15-source.tar.gz`, `initial-manifest.json`, `initial.diff`가 수정 전 소스·해시·Git 상태다.
현재 최종 Planning 구현 지문은 `57d9922c4e400fb376c7d5f1da5341b26d7f48244241ea89dd03090eb76f3bbf`다.
각 시험의 `summary.json`, `runtime-planning-source.tar.gz`는 **실제로 실행한 버전**을 보존한다. 이전 버전의 시험을 모두 최종 지문의 시험으로 간주하지 않는다.

- `planning_node.py`: 기존 Lattice도 원자 상태/장애물 프레임을 선택할 수 있게 하고, HOLD 발생 순간의 source·수신·producer·callback 나이를 기록한다. plan/watchdog 시작 시 DDS에 이미 도착한 프레임을 기존 callback으로 반영한다. 계산 끝에 새 프레임으로 나이만 갱신하지 않는다. 궤적은 시작 시 반영한 관측으로 검사한다.
- `s4_morai.launch.py`: `s4_atomic_sensor_frames=true`. 범용 Planning 기본값은 false이며 기존 연동 선택을 보존한다.
- `static_obstacle_planner.py`: NORMAL hazard 검사에서 기존 정확한 차체 기하 캐시와 16개 구간의 보수적 경계 상자를 사용한다. 차체·차축의 두 안전 거리 하한이 **모두** 증명된 원거리 구간만 정밀 호출을 생략한다. 각 원래 sweep의 최소 거리 값도 그대로 계산한다.
- `s4_morai_generalized_trial.py`: 센서 전송 파라미터를 기록하고, 정상 거리 시험 완료 후 기존 경로 추종을 유지하며 1km/h 목표로 감속한다. 실제 1.5km/h 이하에서 기존 정지를 호출한다. 감속도 samples/bag 주행 감사에 포함한다. 오류·센서·충돌·중단은 기존 즉시 정지로 처리한다.
- `s4_failure_forensics.py`: 닫힌 bag과 실제 Ego 차체로 계획/실제 경계 및 정지 구간 전체 Ego 프레임을 독립 분석한다.

v15의 차선 후보 우선·완결 Offset fallback 구조, 후보/복귀 기하와 물리 제한을 유지했다.
Global Path 파일, EKF, Map Matching, Pure Pursuit, PID, Control, 차량 기하와 속도 계획 소스는 변경하지 않았다.
센서 0.6초 한도, digest·fault/replay·세션·차체·금지선·도로 포함·실제 PP rollout 검사를 유지했다.

## 약 2.1km 센서 정지의 원인과 동일 구간 비교

원본 전체 bag의 최초 HOLD는 **226.151809초**의 `S4_SENSOR_SENSOR_SOURCE_STALE`다.
과거의 226.206초는 감독 샘플에서 처음 관측한 시각이며, 400초 뒤의 `live topic freshness failed`와 다르다.
당시 producer는 약 **0.090초** 전 source의 새 READY 프레임을 이미 발행했다.
Planning의 마지막 완성 쌍은 그 새 source보다 약 **0.535초** 오래됐고, 정지 시 source 나이는 약 **0.625초**였다.
100ms 주기의 Planning 계산은 직전 최대 **173ms**로 늘었다. 분리 상태/장애물 토픽의 콜백·짝짓기 대기와 계산 지연이 겹쳤다.
이는 LiDAR 송신 자체가 끊겼다는 증거가 아니다. 나중의 ready=true가 앞선 HOLD를 부정하지 않도록 HOLD 순간 증거도 추가했다.

원자 프레임과 계산 최적화만 적용한 전체 재시험도 2210.478m에서 같은 HOLD가 재발했다.
그때 producer의 최신 source 나이는 약0.182초, Planning 직전 계산은 최대149.7ms였다.
Humble의 timer가 subscription task보다 먼저 실행되면 DDS 큐의 새 완성 프레임을 반영하기 전에 오래된 보관 프레임으로 정지할 수 있었다.
이를 plan/watchdog **시작**의 nonblocking DDS take로 보완했다. 같은 상호배제 callback group에서 기존 frame callback과 검증기를 사용한다.
실제 DDS에만 도착하고 planner executor는 한 번도 돌지 않은 격리 재현, 입력 단절, 오래된 heartbeat 정지를 함께 검증했다.
`atomic_inbox_frames`는 실제 사전 수신 횟수다. 진짜 공백·invalid·replay·fault 검사는 그대로 적용된다.

같은 A102 시작20m / NORMAL60·SHARP30·S4 10 / 목표250m 실제 재시험:

| 구현/종료 | 결과 | 센서·경계 판정 |
| --- | --- | --- |
| 수정 전 v15 | 162.197m에서 센서 정지 | HOLD 1, Local Path 소실 1 |
| 원자 프레임만 적용 | 250.718m 도달 | 주행 HOLD 0, 종료 제동 중 실제 B11264 접촉 |
| 경로 유지 감속, 계산 최적화 미완성 | 감속 중 센서 한도 초과 | 실패 보존, 성공 제외 |
| 정확한 기하 캐시만 적용 | 감속 말기 약1.7km/h에서 source 0.604885초 | 실패 보존, 한도 유지 |
| 기하/block 보완 버전+경로 유지 종료 | 250.116m 도달, 감속 포함268.653m | 동일 구간은 모두0, 이후 전체 경로에서는 DDS 대기 HOLD 재발 |
| 최종 DDS 사전 수신+위 보완 | 250.945m 도달, 감속 포함273.194m | HOLD 0, 경로 소실 0, 충돌 0, 실제 차체 경계 접촉 0 |

최종 성공은 감속·실제 정지까지 검사했다. Local Path 최대 발행 간격은 약 **0.235초**이며,
종료 후 정지 구간의 전체 Ego 프레임 감사도 접촉 0이다.
최종 근거: `inbox_sensor_segment_orderly_stop/`, `inbox_segment_final_forensics/`, `inbox-segment-audit.log`.
이전 block 버전의 성공·전체 실패도 별도 폴더에 보존했다.

## 실제 금지 경계 접촉은 원인을 나누어 판정한다

**A303의 원본 전체 주행:** 금지 B2 `B219BS011096`, `B219BS011097`에 실제 차체 sweep 47표본이 접촉했다.
45표본은 실제 자세의 차체 자체가 접촉했고, 실제 차체를 Global Path 중심·접선에 놓아도 45표본이 접촉했다.
그 구간의 실제 횡오차는 최대 **0.03193m**다. 조향 추종 오차만으로 설명되지 않는 원래 경로·차체 여유 문제다.
모든 접촉 표본은 NORMAL이며 회피 진입/유지/복귀가 아니다.

계획 계약은 폭1.860m·길이4.899m이고, 독립 MORAI Actor box는 폭약2.103m·길이약4.915m다.
명목 차체 감사도 41표본 접촉했다. 실제 box 폭 차이가 모든 접촉을 설명하는 것은 아니다.
Actor box는 사후 감사에만 사용했고 planner에 정답 치수를 주입하지 않았다.
NORMAL의 기존 경계 모드는 diagnostic이며 실제 회피의 경계 gate와 구분한다.
근거: `original_final_forensics/forensics.json`, `boundary-contacts.json`, `boundary-evidence.png`.

**동일 구간 시험의 종료 제동:** 처음에는 목표 거리 이후 `steer=0`, `brake=0.65`로 감속해 곡선에서 실제 횡오차가 최대약0.419m가 됐다.
정지 중 전체 Ego 프레임 48표본에서 `B219BS011264` 접촉을 확인했다. 이 위치의 Global Path 차체는 접촉하지 않았다.
이는 시험 종료/기존 정지 제어 경로의 문제이며 정적 회피 궤적 선택 실패와 다르다.
기본 Control 소스를 보존하고 정상 시험 종료 절차를 수정한 뒤 최종 동일 구간에서 접촉 0을 확인했다.
이 수정은 목적지 자율 정지 또는 모든 비상 정지의 곡선 추종 완료를 의미하지 않는다.
근거: `after_stop_detailed_forensics/`, `block_segment_final_forensics/`, `inbox_segment_final_forensics/`.

## 실제 정적 배치와 지도 근거

초기 원자 프레임 버전의 실제 정적 시험은 19개다.
기존 성공 회귀 9개(10km/h 7개, 기존15km/h 2개)와 새 seed `2026111802`의 8개, 기존 A404 실패·A632 재현을 수행했다.
실행상 회피·복귀 15개 중 **12개가 전체 배치 엄격 기준을 통과**했다.
추가2개는 회피 구간의 실제 차체 도로 포함·복귀·무정지 조건을 확인했으나 NORMAL 지도 증거가 부족했다.
새 A404 08은 회피·복귀했지만 회피 도로 포함 미증명 6표본으로 엄격 성공에서 제외했다.
A404 거부3개와 A632 탐색 시간 초과1개도 보존했다. 이 19개에서 주행 센서 HOLD·실제 차체 금지선 접촉·충돌 메시지는 모두 0이다.

기존 통합 배치의 NORMAL 미증명 533표본(이전 v15는520), Offset edge의31표본(이전32)은 회피 구간 미증명과 구분한다.
두 배치의 회피 구간 미증명은 0이고 실제 모든 회피 commit의 이동 복귀를 확인했다.
전체 엄격 기준을 완화해 이 둘을 성공으로 바꾸지 않았다.

최종 `57d9922…` 소스로 차선·작은 Offset·Offset→차선 통합 성공 배치3개를 추가 재주행했다(`final_inbox_static10/`).
세 배치 모두 실제 회피·복귀, 센서 HOLD0·충돌0·차체 경계 접촉0·회피 도로 미증명0을 확인했다.
차선/작은 Offset은 전체 엄격 통과, 통합 배치는 NORMAL 지도 미증명528표본으로 전체 엄격 통과에서 제외했다.
초기19개를 최종 지문의19개 재시험으로 표기하지 않는다. 정상/실패/전체 미션을 합친 실제 실행은 **31개**이며 모든 버전을 보존했다.

- **A632:** A2 정합은 확인되지만 양쪽 B2 근거가 없고 측정 도로 mask가 원래 차체도 포함하지 못한다. 실제 도로가 없거나 물리적으로 회피 불가능하다는 뜻은 아니다. 지도 증명 공백과 길어진 실패 탐색/시간 초과를 각각 기록한다.
- **A404:** 양쪽 B2 근거가 있으나 차선 끝·복귀/제어 tail의 증명 공백이 있다. 늦은 장애물은 `OFFSET_RETURN_UNPROVEN`으로 거부됐고 이른 배치도 실제 차체6표본이 미증명이다. 현재 후보의 실패를 모든 물리 경로의 불가능성으로 단정하지 않는다.
- 실제 제약으로 회피 불가능성이 확인되는 경우 안전 정지는 정상 판단이다. 위 미증명 배치를 그 증명 없이 불가능 배치 또는 회피 성공으로 처리하지 않는다.

seed/실제 치수/배치는 `random-manifest.json`, 각 `summary.json`과 `placement_scale_verification`에 있다.
Actor API는 배치·복원·사후 감사에만 사용했고 실제 LiDAR로 판단했다. 실패 후 배치를 재추첨하지 않았다.

## 전체 미션과 남은 범위

원본 약2444m Global Path, 시작 A380·18개 경유지·마지막 목표 A119BS010184를 보존했다.
원자 프레임 버전의 전체 재주행은 센서 정지를 지나 **2226.340m**에서 A378 `BRAKING_DISTANCE`로 중단됐다.
당시 센서는 준비 상태였으며 장애물 여유63.749m, 계산 정지거리66.494m였다. 삽입한 NPC가 없는 기본 주행이다.
확인 전 메모리 hazard/정지 gate와 실제 도로 물체·기준 경로 관계의 추가 검증이 필요하다.
회피 경로를 실행한 실패와 구분하지만, S4의 공통 진입 안전 정책도 관련되므로 이를 전부 범위 밖으로 제외하지 않는다.
해당 전체 시험의 HOLD는0, 실제 차체 경계 접촉은48표본이었다. 미션 성공이 아니다.

기하/block만 보완한 다음 시도는2210.478m에서 센서 HOLD로 실패했다(`full_route_final_block/`).
이는 최종 성공 결과가 아니며, DDS 사전 수신 보완의 재현 근거로 보존했다.

**최종 DDS 보완 버전**은 기존 센서 실패와 A378 정지 구간을 지나 감독 목표 **2437.156m**에 도달했다(`full_route_final_inbox/`).
실제 최고59.814km/h, 최대 제어 추종 오차0.414m, 센서 HOLD0·MORAI 충돌 메시지0이었다.
그 이후 목적지 감속에서 reference가 **0m**로 소진되고 `INVALID_INPUT_OR_CLOCK`가 발생했다.
직전 local length는2.368→1.708→1.059→0.321→0m였다. 거부 시 실제 속도22.28km/h, 센서는 ready·source 나이약0.032초였다.
시간 역행의 증거가 아니라 목적지 도달 전 정지 계획 없이 경로가 끝난 일반 미션 입력/종료 문제로 분리한다.
`goal_reached=False`와 reference 최소 길이 검사를 바꾸어 통과시키지 않았다.
근거: `goal-termination-failure.json`, `full_inbox_final_forensics/`, `inbox-full-audit.log`.

최종 전체 주행의 실제 차체 금지선 접촉은 **49개 sweep 표본**이며 모두 NORMAL A303이다.
실제 자세 접촉47, Global Path 접선에 놓은 실제 차체 접촉45, 접촉 구간 최대 횡오차0.03235m다.
발행 Local Path와 Global Path의 해당 기준점 차이는 수치 오차 수준이었다(최대약2.4e−13m).
기준 경로 여유 문제는 센서 수정으로 해결되지 않았다. 이 때문에 **전체 안전 완료가 아니다**.
Local Path 소실1회는 목적지 입력 거부 시의 빈 경로이며 센서 HOLD에 의한 소실과 구분한다.
최종 전체 Planning 시간은 p95약102ms·최대228ms였다. 아래77% 계산 절감을 전체 callback의 실시간 보장으로 확대하지 않는다.

기본 Global Path 여유·기존 일반 정지/목적지 제어 문제는 정적 회피 실행과 별도 이슈다.
전체 미션 완료에는 영향을 주며, 정적 기능 자체의 검증 결과와 함께 보고한다.
기존 FSM의 `goal_reached=False`와 감독기의 거리 도달 후 정지는 목적지 자율 정지 검증과 다르다.
정적 자체 판정에는 A404 회피 영역 미증명, A632 도로 근거 부족과 실패 탐색 지연을 남긴다.
NORMAL 경로 여유와 목적지 정지는 전체 미션 완료 기준에 별도로 반영한다. A~F는 시작하지 않았다.

## 테스트·계산 검증

- 최종 `final-inbox-regression.xml`: **294 passed**. v15의251개 범위에 센서/기본 속도/캐시·block·실제 DDS 큐 회귀를 포함한다. 이전 block 단계293개 결과도 보존했다.
- `build-final-inbox.log`: Planning/Bringup 빌드 성공. `git diff --check`, 도구 문법 검사 통과.
- `block-hazard-benchmark.json`: 기록 입력66개 장애물·161구간. v15 전수 검사와 hazard 및 최소 거리 값이 정확히 같다. 반복 중앙값40.08→9.39ms, cold41.10→21.46ms. 오프라인 계산 측정이며 실제 주행 성공으로 세지 않는다.
- 잘못된 stdin 기반 pytest/중단한 광범위 시험과 오래된 Mock 기반 검사 실패도 별도 보존했다. 명시적 v15/current 모듈 preload 비교는62개 실패가 양쪽 동일했다(`mock-test-baseline-comparison.json`). 최종294개 통과와 혼동하지 않는다.

## 실행·보존

기존 스택이 정지하고 `/ctrl_cmd` 발행자가1개인 상태에서 사용한다. Launch를 중복 기동하지 않는다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 tools/s4_morai_generalized_trial.py \
  --link A219BS010102 --start 20 --follow-distance-m 250 \
  --approach-speed-kmh 60 --seconds 70 --prepare-before-spawn --record-bag \
  --output local_data/s4_followup_20261118/새_재현_디렉터리
```

원본 프로필은 `atomic-original-profile.yaml`, 정적 검증 프로필은 `atomic-validated-profile-10.yaml`, 기존15 회귀는 `atomic-validated-profile-15.yaml`이다.
새 난수 배치는 `random-manifest.json`, 기존 배치는 `regression-manifest.json`이다.
정지·기존 노드 종료·새 지문 확인을 수행하는 교체 도구는 결과 루트의 `reload-planning.py`다.
원본/현재 SHA·파일 해시·diff·소스 archive, 닫힌 bag·JSONL·파라미터·명령·최종 복원 결과를 이 결과 루트에 보존한다.
기준선은 `d449993` 또는 보존 tar를 별도 디렉터리에 해제해 대조할 수 있다. 현재 작업 트리를 초기화할 필요가 없다.

최종 보존 파일:

- `final-source-preservation.json`, `worktree-final.diff`, `worktree-final-source.tar.gz`: HEAD·구현 지문·변경 해시·전체 소유 소스와 재현 지원 코드.
- `final-execution-ledger.json`, `experiment_summary.csv/json`: 시험별 실제 지문·파라미터·판정·재현 argv. 별도의 기존 campaign 명령 기록도 유지한다.
- `final-parameters/`: 실행 중인 Planning·Control·LiDAR·Localization 실제 전체 파라미터. Localization은 제한시간 있는 직접 List/Get RPC로36개 전부 보존했다.
- `input-map-route.tar.gz`, `morai-grpc-support.tar.gz`: 실제 지도/저장 경로 및 현재 임시 SDK 지원. SDK 지원 archive의 경로 기준은 `/tmp`다.
- `final-data-hashes.json`: 닫힌 bag·JSONL·설정·소스·그림의 파일별 SHA-256. 실행 중인 planner 로그는 별도 닫힌 snapshot을 해시한다.
- `final-restoration.json`, `final-state-verification.json`: 최초 Ego 복원 XY 오차약0.00578m, 원래 Actor는Ego만, `/ctrl_cmd` 발행자1개, drive=false, 목표/accel0·brake0.65, 센서 ready, 기본 `lattice`·60/30/10.

원래 기본 선택·속도·장면을 복원했고 센서 수정 선택 `s4_atomic_sensor_frames=true`만 의도적으로 유지했다.
최종 상태와294개 회귀 통과는 정적 전체 완료 또는 목적지 자율 정지 완료 판정이 아니다.
