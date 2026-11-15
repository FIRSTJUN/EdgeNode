# NORMAL 경계 오정지 수정 — 2026-11-15 KST

기준 HEAD `ffc382c`. 커밋·푸시는 하지 않았다. 목표 60/30/10, 저장 미션과 장애물
배치, 기존 차선 기반 S4 래티스를 보존했다. 기존 성공한 15km/h 일반 주행과
장애물 회피, 전체 회귀시험은 반복하지 않았다.

## 실제 접촉과 링크 변경

기존 Planning 로그에서 21:54:40.820 KST `READY → NORMAL_DRIVE`,
21:54:49.678 KST 해당 경계 충돌과 `NORMAL_DRIVE → EMERGENCY_STOP`을 확인했다.
약 8.86초 뒤의 정지다. `A219BS010380`은 저장 미션의 시작 링크이며, 조사 당시
위치는 미션의 `A219BS010813` 교차로 연결을 지난 `A219BS010398`이다.
현 위치는 A010398 중심선에서 약 0.40m, A010380 끝점에서 약 16.53m 떨어져 있다.
정지 당시 링크의 연속 기록은 없어 링크 전환 시각까지 확정하지 않는다.
Localization reset 기록은 비어 있었으며 이번 작업에서 차량을 재배치하지 않았다.

`B219BS011367`은 황색 단일 실선 중앙선 `111/501`, `R_linkID=A219BS010399`,
`L_linkID`는 공란이다. 반대 방향 링크에만 연결된 중앙선으로, 링크 ID가 다르다는
이유로 제외할 수 없다. 지도와 Sonata 차체의 기하학적 교차가 계산된다.
MORAI 물리 충돌 메시지는 0이었다.

- 첫 접촉의 Planning 저장 자세: `(49.773033, 1149.898300, -9.855150°)`.
  후륜 기준 Global Path 오차 약 **0.00577m**에서도 전방 차체는 중앙선에 닿았다.
- 조사 시작의 제동 후 Ego 자세: `(53.708591, 1148.528320, -19.757799°)`.
  경로 방향 약 -28.12°보다 약 8.36° 덜 우회전했고 전방 좌측 차체가 교차했다.
- 원래 8.86초의 연속 실제 궤적·속도·Lookahead·조향 기록은 없다. 저장된 접촉
  자세와 현재 자세, 지도, 계산 재생을 근거로 분석했으며 연속 실측 복원을 주장하지 않는다.

## NORMAL과 S4의 역할 분리

| 검사/출력 | 회피 필요 장애물 미확인 NORMAL | 확인된 장애물의 S4 회피·복귀 |
|---|---|---|
| 경로 | 기존 Global → Local Path 객체 유지 | 검증한 임시 회피/복귀 경로 |
| 경계 접촉 | `boundary_contact_reason`, `boundary_check_mode=diagnostic` | 전체 경계 검사로 후보 거부/정지 |
| 경계 이유의 FSM 연결 | `LANE_BOUNDARY_COLLISION`으로 정지시키지 않음 | 안전한 회피가 불가능하면 정지 |
| LiDAR 실제 차체 충돌·최소 간격, 무효 입력, 센서 신선도 | 유지 | 유지 |
| 중앙선·외곽·차체 swept polygon·rollout 검사 | 회피 후보 생성 없음 | 유지 |

`_obstacle_safety()`를 분리해 `_normal_hazards()`가 경계 이유 때문에 실제 경로
장애물을 누락하지 않게 했다. 선행 `_body_safety()`의 경계 정지 연결은 회피 중에만
사용한다. NORMAL에서도 확인 횟수와 진입 거리 조건을 충족한 경로 장애물이 있으면
S4 경계 정지 검사를 활성화한다. 기존 래티스 생성·선택·복귀는 보존했다.

## 일반 주행 제어 개선

교차로 출구 경로에는 약 -6.51°에서 -28.12°로 방향이 바뀌는 구간이 있다.
기존 허용값 `1.6rad`는 해당 교차로의 누적 회전을 대부분 통과시켜 30km/h에서
약 6.67m의 Lookahead를 그대로 사용한다. 첫 접촉 자세에서 이때 PP 조향은 약
-0.3845, 3.5m 목표의 조향은 약 -0.6277이다(좌회전 양수인 PP 정규화 값).
출구에서 우회전이 충분히 진행되지 않은 전방 차체와 큰 preview의 관계를 확인했다.
원래 순간의 실측 속도·조향이 없어 단일 원인으로 확정하지 않는다.

Control에 `normal_lookahead_heading_budget_rad`를 추가하고 MORAI Launch에서
`0.25rad`로 설정했다. 직선의 속도 기반 preview, 3.5~12m 범위, PP/PID와 조향
부호·상한·변화율은 유지한다. S4 Control/rollout의 `1.6rad`도 유지했다.
동일 지도·30km/h·조향 지연/변화율의 자전거 모델에서 최대 경로 오차가 약
0.355m에서 0.131m로 줄었다. 두 모델 모두 해당 중앙선 접촉은 발생하지 않았다.
이는 모델 비교이며 원래 MORAI 실측 재현이나 60/30 안정성 입증은 아니다.

## 현재 위치의 실제 확인

Planning/Control 두 프로세스만 교체하고 Localization·LiDAR·편집·시각화·MORAI는
재시작하지 않았다. PID와 실행 설정은 `logs/s4-normal-boundary-20261115/after-reload-state.json`에
보존했다. 교체한 두 프로세스는 기존 Launch의 자식이 아니므로 종료할 때는 그 기록과
현재 실행 명령을 대조해 개별 SIGINT로 종료해야 한다.

- 재로딩 후 `S4=NORMAL`, `FSM=READY`, 경로 장애물 0. 기존 경계 접촉은 진단에만
  남았고 정지 latch는 비었다.
- 검증 준비 1회는 Global Path 수신 전 주행 활성화 없이 종료했다. 첫 활성화도
  감시기가 `WAIT_FRESH_VALIDATION` 센서 복구 대기를 실패로 처리해 실제 이동 전에
  종료했다. 이 독립적인 신선도 안전 검사는 제거하지 않았다.
- 이후 현재 위치에서 **실제 NORMAL 이동 1회**: 정지 명령 전 **6.271m**, 최고 실측
  **20.278km/h**, 최대 Control 경로 오차 **0.465m**. 시작부터 있던 차체 접촉은
  약 **1.281초** 후 해제됐으며 마지막 이동 표본의 해당 중앙선 간격은 약 **0.824m**다.
  처음부터 접촉이 없었다고 기록하지 않는다.
- 경로 장애물 0, S4 entry/return commit 0, 물리 충돌 0. 실제 Ego 궤적, Global/Local
  Path, Lookahead, PP/적용/실측 조향을 기록했다. 이들은 최신 callback 표본이므로
  Ego와 Localization의 순간 차이에 전송·갱신 시차가 포함될 수 있다.
- **60/30/10 유지**, 실제 Global Path는 이전 경로의 정확한 suffix, 저장 미션 파일의
  SHA-256도 동일했다. 장애물 배치를 변경하지 않았다.
- 종료 후 `enable_drive=false`, 실제 속도 약 **0.00116km/h**, 목표 0, 가속 0,
  브레이크 0.65, `stop_confirmed=true`, `FSM=READY`를 확인했다.

추가 정지 관측에서 `/control/path_error`는 약 0.822m였다. 이는 Local Path의
단조 진행 기준 시작점이 앞에 남아 생긴 거리도 포함한다. 같은 표본의 실제 Ego와
Global Path 간 횡방향 거리는 약 0.275m, Localization 기준은 약 0.283m였다.
정지 후 0.822m를 실제 횡방향 이탈로 해석하지 않는다. 이 관측은
`parked-path-diagnostics.json`에 남겼으며 경로나 진행 기준을 임의로 재설정하지 않았다.

관련 검사 **19개**, Planning/Control/Bringup 3패키지 빌드, 문법·diff 검사가 통과했다.
실제 두 접촉 자세의 NORMAL 통과, 확인된 장애물의 경계 정지, S4 단계·후보의 중앙선
검사, 장애물 판별 누락 방지, 실제 장애물·센서 정지, NORMAL/S4 preview 분리를 검증했다.
전체 회귀시험은 하지 않았다. 현재 위치의 출발은 확인했으나 원래 코너의 변경 후
재통과는 수행하지 않았다.

근거는 `logs/s4-normal-boundary-20261115/`의 `planning-diagnostics-before.json`,
`geometry-before.json`, `replay-before.json`, `tracking-model-comparison.json`,
`normal-tracking-comparison.png`, `current-start-normal/{summary.json,samples.jsonl,tracking-analysis.json}`이다.
회귀 fixture는 Planning의 `test/fixtures/s4_normal_boundary_start.json`에 보존했다.
