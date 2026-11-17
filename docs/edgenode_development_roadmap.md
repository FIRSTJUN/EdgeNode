# EdgeNode 단계별 독립 개발 진행표

이 문서는 실제 실행·검증 상태를 기록한다. 계획 항목을 구현 완료로 취급하지 않는다.
기준 브랜치 `s4-finish`, 착수 SHA `363c4c9f68c15f31d7e525cf13568f6f827869ce`.
현재 수정은 미커밋이며 커밋·푸시를 수행하지 않았다.

## 현재 단계: 정적 장애물 통합 안정화

소스 루트: `/workspace/ros2_ws/src/edgenode_planning/edgenode_planning/`.

| 기능 | 실제 소스 | 실행 선택 | 상태 |
| --- | --- | --- | --- |
| 기존 정적 Lattice | `static_obstacle_planner.py` | `static_avoidance_method=lattice` | 기본 선택 유지 |
| 차선 후보 선택 | `lane_candidate_bank.py`, `lane_candidate_planner.py` | `static_avoidance_method=lane_candidates` | 명시적 실험 모드, 최종 완료 판정 보류 |
| 완결형 Offset·차선 끝 복귀 | `lane_candidate_planner.py` | 위 실험 모드 내부의 전략 선택 | 진입·통과·복귀를 함께 검증하고 실행 중 최신 입력 재검사 |
| 실행 코드 증거 | `planning_provenance.py` | `/planning/avoidance_debug` | 코드 지문과 실험 스냅샷 대조 |

설정·결과 루트: `/workspace/local_data/s4_static_integrated_validation_20261118/`.
실험 설정: `validated-profile-10.yaml`, 원래 설정: `initial-planning_node.yaml`.
기본 주행의 Global Path, EKF, Map Matching, Pure Pursuit, PID 및 Control 소스는 이번 정적 개선에서 변경하지 않았다.
차선/Offset은 정적 회피 안에서 하나의 실행 담당자가 선택하는 전략이다. 후속 동적 추적·추월·고속 추종 알고리즘과 혼합하지 않는다.

실험 실행 예시(기존 스택이 준비되고 정지한 상태):

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/s4_random_static_campaign.py run \
  local_data/s4_static_integrated_validation_20261118/unit-verified-integration-manifest-10.json \
  --output /workspace/local_data/새로운_실험_디렉터리
```

실행 중인 스택에 Launch를 추가 실행하지 않는다. Planning 교체는 정지·drive 비활성 상태에서 기존 프로세스 종료를 확인한 뒤 하나만 기동한다.
`/ctrl_cmd` 발행자는 항상 정확히 하나여야 한다.

최종 오프라인 검증: 251개 통과, Planning/Bringup 빌드 성공. 실제 주행 성공과 별도 증거다.
최종 선택 구현은 v15이며, 전체 native 도로 검사를 확대한 v12~v14는 기존 성공을 차단하여 별도 보존 후 비채택했다.

최종 MORAI 판정·경계·지도 증명 공백·실패 배치는 위 결과 루트의 `report_ko.md`, `experiment_summary.csv`를 따른다.
정적 최종 판정 전에는 아래 후속 단계에 착수하지 않는다.

## 후속 단계의 격리 계획

다음 경로는 계획이며 아직 생성·구현하지 않았다. 정적 완료 기준선을 보존한 뒤 별도 브랜치/worktree에서 단계별로 시작한다.

| 단계 | 계획된 독립 경로 | 선행 조건 | 검증 상태·SHA |
| --- | --- | --- | --- |
| A S자 고속 추종 | `/workspace/experiments/s_curve_tracking/` | 정적 안정화 완료·기준선 보존 | 미착수, SHA 없음 |
| B Lattice 20km/h | `/workspace/experiments/lattice_speed_20/` | 정적 기준선 및 A 완료 | 미착수, SHA 없음 |
| C 동적 객체 추적 | `/workspace/experiments/dynamic_tracking/` | A·B 완료 | 미착수, SHA 없음 |
| D 직선 저속차 추월 | `/workspace/experiments/dynamic_overtake_straight/` | C 검증 완료 | 미착수, SHA 없음 |
| E 버스정류장 추월 | `/workspace/experiments/dynamic_overtake_busstop/` | D 반복 성공 | 미착수, SHA 없음 |
| F 60km/h 혼합 교통 | `/workspace/experiments/dynamic_traffic_60/` | D·E 안정화 | 미착수, SHA 없음 |

각 단계는 소스, YAML/Launch, 테스트, seed/배치 manifest, bag/JSONL, 보고서, 실행 명령, 기준 SHA, 마지막 성공 상태 및 재개 위치를 독립적으로 보존한다.
새 기능의 실험 Launch에서만 활성화하며 기본 Launch나 이전 기준선에 자동 반영하지 않는다.
공유 인터페이스가 필요하면 독립 Adapter/Router로 연결하고 제어 발행 담당자는 하나로 유지한다.
Actor API는 배치·속도 설정·복원·사후 정답 감사에만 사용한다.

## 다음 재개 위치

현재 단계의 크기 검증 회귀시험, 무작위 독립 감사, 같은 파라미터·실제 장애물 치수 비교의 최종 결과는 보고서를 따른다.
남은 과제는 도로 영역 근거, 복귀 가능 공간과 탐색 지연, 기본 전체 미션의 미충족 항목이다.
지도 근거가 없는 A632, 일부 A404/교차로 표본 및 실패한 복귀 경로는 성공으로 포장하지 않는다.
원래 장면·전체 설정 복원은 결과 루트의 `final-state-verification.json`, 변경 보존은 `modified-files-final.json`과 `worktree-final.tar.gz`를 확인한다.
미완성 항목을 해결하기 전에는 후속 단계로 넘어가지 않는다.
