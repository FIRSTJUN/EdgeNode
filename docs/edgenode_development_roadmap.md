# 2026-11-19 단계별 현재 진행

v61 `29eef7e`, A `fc81ffb`, B `000045f`를 보존했다. C도 정의한 실제 MORAI 추적7시험·621개 회귀·2패키지 빌드를 통과했다. C 지문은 `6aa36ba2836584ea398c7fa056bca06c8785d94c9d179af6ea35be654957718d`이며 완료 SHA는 C 보존 묶음 `commit-receipt.json`에 기록한다. D→E→F는 C 완료 커밋 후 같은 `s4-finish`에서 순서대로 진행한다. 아직 검증하지 않은 동적 추월·혼합 교통을 완료로 표시하지 않는다.

각 단계 소스/프로필/전용 Launch/결과 폴더를 분리하고 기존 기본 Launch·완료 소스·지도·경로·증거를 유지한다. 실제 안전 정지와 성공을 구분하고 Actor 정답은 배치·복원·사후 감사에만 쓴다. push는 Ubuntu 호스트에서 사용자가 수행한다. 별도 하이패스3D 연구는 범위 밖이다.

A [완료 기록](s4_a_scurve.md), B [완료 기록](s4_b_static20.md), C [완료 기록](s4_c_dynamic_tracking.md)와 [근거](validation/s4_c_evidence.json)에 재현 명령·실제 속도·범위·한계·실패 보존이 있다.

아래는 v61 및 이전 시점의 보존 기록이다.

---

# 2026-11-19 정적 안정화 최종 실제 검증

검증한 MORAI 조건에서 정적 안정화와 전체 기본 미션 검증을 완료했다. v61 Planning `c3ecbc84fd667e47e7ea125ac4f5c69b021a5f64ebd5ded0af2ca14c69d19e52`로 정적 21배치(기존·연속·새 무작위), 정상 전체 경로 2회, 4장애물 전체 미션, 실제 생산 중단·물리 차단 안전 정지를 통과했다. 18개 경유지·자율 목표 정지·독립 안전 감사를 확인했다.

기본 `lattice`와 v15를 보존했고 검증 프로필은 명시적 `lane_candidates`다. 원본 Global Path·Control·Localization은 보존했다. A~F와 별도 3D 지붕·기둥 인식은 미착수다. 사용자가 승인한 최종 로컬 보존 커밋을 수행하며 푸시는 사용자 호스트에서 수행한다. 현재 실행은 사용자가 복원한 기본 `lattice`로 주행 비활성·정지 상태이며, 실제 검증 프로필과의 차이 및 정확한 재현 명령은 최종 기록에 명시했다.

상세 결과·제한·명령·SHA·복원은 [최종 검증 기록](s4_static_final_completion.md)을 따른다. 아래는 이전 시점의 보존 기록이며 당시의 미완료 표기를 현재 상태로 해석하지 않는다.

---

# EdgeNode 단계별 독립 개발 진행표

이 문서는 실제 실행·검증 상태를 기록한다. 계획 항목을 구현 완료로 취급하지 않는다.
기준 브랜치 `s4-finish`, 이전 착수 SHA `363c4c9f68c15f31d7e525cf13568f6f827869ce`.
v15 보존 커밋은 `d449993b021bd96fd29781d7759b99a18970e4cf`이며, 2026-11-18 센서 보완은 이 HEAD의 깨끗한 작업 트리에서 시작했다.
이번 보완은 미커밋이며 기존 변경 초기화·커밋·푸시를 수행하지 않았다.

## 2026-11-19 현재 작업

v59 생산 코드(`da7ec04a58426553ee7beda3754c46ac534230319e9ada5fc3c5f438cbe8644c`)로 동일 전체 장애물 배치를 재시험 중이다. v56의 두 전체 일반 주행과 21개 정적 배치 성공은 버전별로 보존하며 v59 완료로 이전하지 않는다. v58 A632 조기 복귀의 새 관측 여유 실패를 수정했다. 외부 호스트 주소 변경과 MORAI DDS 수신 단절도 설정·장면 보존 후 복구했으며 회피 알고리즘 실패와 구분한다. 최종 완료는 계속 보류, 후속 A~F는 미착수다.

## 현재 추가 검증 상태 — 정적 최종 완료 보류

후속 A~F는 미착수다. 이번 최종 안정화의 현재 증거와 남은 완료 조건은 [정적 최종 검증 기록](s4_static_final_completion.md), 결과는 `/workspace/local_data/s4_final_completion_20261118/`를 따른다. 아래 v15 및 센서 보완 기록은 해당 시점의 보존 기준이다.

전체 저장 경로 2444m·18개 경유지의 실제 자율 목표 정지 1회를 확인했다(`full_v32_native_normal_01`). 센서 HOLD·금지 B2·물리 C3 접촉 0이며, v37 측량 지도의 **사후** 도로 포함 감사도 미증명 0이다. 당시 실행 지도·소스와 현재 최종 후보를 구분하며 최종 후보의 반복 전체 주행은 아직 진행 중이다.

실제 차체, 혼합선 방향 허용, A303 병합 및 A073 중심선 국소 보정, 유한 목적지 감속·정지, 동치 네이티브 센서 계산, 물리 C3 독립 hard gate, 완결 Offset 진입·통과·복귀와 인계 조건을 보완했다. 원본 저장 Global Path·18개 경유지·v15·Control·Localization은 보존했다. A632/A404 도로 증명은 실제 C3/A2 자료로 복원했으며 임의 폭·convex 공백 채우기는 사용하지 않았다.

현재 v56 Planning 후보 지문은 `9397a056b19d8cc859e7e3eb582105a9f2041d4ca48502428b75f6ebdf4077c8`다. 15km/h 횡가속도 한도는 유지하고 명목 계획 여유를 추가해 기존 두 15km/h 배치를 실제 재통과했다. A632 관측 외곽 확대 시 여유 선택 및 A404 경유지 보존 조기 복귀를 보완했으며 실제 최종 자격은 진행 중이다. 기존 배치의 실제 회귀·15km/h·새 무작위 배치·전체 기본 주행 반복·장애물 포함 전체 미션·최종 센서 중단 및 불가능 배치 검증이 모두 통과하기 전에는 완료로 변경하지 않는다.

v56에서 기존 9개·15km/h 2개·연속 2개·새 무작위 8개와 전체 일반 주행 2회를 실제 통과했다. 그러나 장애물 4개를 포함한 전체 주행은 굽은 진입의 탐색/인계 계약 때문에 실패했다. 실패를 물리 불가능이나 지도 부족으로 처리하지 않는다. Global/지역 station 정합, 로컬 캐시, 대기 속도 제한, 공간 복귀 및 진입 정렬을 보완한 v58의 동일 배치 재시험을 진행 중이다. **미완료, A~F 미착수**이며 자세한 버전별 기록은 최종 검증 문서를 따른다.

## 2026-11-18 센서·전체 경로 보완 검증

결과 루트: `/workspace/local_data/s4_followup_20261118/`.
v15 전체 소스와 SHA-256은 `preserved-v15-source.tar.gz`, `initial-manifest.json`에 별도로 보존했다.
차선 후보/Offset 경로·차량 기하·Pure Pursuit·Control·속도 계획 소스는 보존했다.
`static_obstacle_planner.py`의 NORMAL hazard 계산만 기존 정확한 기하 캐시·보수적 block 하한으로 최적화했다. 후보·안전 거리·정밀 검사 결과와 각 sweep의 거리 값은 유지한다.

- 원본 전체 주행 bag의 최초 SENSOR_HOLD는 226.151809초다. 과거 보고서의 226.206초는 감독 샘플에서 처음 관측한 시각이다.
- 직전 LiDAR 생산 프레임은 약 0.090초밖에 지나지 않았지만 Planning이 가진 완성 쌍은 오래된 프레임이었다. 100ms 주기의 Planning 계산이 최대 173ms로 늘면서 분리 토픽의 콜백·짝짓기 지연이 발생했다.
- `s4_atomic_sensor_frames=true`는 상태·장애물을 함께 담은 최신 완성 프레임을 사용한다. MORAI Launch 기본값은 true, 범용 Planning 기본값은 false다. 기존 0.6초·digest·fault/replay·세션·충돌·경계 검사를 유지한다.
- 동일 A102 구간은 수정 전 약 162.2m에서 센서 정지, 수정 후 약 250.7m 완료·센서 HOLD 0·Local Path 소실 0을 기록했다.
- 전체 저장 경로 재주행은 센서 실패 지점을 지나 약 2226.34m에서 `BRAKING_DISTANCE`로 중단됐다. 센서는 준비 상태였고 A378의 경로 장애물 여유 63.75m가 계산 정지거리 66.49m보다 짧았다. 전체 경로 완료로 판정하지 않는다.
- A303 금지선 접촉 47개 sweep 표본 중 실제 자세의 차체 접촉은 45개다. 실제 차체를 Global Path 접선에 놓아도 45개가 접촉하며, 접촉 구간 실제 횡오차는 최대 0.03193m다. 기본 경로·차체 여유 문제와 정적 회피 궤적 실행을 구분한다.
- 오프라인 집중 회귀 281개 통과, Planning/Bringup 빌드 성공. 오래된 Mock 기반 노드 테스트의 기존 실패와 잘못된 stdin 기반 pytest 실행 결과도 숨기지 않고 별도 보존한다.
- 후속 계산 보완을 포함한 최종 회귀는 293개 통과다. 같은 기록 입력의 hazard·최소 거리 값은 v15와 동일하며 반복 계산 중앙값은 약40.1→9.4ms였다.
- 계산 최적화 후 전체 경로에서 재발한 DDS callback 대기를 추가 보완했다. plan/watchdog 시작 전에 도착한 완성 프레임을 검증·반영하며 계획 끝의 시간만 갱신하지 않는다. 실제 DDS 대기/입력 단절/오래된 heartbeat를 포함한 최종 회귀는294개 통과다.
- 최종 동일 A102 구간은 정상 감속·정지까지 포함268.653m, 센서 HOLD·경로 소실·충돌·실제 차체 경계 접촉0을 확인했다. 종료 정지 시 조향0으로 경계를 접촉했던 별도 시험도 실패로 보존했다.
- 최신 DDS 보완 지문은 `57d9922c4e400fb376c7d5f1da5341b26d7f48244241ea89dd03090eb76f3bbf`다. 동일 A102 재시험은 감속·정지 포함273.194m, HOLD·경로 소실·충돌·경계 접촉0이었다.
- 최신 전체 시험은2437.156m의 감독 목표 거리에 도달하고 센서 HOLD0·충돌 메시지0을 기록했다. NORMAL A303 차체 경계 접촉49표본과 목적지 reference0m에 따른 안전 거부가 남아 전체 안전 완료는 아니다.
- 최신 소스의 차선·Offset·통합3배치도 회피·복귀 완료했다. 회피 도로 미증명/경계 접촉/HOLD는0이며 통합 배치 NORMAL의 지도 미증명528표본은 별도 유지했다.
- 실제 실행31개와 버전별 판정·소스·파라미터·bag·SHA·명령을 보존했다. 기본 `lattice`·60/30/10·drive=false 및 최초 Ego 위치(약5.8mm 오차)를 복원했고 제어 발행자는1개다.

차선·Offset·연속 회피 회귀와 seed `2026111802`의 새 장애물 8개 실제 시험 및 독립 감사는 이 결과 루트에 보존한다. 정적 최종 완료 판정과 A~F 착수는 계속 보류한다.
원인·수정·엄격 판정·버전별 한계·실행 명령은 [센서·경계 후속 검증](s4_sensor_boundary_followup_20261118.md)을 따른다.

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

v15 보존 시점 오프라인 검증: 251개 통과. 현재 센서·계산 보완은294개 통과, Planning/Bringup 빌드 성공이다. 실제 주행 성공과 별도 증거다.
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
