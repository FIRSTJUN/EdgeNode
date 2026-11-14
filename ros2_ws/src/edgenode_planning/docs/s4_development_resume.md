# S4 개발 재개 기록

2026-11-14 요청에 따라 11월 13일 개발 상태를 보존하고 개발 검증을 재개했다.
브랜치는 `s4-finish`, HEAD는 `79a28aa`다.

## 취소와 원본 보존

다음 세 파일만 SHA-256을 확인하고 백업한 뒤 제거했다.

- `tools/s4_morai_drive_check.py`
- `tools/test_s4_morai_drive_check.py`
- `ros2_ws/src/edgenode_planning/docs/s4_stability_20261114.md`

세 파일은 Git 미추적 파일이고, 오늘 작업 전 43개 manifest에 없으며,
11월 14일 수정 시각과 당시 보고서의 신규 파일 목록이 일치했다.
mtime만으로 생성일을 단정하지 않고 이 근거를 함께 사용했다.

기존 수정·신규 파일 **43개는 개발 재개 후에도 모두 SHA-256이 동일**하다.
영속 원본 manifest, 원본 tar의 파일 내용, `/tmp/s4-preserve-20261114T132932Z/manifest.json`을
서로 대조했다. 취소 전 Git 변경 목록은 43개와 위 3개의 합계 46개였다.
구분 불가능한 다른 변경을 되돌린 것은 없다.

백업과 증거: `/workspace/logs/s4-resume-20261114T143309Z/`

- `all-46-files-before.tar.gz`: 취소 전 변경·신규 파일 전체
- `cancelled-20261114/`: 제거한 세 파일의 원문
- `manifest-before.json`, `file-times-kst.json`: 파일별 해시·수정/메타데이터 변경 시각
- `working-before.patch`, `index-before.patch`, `head.txt`, `status-before.txt`
- `integrity-before.json`, `cancellation.json`: 43개 원본과 삭제 범위 검증
- `development-before/`: 이번 개발에서 추가 수정한 기존 검증 파일의 사본

Git 초기화·clean·브랜치 이동·커밋·푸시는 하지 않았다. 오늘 남은 실험 로그와
분리 빌드 디렉터리는 근거 보존 및 현재 실행 환경 유지를 위해 삭제하지 않았다.

## 11월 13일 실제 성공 범위

근거 디렉터리: `logs/s4-integration-20261113/`.

| 기록 | 확인한 결과 |
| --- | --- |
| `s4_15_to_10kph_fixed.json` | 568개 표본, NORMAL → DETECT → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL. 약 58.10초에 NORMAL 복귀, 목표 15km/h로 회복. 충돌 표본 0, 종료 후 DISABLED/READY/정지. |
| 같은 회피 기록 | 관측 최고 속도 15.902km/h. 회피/복귀 단계에는 10 및 안전 검증용 0km/h 표본이 있으므로 무정지 회피 성공으로 표현하지 않는다. |
| `final_profile_normal_15kph.json` | 311개 표본, 약 34.99초의 일반 주행. 최대 경로 오차 0.170821m, 0속도 목표 표본 0, 최고 15.914km/h, 종료 후 DISABLED/READY/정지. |
| `relocation_test.json` | 위치 재정렬 성공 및 노드 보존 표시가 있으나 최종 EMERGENCY_STOP/BLOCKED. 위치 초기화 성공과 주행 재개 성공을 구분한다. |
| `scoped_tests.txt`, `build_final.txt` | 당시 선택 회귀 336 passed, 관련 5개 패키지 빌드 성공. |

핵심 로그 해시와 추출 값은 `nov13-evidence.json`에 보존했다.
오늘의 70초 검증 실패를 이유로 위 기능을 삭제하거나 이전 코드로 교체하지 않았다.

## 재개한 개발과 검증

생산 코드·기존 43개를 보존하고, 현재 구현을 검증하지 못하던 시험 코드를 수정했다.

1. `test/offline_static_avoidance.py`: 현재 Control/S4 rollout이 공유하는
   `path_lookahead_distance`와 생산 조향 변화율 제한 함수를 사용한다.
   0.6rad/s 명령 제한은 지연 큐 **앞**에 적용하여 실제 명령 계산 순서를 맞췄다.
   시험별 설정을 명시할 수 있게 하되 `AvoidanceConfig` 검증을 그대로 사용한다.
2. `test/synthetic_s4_loop.py`: 경로 초기 해시, Control 기본 파라미터와 조향 이력,
   전체 return rollout의 `duration_s` 전달을 복구했다. 실제 Control 루프/PID/PP를 호출한다.
   원래 차선의 새 장애물은 안전한 복귀 대기가 가능하며, 두 차선을 막는 장애물은
   정지해야 하므로 두 주입 장면을 구분했다.
3. 오래된 회귀 계약을 현재 코드와 일치시켰다. 얇은 LiDAR 물체의 보수적 5cm footprint,
   DETECT와 후보 검색의 분리, Route Editor의 원자적 `route_start_link_id` 적용,
   실제 LocalPathPlanner 반환 구조를 확인한다. Cache 검증은 실제 차체 여유보다
   큰 **유효한** obstacle/hold margin을 사용한다.
4. `test/test_static_obstacle_closed_loop.py`: 현재 S4 launch의 0.6rad/s 제한을 사용하는
   성공 사례와 제한을 사용하지 않을 때의 보수적 STEERING_RATE 거부를 분리했다.
   현재 80m preview를 과거 20m preview의 NO_SAFE_ENTRY 기대값으로 판정하지 않는다.
5. `test/test_s4_resume_profiles.py`: 15→10km/h 좌우 회피/복귀를 직선과 반경 300m에서
   검증한다. 50/25/15km/h 프로파일, 커브 전 제동 거리, 횡가속도 제한,
   독립적인 15km/h admission도 검사한다.

오프라인 15→10 좌측 직선 모델의 대표 결과는 최대 추종 오차 0.063669m,
최소 장애물 여유 2.01973m, 최대 조향 변화율 0.334314rad/s다.
`offline-15-to-10-left.json`에 저장했다. 이 모델의 CPU 시간이나 가상 제동을
실제 MORAI 처리 지연·브레이크 성능의 검증으로 사용하지 않는다.

속도 50/25/15/10은 해당 분류의 설정 상한이다. 기존 횡가속도 1.5m/s² 제한에 따라
예를 들어 곡률 0.05/m에서는 약 19.72km/h, 0.12/m에서는 약 12.73km/h까지
추천 속도가 낮아질 수 있다. 이 제한을 완화하지 않았다.

## 전체 저장 경로의 오프라인 검사

18 waypoint, 45 link, 2444.075m의 기존 Dijkstra 미션을 변경하지 않고 2m 간격으로
1,218개 위치를 검사했다. 마지막 10m는 goal 접근 처리를 별도로 보기 위해 제외했다.
각 위치에서 80m Local Path, 속도 프로파일, PP 입력을 평가했다.

- Local Path/속도/PP 무효 입력: 각각 0개
- 최대 제한 projection 오차: 0.011326m
- 추천 속도 범위: 12.213~50km/h, admission 후 범위: 7.317~15km/h
- 지도 접선 자세에서 PP 요청이 현재 0.70 조향 상한을 넘는 위치: **11개**
- 미션 경로 및 저장 route 파일의 검사 전후 해시 동일

상세: `saved-route-profile-sweep.json`.
이는 지도상의 고정 자세 검사이며 실제 차량 궤적이나 승속 승인 자료가 아니다.
11개 지점은 후속 곡선 추종 검증 대상으로 남긴다. 같은 검사의 mode는 CRUISE/CURVE였으므로
급커브 전환의 실제 미션 주행 검증까지 완료했다고 보고하지 않는다.

## 실행 환경과 후속 작업

실제 도메인 0에서는 구독과 GetParameters 조회만 수행했다.
검증 프로세스는 별도 ROS_DOMAIN_ID 183~195 및 ROS_LOCALHOST_ONLY=1을 사용했다.
테스트용 생산 노드가 필요한 ROS 회귀도 해당 격리 도메인에서만 실행했다.

현재 안전 정지의 근거는 `current-readonly-state.json` 및 현재 planning 로그다.
시작 시 위치추정 미준비 이후 `S4_SENSOR_RECOVERY_TIMEOUT`이 latch되어 있다.
센서 READY 상태와 latch 해제는 별개다. 관측 당시 `enable_drive=false`,
EMERGENCY_STOP, 목표 정지, accel 0, brake 0.8 및 실제 속도 약 0을 확인했다.
실제 스택·Route Editor·Planning Visualization·RViz를 재실행하거나 설정 변경하지 않았다.

남은 순서는 다음과 같다.

1. 기존 안전 절차에 따라 현재 센서 latch의 원인을 정리하고 정지 상태의 복구 조건을 확인한다.
   이번 작업에서 fault reset, 위치 재설정, 주행 활성화는 실행하지 않았다.
2. 저장 경로의 11개 조향 검토 위치를 실제 차량 자세·차선 여유와 대조한다.
   이 기하 검사만으로 글로벌 경로를 편집하거나 Control gain을 변경하지 않는다.
3. 안전 조건이 충족된 뒤 15km/h 기준의 반복 회피·복귀, 커브/급커브 전환 및
   선행 감속을 실제 MORAI에서 검증한다.
4. 유효 LiDAR 표적 탐지거리·누락·지연과 실제 감속/정지거리를 확보한 다음에만
   단계별 승속을 판단한다. 현재 검증 상한 15km/h와 검증 거리 20m는 유지했다.

최종 회귀 집계와 파일 무결성은 동일 증거 디렉터리의 `validation-summary.json`,
`integrity-final.json`, `current-readonly-state-final.json`을 참조한다.
