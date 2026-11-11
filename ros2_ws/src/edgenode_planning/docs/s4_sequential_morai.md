# S4 단계별 MORAI 실험 결과

2026-11-12 KST, `s4-finish`. 커밋·푸시 없음.

사용자가 K-City에 배치한 정지 차량 한 대를 사용했다. 장애물은 유지하고 시작 위치를 연결된 동일 방향 차선에서 약 7 m 뒤로 조정했다. 실험 중 제어 발행자는 한 개였다.

실제 실행 순서: `NORMAL → DETECT → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL`.

| 상태 | 주행 활성화 후 시간(s) |
|---|---:|
| NORMAL | 0.000 |
| DETECT (약값) | 5.041 |
| AVOID_ENTRY | 5.422 |
| AVOID_HOLD | 31.386 |
| RETURN | 62.989 |
| NORMAL | 81.187 |

진입 커밋 1회, 복귀 커밋 1회. 복귀 전 저속 유지 시간과 차체 안전거리 확인을 통과한 뒤 STEP 4에서만 복귀 경로를 생성했다.
측정 최고 속도 3.30 km/h. 복귀 후 정상 추종을 재개하고 정지했다. 최종 측정 속도 0.000004 km/h, 가속 0.0, 브레이크 0.65.

MORAI 외 실차 실험은 수행하지 않았다. 이 결과는 현재 같은 방향 다차선 장면의 실행 확인이다.

검증: Python 문법 통과, 선택한 핵심 테스트 37개와 단계 표시 검사 1개 통과. 이전 대규모 회귀 데이터는 재실행하지 않았다. ROS2 빌드 및 실행 과정에서 남은 노드가 없는 것을 확인했다.

로그: `/workspace/logs/s4-sequential/summary.json`, `trial_result.json`, `trial_telemetry.jsonl`, `trial_launch.log`.

재실행 방법과 조정 파라미터는 [static_obstacle_avoidance.md](static_obstacle_avoidance.md)를 참고한다.

DETECT는 기존 안전 대기 표시 `VALIDATING`에 가려졌지만, 검출 footprint와 진입 커밋 0·경로 생성 0인 debug 프레임으로 확인했다. 이후 표시도 보완하여 `/planning/avoidance_stage`와 debug의 `stage`, RViz·터미널에 실제 실행 단계를 유지한다.
