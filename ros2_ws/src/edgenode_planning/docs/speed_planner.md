# 속도 계획 — NORMAL / SHARP_CURVE

2026-11-15부터 직선과 일반 커브는 `NORMAL`, 급커브는 `SHARP_CURVE` 두 모드다.
기본 NORMAL 목표는 60km/h(`cruise_speed_kmh`), 급커브는 30km/h
(`sharp_curve_speed_kmh`)다. 70/40 등으로 두 파라미터만 변경할 수 있다.
S4 회피·복귀는 별도 `avoidance_avoid_speed_kmh`(Launch `avoidance_speed_kmh`) 10km/h다.

곡률은 XY triplet → 거리 가중 median → 연속 거리 persistence로 판단한다.
실제 짧은 커브를 놓치지 않도록 anticipatory 모드에서는 동일 방향 회전의 거리 평균도 사용한다.
급커브 진입 임계값 0.10/m, 해제 0.095/m이며 일반 커브는 별도 저속 모드로 전환하지 않는다.
지도상의 작은 점 돌출과 취소되는 좌우 회전은 급커브 판정에서 제외한다.

선행 제동은 설정된 급커브 목표에 도달하기 위한 거리·반응 시간 모델만 사용한다.
자동 횡가속도 속도 상한과 조향량에 비례하는 연속 속도 상한은 적용하지 않는다.
측정 조향이 급커브 범위에 남으면 설정된 급커브 목표를 유지하며 그보다 낮은 속도를 만들지 않는다.
이는 물리적인 추종 성공을 보장하지 않으므로 실제 성능은 주행 결과와 별도로 기록한다.
S4 경로의 횡가속도·곡률·차체 충돌·PP rollout 적합성 검사와 경로 불성립 시 정지는 유지한다.

Planning 명령은 FSM이 주행을 허용할 때 위 목표를 사용하고, S4가 활성화되면
S4 목표를 적용한다. `enable_drive=false`, 센서 고장, 무효 경로, session 만료와
충돌·기하 거부는 정지 명령으로 이어진다.
`max_drive_speed_kmh`, `morai_speed_admission_enabled`, 검증 속도 상한,
`stopping_speed_cap`의 운행 명령 연결과 독립 admission 단계는 제거했다.
과거 API 호출용 `curve_speed_kmh` 생성자 인자는 호환을 위해 받지만 계산에는 사용하지 않는다.
ROS 파라미터로는 선언하지 않는다.

현재 실행·목표 변경·상태·정지는 [수동 MORAI 매뉴얼](../../../../docs/morai_manual.md)을 참고한다.
