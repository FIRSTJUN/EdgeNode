# S4 지도 사용

별도 도로 경계 파일 로더·승인 해시·차체 포함 검사와 해당 테스트는 삭제했다. 추가 경계 데이터가 실행 조건이 아니다.

기존 MGeo는 현재 링크, 바로 인접한 링크, 방향, 연결성, 차선 변경 허가 및 실제 중심선 좌표 확인에 사용한다. 진입은 진입 방향만 검사하고, 복귀 방향과 합류 구간은 STEP 4에서 검사한다.

`avoidance_require_explicit_permission`은 기존 엄격한 메타데이터 검사 모드 선택용이다. MORAI 실험 launch는 방향별 인접 링크를 사용하고 명시적 금지는 적용한다. 현재 실행 방법은 [static_obstacle_avoidance.md](static_obstacle_avoidance.md)를 참고한다. 이전 지도 조사 기록은 현재 FSM의 필수 실행 조건이 아니다.
