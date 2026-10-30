# Curvature speed classification

기존 속도값과 진입 임계값을 유지한다. K-City 설정은 CRUISE 10 km/h,
CURVE 5 km/h, SHARP_CURVE 3.5 km/h이며 진입 임계값은 0.04 / 0.10 (1/m)이다.
경로 좌표, Local Path, 조향, PID는 이 처리에서 변경하지 않는다.

기존 판정은 raw 최댓값이 아니라 **point 개수 기준 80백분위**였다. 짧은 연결부에
point가 몰리거나 남은 preview가 짧아지면 높은 곡률 sample이 판정을 지배할 수
있었다. 실제 K-City 지도 25,341개 segment의 간격 중앙값은 약 0.5 m였다.
이는 오프라인 지도 분석이며 사용자 MORAI 감속 지점의 실주행 재현은 아니다.

## 공간 필터와 지속성

1. 기존과 같이 중복 XY를 제거하고 10 m preview까지 곡률을 계산한다.
2. 좌우 방향 부호를 보존한 곡률에 거리 가중 median을 적용한다. 기본 window는
   2 m이며, 각 sample의 가중치는 인접 sample 사이 거리의 중간점으로 정한
   담당 구간 길이다. 첫/마지막 sample은 preview 경계까지 담당한다.
3. 필터 결과의 절댓값이 **연속 2 m 이상** 유지되는 최대 곡률을 판정값으로
   사용한다. 떨어져 있는 짧은 spike 구간은 합산하지 않는다.

차량 경로나 point 위치는 smoothing하지 않는다. 처리 대상은 곡률 값뿐이다.
Preview가 2 m보다 짧으면 남은 전체 거리에서 지속되는 값을 사용한다. 따라서
짧아진 실제 커브도 감속할 수 있고, 2 m 조건 때문에 자동으로 CRUISE가 되지 않는다.
유효한 triplet이 없는 입력은 기존과 같이 None을 반환해 Node에서 정지 gate를 유지한다.

## Hysteresis

진입 임계값은 그대로 두고 해제 임계값만 0.005 (1/m) 낮춘다.

| 전이 | 기본 기준 |
|---|---|
| CRUISE → CURVE | 판정 곡률 >= 0.04 |
| CURVE → CRUISE | 판정 곡률 < 0.035 |
| CURVE → SHARP_CURVE | 판정 곡률 >= 0.10 |
| SHARP_CURVE → CURVE | 판정 곡률 < 0.095 |

SHARP_CURVE에서 값이 0.035 미만까지 내려가면 바로 CRUISE로 해제된다.
`SpeedPlanner.plan(points, previous_speed_mode=None)`은 이전 mode를 선택 입력으로
받으며 내부 상태를 저장하지 않는다. Planning Node가 이전 mode를 기억하고,
경로 변경 또는 유효한 속도 판정이 없는 cycle에는 초기화한다.

## 설정

`planning.yaml`의 다음 값은 Node 시작 시 SpeedPlanner에 전달한다. 변경 후에는
Planning Node를 재시작해야 한다. 기존 파라미터/속도 정책의 초기화 방식과 같다.

| ROS 파라미터 | 기본값 | 단위 |
|---|---|---|
| `speed_curvature_smoothing_window_m` | 2.0 | m |
| `speed_curvature_persistence_m` | 2.0 | m |
| `speed_curvature_hysteresis` | 0.005 | 1/m |

공간 window와 지속 거리는 양수여야 한다. hysteresis는 0 이상이며 0이면 해제
임계값과 진입 임계값이 같아진다. 잘못된 값은 기존과 같이 안전하게 판정을 거절한다.

## 진단값의 의미

| 토픽 | 의미 |
|---|---|
| `/planning/speed_max_curvature` | 기존과 동일한 raw 절대 곡률 최댓값 |
| `/planning/speed_curvature` | **변경:** raw p80 대신 smoothing·연속 거리 조건을 통과한 판정 곡률 |
| `/planning/speed_mode` | 위 판정값과 hysteresis로 정한 mode |
| `/planning/recommended_speed` | mode의 추천 속도 |
| `/planning/target_speed` | 기존 drive/FSM gate와 상한을 거친 실제 목표 속도 |

추가 ROS 토픽은 없다. Python 반환값에는 `raw_representative_curvature`(기존 p80),
`filtered_max_curvature`(median 이후 최댓값)를 추가해 unit/offline 분석에 사용한다.
기존 raw 곡률 수식과 raw max는 유지하므로 raw spike 자체가 진단에서 사라지지 않는다.
Hysteresis 유지 구간에서는 곡률이 진입 임계값보다 조금 낮아도 CURVE가 유지될 수 있다.

## 사용자 MORAI 확인

기본 검증 기준은 10 km/h / 60초이다. 직선의 짧은 연결부에서는 raw max가 높아도
판정 곡률이 낮으면 CRUISE 10 km/h를 유지해야 한다. 실제 지속 커브에서는
CURVE 5 km/h, 지속 급커브에서는 SHARP_CURVE 3.5 km/h를 확인한다.
경계값 근처의 작은 변동에 mode가 반복 전환되거나, 실제 긴 커브에서 CRUISE가
유지되면 정상으로 간주하지 않는다. 코드/unit test 결과와 실주행 결과를 구분한다.
