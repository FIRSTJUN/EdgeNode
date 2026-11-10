# K-City curvature speed classification

장거리 선행 감속은 별도 [Longitudinal Speed Planner](longitudinal_speed_planner.md)가
담당한다. 기본 비활성이며 이 문서의 Local 검출/히스테리시스 의미는 유지한다.
거리·횡가속도 상한과 상승 제한은 기존 추천속도를 높이지 않고 최종 추천에 합성된다.

`SpeedPlanner`는 ROS와 독립적으로 XY 경로에서 추천속도를 계산한다. 기존 세 점 곡률,
distance median, persistence와 속도/임계값은 유지하고 heading 보완과 공간 상태를 추가한다.
경로 좌표, Pure Pursuit, PID, Steering Limit, 기존 Local 설정 기본값은 유지한다.
이번 오프라인 비교는 30/15/8 km/h, 임계값 0.025/0.07, hysteresis 0.005 기준이다.
실제 적용값은 runtime parameter를 확인해야 하며 이 프로파일의 실주행 안전성을 보장하지 않는다.

## 검출과 품질 검사

기존과 동일하게 전체 XYZ를 검증하고 중복 XY를 제거한 뒤 Preview까지 자른다.
Raw 세 점 곡률, max/p80, signed distance median과 연속 거리 persistence는 그대로 계산한다.
MGeo의 몇몇 꼭짓점에 회전량이 몰려 기존 대표값이 0이 되는 경우를 heading으로 보완한다.

Heading은 물리 거리 0.25m grid에서 측정한다. 제어 경로를 resample하는 작업이 아니다.
가장 긴 가용 대칭 tangent 관측 구간(최대 8m, 최소 4m)의 순 회전량/거리를 사용한다.
큰 구간에 역회전이 섞이면 4m 구간으로 S커브의 한쪽 lobe를 평가한다.
순/절대 회전량 비율 >=0.9, 누적각 >=8도, 회전량 10~90% 분포 거리 >=2m,
서로 다른 1m bin 두 곳 이상에서 각각 >=1도 조건을 요구한다.
4m 미만의 짧은 끝부분에는 heading 근거를 만들지 않으며 기존 유효 짧은 커브는 유지한다.

고립 돌출은 10m 이내에서 같은 선/방향으로 돌아오고 양쪽의 안정된 tangent가 관측되는
짧은 excursion으로 확인한다. 누적 회전량 >=60도와 돌출/잦은 방향 반전도 요구한다.
충분히 분포한 실제 turn lobe가 있으면 U턴/넓은 S커브로 보고 제거하지 않는다.
확인된 artifact 구간만 기존 곡률 support에서 제외한다. 다른 위치의 정상 커브와 Raw Max는
보존하며 경로 자체를 이동/삭제하지 않는다.

**관측 범위:** 커브 검출은 설정 Preview와 실제 가용 길이 중 작은 범위이며 heading 보완은
최대 20m다. 품질 검사는 실제 Local Path의 최대 20m를 사용할 수 있어 Preview 10/15m
바깥의 반환 geometry도 확인할 수 있다. 이는 잡음을 확인하는 용도이며 Preview 밖의
새 커브를 감지해 선행 감속하는 용도가 아니다. 입력 밖으로 연장하거나 padding하지 않는다.
`heading_evaluated_length_m`와 `quality_context_length_m`로 두 범위를 구분한다.

Geometry만으로 모든 경로 품질 문제와 진짜 도로를 구분할 수는 없다. 알려진 돌출 검사
통과를 임의의 실제 경로 돌출을 고속으로 무시해도 된다는 뜻으로 해석하지 않는다.

## 진입 지연 없는 공간 상태 유지

기본 판정은 품질 검사 후 기존 대표값과 heading 보완값 중 큰 값이다. Heading이 기존보다
더 낮은 속도 Mode를 요구한 커브만 좌표 anchor로 추적한다. 실제 전방 경로에 anchor가
남는 동안 확인한 강도를 유지해 CURVE/SHARP 왕복을 줄인다. 상향 감속은 즉시 반영하며
시간 debounce나 큰 hysteresis 증가를 사용하지 않는다. Anchor가 뒤로 지나가거나 가용
Preview의 남은 회전량이 6도 미만이면 해제한다. 같은 방향의 겹치는 근거만 합친다.

Planner 내부에 숨은 이력은 없다. 호출자가 `curve_state`를 다음 호출의
`previous_curve_state`로 전달한다. PlanningNode가 이를 담당하며 경로 교체, 입력 실패,
런타임 설정 교체 시 Mode와 curve state를 함께 초기화한다. Route progress/drive lease는
변경하지 않는다. 이전 Mode만 전달하는 기존 호출도 유효하지만 공간 유지 효과는 없다.

```python
result = planner.plan(points, previous_speed_mode=previous_mode,
                      previous_curve_state=previous_curve_state)
previous_mode = result['speed_mode'] if result else None
previous_curve_state = result['curve_state'] if result else None
```

## 진단값

| 반환값 | 의미 |
| --- | --- |
| `max_curvature`, `raw_representative_curvature` | 기존 Raw Max/p80, 변경 없음 |
| `filtered_max_curvature` | Median 이후 max, 품질 마스크 이전 |
| `legacy_representative_curvature` | 기존 median+persistence 대표값 |
| `quality_filtered_curvature` | Artifact support를 제외한 기존 대표값 |
| `heading_curvature` | 현재 Preview의 heading 보완값 |
| `representative_curvature` | 보완/품질/공간 상태 floor를 포함한 최종 판정값 |
| `quality_rejected_samples` | 제외한 곡률 sample 수 |
| `heading_evaluated_length_m` | 실제 heading 관측 길이, 최대 20m |
| `quality_context_length_m` | 사용한 품질 관측 길이, 최대 가용 20m; 미수행 0 |
| `curve_state` | 다음 호출에 전달할 제한된 anchor/방향/강도 목록 |

`/planning/speed_curvature`는 최종 판정값, `/planning/speed_max_curvature`는 Raw Max다.
`recommended_speed`는 추천값이며 `target_speed`는 기존 drive/FSM gate와 상한을 거친다.
새 ROS 토픽이나 주행 시나리오는 추가하지 않는다. 9개 runtime 설정의 원자적 갱신은
[런타임 파라미터 문서](speed_planner_runtime_parameters.md)를 따른다.

## 오프라인 검증과 주행 제한

| 20m Preview, 동일 bag | 808f640 | 이전 Heading 연구 | 이번 공간 결합 |
| --- | ---: | ---: | ---: |
| 46~52초 감지 /58 | 0 | 29 | 34 |
| 첫 판단 s | 없음 | 46.234 | 46.234 |
| 전체 Mode 전환 | 40 | 73 | 48 |
| 1초 이하 감속 구간 | 12 | 8 | 8 |
| 1초 이하 CURVE/SHARP 왕복 | 0 | 14 | 0 |
| 보수적 직선 선별의 추가 감속 | 0 | 0 | 0 |

직선 선별은 전체 20m의 절대 heading 변화 합 <=5도인 1,315개 cycle이며 영상 정답이 아니다.
이전 추가 감속 515개 cycle의 19개 구간을 원본 MGeo와 대조했다. 큰 회전과 완만한 커브,
경계 부근의 짧은 감지가 섞여 있으므로 전부 오탐 또는 성공으로 자동 판정하지 않는다.
이번 추가 감속 분류는 570개로 늘었고, 약한 단발 감지가 약 .879초 유지되는 사례도 남았다.
완만한 구간까지 포함해 총 방향 변화 <=10도로 선별하면 1,868개 중 감속 분류는
기준선/이전 연구/이번 구현 각각 1/1/14개다. <=15도에서는 2,073개 중 6/14/41개다.
이는 영상 기반 직선 정답이 아니지만, 상태 유지에 따른 정상 구간 영향 검증이 필요한 이유다.

20m에서 감지 당시 속도는 약 31.63km/h, Ego 기준 커브까지 약 14.67m다. 8km/h까지
감속하는 데 지연 .3s/감속도 3m/s²를 가정하면 약 14.68m로 여유가 거의 없고, 2m/s²면
부족하다. Preview 숫자를 실제 Local Path보다 크게 쓰는 것만으로 해결되지 않는다.
최대속도 상승을 권고하지 않는다. 실제 MORAI 주행은 수행하지 않았으며 낮은 속도에서
감속 응답, 경로 오차, 조향 포화와 정상 구간 영향을 별도로 확인해야 한다.
