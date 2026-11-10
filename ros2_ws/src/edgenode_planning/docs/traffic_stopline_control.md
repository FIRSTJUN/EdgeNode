# K-City route 정지선 + ego ROI fallback

현재 설정은 `route_node_ids: [A119BS010245, A119BS010208]`,
`enable_drive: false`, `traffic_light_enabled: false`이다. Planning config는
설치 패키지의 `config/traffic_light_associations.kcity.json`을 읽는다.
`.unverified.json`과 example 파일은 승인 상태를 변경하지 않았다.

## 지도 근거와 정지선 순서

MGeo의 `global_info.json`에 기록된 원본은 `HDMap_UTM52N_타원체고`,
EPSG:32652, 원점 `[302459.942, 4122635.537, 0]`이다.
MGeo subset은 link/node만 포함한다. 같은 원본의 B2_SURFACELINEMARK
Kind530 횡교차 형상과 C1_TRAFFICLIGHT의 ID/LinkID/Ref_Lane을 대조했다.
생성기는 A2_LINK shp/dbf/prj 해시를 MGeo source_sha256과 먼저 비교한다.
UTMK 형상을 이 원점과 혼용하지 않는다.

기준 route: A219BS010380에서 두 waypoint를 순서대로 방문한 25 links,
725 XYZ points, 길이 894.122589m,
SHA256 `c5001ff4202508387ae78abc283b4360f37a56a1b37193f70f2147c3f0143be6`.
아래 arc는 이 기준 route의 시작점부터 계산했다. runtime에서는 실제 새 route의
교차 arc를 다시 계산하므로 기준 arc를 그대로 대입하지 않는다.

| 정지선 후보 ID | 기준 arc m | 접근 link | C1 신호 후보 끝 3자리 |
|---|---:|---|---|
| B219BS010007 | 136.206 | A219BS010380 | 없음 |
| B219BS010135 | 224.604 | A219BS010385 | 없음 |
| B219BS010005 | 264.068 | A219BS010383 | 003, 005, 006 |
| B219BS010012 | 315.213 | A219BS010382 | 없음 |
| B219BS010142 | 360.151 | A219BS010398 | 없음 |
| B219BS010140 | 403.960 | A219BS010395 | 041, 042, 045, 046, 050 |
| B219BS010139 | 501.196 | A219BS010393 | 017, 020 |
| B219BS010024 | 630.752 | A219BS010404 | 없음 |
| B219BS010144 | 688.233 | A219BS010408 | 없음 |
| B219BS010018 | 736.294 | A219BS010419 | 027, 028, 030, 031 |
| B219BS010013 | 874.021 | A219BS010429 | 033 |

C1 접두어는 `C119BS010`이다. B...005 접근 link의 다음 link는
A219BS010649이며 C...003/005/006의 LinkID도 이것이다. 이것은 위상상
후보 근거이지 어느 영상 bbox가 어느 신호인지 확정한 연결이 아니다.
B2 후보의 직접 R_linkID/L_linkID는 비어 있고 C1에는 facing/yaw가 없다.
파일에는 실제 정지선/신호 후보/link/node/형상/방향/원본 해시를 보존했다.
종전 strongest candidate의 우회전 약89.63도도 직진 검증으로 바꾸지 않았다.

## fallback 범위와 한계

파일은 `mode: ROUTE_STOPLINE_FALLBACK`, `fallback_enabled: true`,
`verified: false`이다. map signal ID를 만들어 넣거나 verified binding으로
위장하지 않는다. 구성된 waypoint 순서, 실제 route node/goal/link 포함,
횡교차 및 기준 진행 방향과 45도 이내인 기하만 허용한다. 이미 지난 route
교차는 제외하고 가장 가까운 다음 교차 하나만 선택한다. 반복 도로는 ID와
실제 route arc의 조합으로 구분한다. route가 바뀌면 상태와 arc를 재계산한다.

전방 ROI는 640×480 `Camera-1` 영상에서 정규화 `[0.45, 0, 0.70, 0.45]`이다.
기존 관측 bbox envelope x0.536~0.667/y0~0.038를 포함하는 중앙 상부 영역이다.
bbox **중심**이 ROI 안에 있고 confidence>=0.25여야 한다. 숫자 ROI/confidence 값은
유지했다. ROI 경계에 걸친 큰 신호도 중심이 안에 있으면 후보가 된다.

복수 후보는 일괄 제외하지 않고 `traffic_light_candidates.py`에서 점수를 비교한다.
기본 가중치는 예상 수평 위치0.40, 상부 영역0.10, 크기0.08, confidence0.12,
이전 bbox 연속성0.26, 이전 상태 지속0.04이다. 최초 프레임은 연속성 항목을
제외하고 정규화한다. 예상 수평 위치는 영상 중심을 기본값으로 쓰며, route의
정지선 접근 방향과 ego yaw의 차이를 nominal FOV90도 기준 보조 힌트로 사용한다.
이는 카메라 외부보정이나 실제 신호 객체 투영이 아니다.

같은 head의 중복 검출(IoU>=0.65, 동일 class)은 합친다. 이전 대상과의 IoU 또는
영상 폭/높이로 정규화한 중심 이동으로 연속성을 계산한다. 0.08 초과 중심 점프와
낮은 IoU 후보는 감점하고, 기존 대상과 연결되는 후보가 전혀 없으면 0.5초 track
만료 전에는 `TARGET_JUMP_REJECTED`로 새 대상을 바로 채택하지 않는다.
기존 후보 유지 bonus는 보통0.04, 정지 거리/여유 근처에서는0.16이다. 점수는
이 bonus 때문에1을 넘을 수도 있다. 1위-2위 최종 점수차가0.06 미만일 때만
`AMBIGUOUS_CANDIDATE_SCORES`로 처리한다. 동점에서 임의 순서로 고르지 않는다.

새 bbox target으로 전환하면 RED/GREEN 확인 횟수를 초기화해 서로 다른 신호의
증거를 합치지 않는다. 같은 camera stamp를 반복 처리해도 track 수명이나
확인 횟수를 늘리지 않는다. 기존 RED3frame/0.3초, UNKNOWN hold0.8초는 유지한다.

**이 ROI는 명시적인 heuristic이며 신호 소유권을 물리적으로 검증한 결과가 아니다.**
ROI 안에 다른 차선의 단일 신호만 있으면 이를 완벽히 구분할 수 없다. Kind530
형상의 실제 정지선 동일성, 우회전/화살표 허가, 주행 중 ROI, 카메라 보정,
제동 성능은 실제 화면과 감독하의 시험에서 확인해야 한다. 모델은 색상만
구분하므로 GREEN을 회전 허가의 증명으로 사용하지 않는다.

## 입력과 정지/해제

실제 입력은 `/perception/traffic_light/detections`의 String JSON이다:

```json
{"state":"RED","stamp":{"sec":1,"nanosec":0},"frame_id":"Camera-1","image_width":640,"image_height":480,"prediction_age_sec":0.02,"predictions":[{"label":"red","confidence":0.9,"x":360,"y":15,"w":20,"h":20}]}
```

x/y는 bbox 중심, w/h는 너비/높이이다. 어댑터가 xyxy로 바꾸고 원래 영상
timestamp를 보존한다. `/perception/traffic_light/state`는 원시 상태 진단용이다.
이 state만으로 전역 제동하지 않는다. 잘못된 underscore detection 토픽은
더 이상 구독하지 않는다. stamp/수신 나이가 0.5초를 넘으면 UNKNOWN이다.

- RED: 서로 다른 source frame 3개 이상, 연속 유지 0.3초 이상 후 적용한다.
- GREEN: 같은 ROI에서 3 frame/0.3초 확인 후 `GREEN_RELEASE`로 해제한다.
- YELLOW/UNKNOWN: 새 정지를 만들지 않는다. 기존 확정 RED는 마지막 RED로부터
  최대 0.8초 유지한다. 영상 단절은 영구 정지 latch가 아니다.
- association/route/pose 오류: 신호 cap을 적용하지 않고 구체적 reason을 출력한다.
- 진행률이 이미 선을 지난 경우 `ROUTE_STOPLINES_PASSED` 또는 다음 정지선을 선택한다.

정지 정책은 기존 Planning 값에서 복사했다: 감속 1.5m/s², 반응 1초,
접근 범위 100m, 기준점 정지 여유 5m (`corner_braking_buffer_m`). 측정하지 않은
차량 전면 오프셋은 0으로 표기하고 `front_offset_measured=false`를 출력한다.
즉 5m는 기준점부터의 **전면을 포함한 여유**로 사용하며 별도 차량 길이를 더하지 않는다.
실차 전면 위치/정지 정확도는 검증되지 않았다. cap은
`d = tau*v + v²/(2a)`로 계산하고 기존 Planning 속도와 최솟값을 사용한다.
먼 거리에서 목표0을 만들지 않으며 정지 여유 위치에서 `TRAFFIC_STOP`/목표0을
사용한다. Control/PID/조향/최대속도는 바꾸지 않았다.

## 실행 노드와 정지 상태 확인

`autonomy.launch.py`는 localization/planning/control 각 1개를 실행한다.
**기존 수동 실행 프로세스와 launch를 동시에 띄우지 않는다.** 기존 세트를
운영자가 정리하고 기존 launch 명령으로 한 세트만 재시작해야 새 코드·YAML이
로드된다. 이번 변경은 실행 중인 기존 노드의 파라미터/프로세스를 바꾸지 않는다.
같은 이름의 노드가 2개라면 파라미터 응답과 목표 속도가 혼재할 수 있다.

```bash
ros2 node list
ros2 param get /planning_node enable_drive
ros2 param get /planning_node traffic_light_associations_file
ros2 param get /planning_node route_node_ids
ros2 topic info /perception/traffic_light/detections -v
ros2 topic echo /planning/traffic_light_gate
```

`enable_drive=false`를 유지한 채 사용자가 아래로 신호 기능만 켜면 gate는
진단을 계산한다. FSM은 주행을 허용하지 않는다.

```bash
ros2 param set /planning_node traffic_light_enabled true
```

확인할 로그:

```text
traffic=RED stopline=B219BS010005 distance=... association=ROUTE_STOPLINE_FALLBACK stop_requested=true target_speed_before=... target_speed_after=... reason=RED_ACTIVE_STOPLINE gate_state=RED evidence=3
traffic=GREEN stopline=B219BS010005 ... stop_requested=false ... reason=GREEN_RELEASE
traffic=RED stopline=none ... stop_requested=false ... reason=ROUTE_STOPLINES_PASSED
```

JSON에는 후보별 center/class/confidence/score/components/rejection, selected_candidate,
selected_score/second_best_score/ambiguity_margin/ambiguous/selected_track_id와
동일 로그 필드 및 evidence_count/evidence_reason, fallback_used,
map_signal_candidates, 거리/정지 여유/원시 상태/ROI 상태/최종 target을 제공한다.
주행 해제 상태에서는 target 전후가0이어도 cap/stop_requested를 확인할 수 있다.

이번 읽기 전용 수집 시 진행률은894.122589m로 모든 정지선이 뒤에 있었다.
그 상태에서 RED라도 뒤쪽 선에 소급 제동하지 않는 것이 정상이다. 운영자가
주행 해제 상태에서 시험 위치를 정지선 충분히 전방으로 재배치하고 경로를
재생성하여 올바른 진행률/active stopline/거리를 먼저 확인해야 한다.

이후 감독자가 MORAI 일시정지/수동 제어를 준비한 환경에서 저속 시험을 별도로
결정한다. RED 안정화→감속→선 전 정지→안정 GREEN 해제를 확인한다.
다른 차선/뒤쪽 RED, 순간 RED, UNKNOWN, 복수 신호도 확인한다. 실제 50km/h
정지나 ROI 소유권은 unit test 통과만으로 보장되지 않는다.
이 문서와 자동 검증은 enable_drive=true 또는 차량 명령을 실행하지 않는다.

## 재생성

`tools/perception/build_route_stopline_fallback.py`는 실제 route JSON(points,
link_ids, node_ids)과 MGeo/원본 지도로 새 출력 파일을 생성한다. 기존 파일을
덮어쓰지 않으며 A2 지도 원본 해시를 비교한다. 기준 route 정보는 runtime
파일의 reference_*에 보존되지만 실제 arc는 runtime route에서 재계산한다.
