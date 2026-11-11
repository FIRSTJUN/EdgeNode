> 과거 구현의 실험 기록입니다. 현재 단계형 FSM과 실행 방법은 `static_obstacle_avoidance.md`를 따릅니다. 별도 도로 경계 검사는 삭제되었습니다.

# S4 Stage 5C 계산시간과 센서 신선도 검증

검사일 2026-11-09 UTC. 브랜치 `s4-finish`, 기준 HEAD
`4c80691aec4d5a64a8567c762cff008648fc3c85`. commit/push를 하지 않았다.

**계산시간 개선과 기존 동작 보존은 확인했다. 실제 시간의 곡선 폐루프 및 ROS 주기
신선도 문제는 해결 완료로 판정하지 않는다.** 0.6초 제한을 유지했다.
실제 MORAI, 실제 K-City 과거 도로 경계 검사, 실제 LiDAR 관측 승인은 이번 결과에 포함되지 않는다.

| 구분 | 판정과 근거 |
|---|---|
| A. 계산시간 | 개선. 실제 Planning 단발 검색 wall p95: 직선 0.483→0.285초, 곡선 좌 1.083→0.428초, 우 1.331→0.484초 |
| B. 종료 시 신선도 | 새 입력을 매번 전달한 단발 콜백 69/69 통과. 연속 20Hz 실행기는 10회 중 5회 stale 래치 발생. 전체 주기의 충족 보장은 실패 |
| C. 폐루프 | 기존 가상 계산 지연의 44/44 기대 결과와 궤적 보존. 실제 core wall 지연을 적용한 3km/h 8건은 직선 4건 복귀, 곡선 4건 STEERING_RATE 정지. 실제 시간의 곡선 복귀 성공으로 보고하지 않음 |

원시 결과와 이 문서의 실행 사본은
[`logs/s4_stage5c_20261109/`](/workspace/logs/s4_stage5c_20261109)에 있다.
로그 폴더는 기존 Git ignore 대상이다. 아래 링크의 JSON/CSV/PNG를 함께 보존해야 한다.

**1. 변경 범위와 기존 코드 보존**

운영 변경은 `avoidance_geometry.py`, `lane_geometry.py`, `lattice_path.py`,
`static_obstacle_planner.py` 네 파일이다. 추가 내용은 시간 계측·비교 도구,
합성 하네스의 별도 measured 모드, 기준 결과 fixture, 신규 테스트와 이 문서다.

- 과거 도로 경계 검사의 불변 convex cell에 16m 격자 인덱스를 붙였다. 기존 AABB 검사에
  도달할 수 없는 셀만 검색에서 제외한다. 원래 셀 순서, 실제 polygon subtraction,
  holes, 경계 접촉·면적 EPS는 유지한다. 큰 셀/질의는 전체 검사로 돌아간다.
  셀별 4,096 bin, 총 65,536 entry 한도로 인덱스 생성을 제한한다.
- 한 clip에서 inside/outside를 함께 계산한다. 원래 비교식·음수 쪽 나눗셈·점 순서를
  보존한다. polygon distance의 선분 상수만 반복문 밖으로 옮겼다.
- 누적 station 검색은 이진 검색 후 원래 EPS의 가장 이른 선분까지 되돌린다.
  실제 지도 선분의 보간식과 매우 짧거나 중복된 선분의 처리 기준은 같다.
- 후보 batch의 동일 점→중심선 투영을 재사용한다. 키는 실제 점과 양쪽 기준선의
  identity 및 전체 좌표 값이다. 기준선의 in-place 편집도 캐시를 무효화한다.
- 한 동기 `plan()` 호출 안에서 **정확히 같은 두 pose**의 형상 검사를 재사용한다.
  경계 객체/셀, 지도 객체, 차량 치수, 설정, 관측 곡률 범위, 장애물 전체 형상,
  road 검사 여부가 바뀌면 무효화한다. 최소 간격 진단도 cache hit마다 반영한다.
  이 memo는 다음 Planning 호출로 넘기지 않는다. 현재 차량의 rollout은 매번 실행한다.
- 기존 `_validated_paths`에는 차량·경계·지도 객체·설정 변경 시 무효화를 추가했다.
  미래 경계 hole로 교체된 경우 짧은 rollout에 도달하기 전에 전체 경로를 거부하는
  신규 시험이 있다. 지도는 기존처럼 로드 후 읽기 전용이며 live 파일 재로딩을 새로
  지원한 것은 아니다. 동일 형상 키의 body 판정은 지도 topology를 캐시하지 않는다.

후보 수/전환 길이/비용/선택 순서, FULL BODY collision, 실제 과거 도로 경계 검사 포함 검사,
곡률·횡가속도·조향·조향 변화율, 복귀 지원 거리 검사를 생략하거나 근사화하지 않았다.
조기 승인, threshold 변경, 지도 외삽, 후보 수 축소를 도입하지 않았다.

**2. 호출 흐름과 병목 측정 근거**

실제 `PlanningNode.plan()`의 흐름은 센서·거리 refresh → map match/Dijkstra cache →
global publication → normal/preview 추출 → 장애물 검증 → STOP-before-search →
Lattice 생성/검증/선택 → Control window → 센서·거리 재검사 → Local Path/diagnostic →
speed plan → 최종 센서·거리 검사 → target publication이다. 운영 노드 구조는 변경하지 않았다.

[`initial_profile.txt`](/workspace/logs/s4_stage5c_20261109/initial_profile.txt)와
[`phases_seconds.csv`](/workspace/logs/s4_stage5c_20261109/comparison/phases_seconds.csv)에
구간별 wall/process CPU, inclusive/exclusive, 호출 횟수가 있다. CSV의 초 단위 컬럼은
`wall_s/cpu_s/self_wall_s/self_cpu_s`다. 겹치는 inclusive 시간을 합산하지 않는다.

| 요청 구간 | 계측 표식 |
|---|---|
| 1 입력 준비 | 01_local_extract, 01_map_matching, 01_global_route, 01_pose_callback |
| 2 장애물 검증·변환 | 02_hazard_validation, 02_obstacle_merge, 02_obstacle_conversion, 02_obstacle_callback |
| 3 후보 생성 | 03_candidate_generation, 03_transition, 03_center_sampling, 03_lattice_plan |
| 4 차량 형상 충돌 | 04_vehicle_collision, 04_swept_body, 04_obstacle_distance |
| 5 경계 | 삭제된 경계 측정, 05_polygon_clipping |
| 6 곡률·조향 | 06_path_constraints, 06_curvature, 06_control_rollout (운영 PP 포함) |
| 7 평가·선택 | 07_candidate_cost, 07_speed_plan; min/정렬 등 선택 잔여 시간은 03_lattice_plan exclusive |
| 8 최종 Local Path | 08_control_path; ROS 메시지 생성·발행 잔여는 10_planning_callback exclusive |
| 9 센서·거리 | 09_sensor_readiness, 09_distance_safety, 09_status_callback |
| 10 ROS/전체 | 10_planning_callback, 10_diagnostic_publication 및 executor 원시 callback timeline |

대표 우측 곡선 알고리즘 profile에서 body 검사 2,540→1,122회, 경계 검사
2,355→937회, clipping 69,376→13,736회다. 생략된 계산은 동일 입력의 중복 작업이다.
instrumented exclusive wall은 clipping 약 0.454→0.059초, 경계 자체 0.280→0.099초,
장애물 거리 0.216→0.082초, swept body 0.140→0.062초, 비용 0.111→0.046초였다.
이 profile은 wrapper 오버헤드를 포함하므로 아래 비계측 percentile과 혼합하지 않는다.

**3. 환경, cold/warm, wall/CPU 결과**

Intel i9-11900H, 보이는 CPU 16개, Python 3.10.12, ROS Humble, 동일 컨테이너에서
전후를 측정했다. `environment.json`과 각 summary의 affinity/module_file을 저장했다.
CPU pin/실시간 우선순위/CPU 주파수 고정/독점 부하 격리를 적용하지 않았다.
다른 검증 프로세스와 부하·주파수 차이가 있으므로 하드 실시간 성능을 추론하지 않는다.
`process_time_ns`는 DDS background를 포함한 프로세스 CPU 시간이고 `perf_counter_ns`는
대기·스케줄링을 포함하는 wall 시간이다.

기준 소스는 Git `4c80691`의 별도 `baseline_src` 사본에서 실행했다. 운영 import와
하네스를 프로세스별로 분리했다. 알고리즘 11종은 최초 1회, warm-up 2회 뒤 검색 40회,
고정 경로 steady 40회, 별도 instrumented 1회다. 실제 노드 3종은 검색/steady 각 20회다.
아래 cold는 새 planner/node의 최초 호출이며 OS cache flush나 프로세스 부팅 WCET가 아니다.
import 시간은 summary에 별도로 있다. p99는 작은 표본의 보간 추정이며 보증값이 아니다.

실제 Planning + 입력 전달 scaffolding의 **wall 초**:

| 구간 | 버전 | cold | warm p50 | p95 | p99 | 관측 max |
|---|---|---:|---:|---:|---:|---:|
| 직선 | 전 | .4560 | .4750 | .4829 | .4837 | .4839 |
| 직선 | 후 | .2764 | .2736 | .2855 | .3361 | .3488 |
| 곡선 좌 | 전 | 1.0748 | 1.0760 | 1.0833 | 1.0837 | 1.0838 |
| 곡선 좌 | 후 | .4206 | .4175 | .4277 | .4291 | .4295 |
| 곡선 우 | 전 | 1.3176 | 1.3208 | 1.3313 | 1.3358 | 1.3369 |
| 곡선 우 | 후 | .4679 | .4706 | .4839 | .4948 | .4975 |

같은 호출의 **process CPU 초**:

| 구간 | 버전 | cold | warm p50 | p95 | p99 | 관측 max |
|---|---|---:|---:|---:|---:|---:|
| 직선 | 전 | .4565 | .4751 | .4829 | .4838 | .4841 |
| 직선 | 후 | .2766 | .2736 | .2851 | .3311 | .3427 |
| 곡선 좌 | 전 | 1.0755 | 1.0761 | 1.0836 | 1.0840 | 1.0841 |
| 곡선 좌 | 후 | .4211 | .4176 | .4277 | .4292 | .4296 |
| 곡선 우 | 전 | 1.3184 | 1.3210 | 1.3315 | 1.3360 | 1.3372 |
| 곡선 우 | 후 | .4683 | .4707 | .4840 | .4948 | .4975 |

**4. 형상·후보·샘플·장애물별 개선율과 한계**

실제 노드 wall p95 감소율은 직선 **40.9%**, 곡선 좌 **60.5%**, 곡선 우 **63.7%**다.
비계측 알고리즘 검색 40회 분류는 다음과 같다. 설정 변화는 합성 벤치마크 입력일 뿐
운영 YAML에 적용하지 않았다. 10/20km/h는 모델 한계 분석이다.

| 입력 | 전 p95(s) | 후 p95(s) | 감소율 |
|---|---:|---:|---:|
| 직선 좌 / 우 | .4676 / .4616 | .2680 / .2608 | 42.7% / 43.5% |
| 300m 곡선 좌 / 우 | 1.1013 / 1.2654 | .4141 / .4543 | 62.4% / 64.1% |
| 곡선, 장애물 4개 | 1.7568 | .7638 | 56.5% |
| 곡선, 장애물 12개 | 3.5327 | 1.6808 | 52.4% |
| 곡선, 후보 1개 / 6개 | .1209 / 1.5720 | .0965 / .5617 | 20.2% / 64.3% |
| 곡선, 조밀한 preview | 1.0248 | .4326 | 57.8% |
| 곡선, 10 / 20km/h 분석 | .8456 / .5588 | .4264 / .3256 | 49.6% / 41.7% |

기본 후보는 3개, 좌측 후보 샘플 324/327/343개, 우측 410/405/413개다.
기본 preview는 실제 추출 후 162점, 조밀 입력은 322점이다. 장애물 부하는
후보를 계속 평가시키는 도로 밖 추가 합성 solids이며 실제 센서 가시성을 승인하지 않는다.
알고리즘 벤치마크는 단독 kernel 측정이므로 그 결과 자체가 ROS 안전 게이트 승인도 아니다.

추가 실제 노드 stress 측정(`after_stress_node`, 3회+warm-up/첫 호출)에서 장애물 4/12개는
모든 단발 호출이 freshness로 빈 경로·속도 0을 출력했다. 이 별도 부하/주파수 구간의
warm p95는 약 .672/1.369초였다. 후보 6개는 cold .5524초 + 입력 .05초로 stale 거부됐고,
나머지 .6초 안의 호출은 통과했다. 보수적인 거부를 성공으로 바꾸지 않았다.
기본 설정 밖 부하까지 0.6초 또는 1.5초 예산을 충족한다고 주장할 수 없다.

**5. 신선도 및 원본 stamp/발행 시각**

`before_node/after_node/*.json`의 `timelines`에 원본 관측 stamp, 도착 ROS/monotonic,
Planning 시작/끝 ROS/monotonic, Local Path·target 발행 ROS/monotonic 시각을 저장했다.
`comparison/source_lifetimes.csv`는 전체 입력 수명이다. 입력 나이는 .05초로 고정하고
각 단발 호출마다 새 observation/status/localization을 실제 callback에 전달했다.

| 구간 | 전 최대 입력→종료(s) | 후 최대 입력→종료(s) | 후 단발 gate |
|---|---:|---:|---|
| 직선 | .5341 | .3986 | 23/23 |
| 곡선 좌 | 1.1338 | .4794 | 23/23 |
| 곡선 우 | 1.3868 | .5474 | 23/23 |

이는 cold 1+warm-up 2+검색 20회다. 뒤의 독립 publication 계측(각 8회)도 24/24
통과했지만, 우측 곡선의 최대 입력 수명은 **.5911초**로 여유가 약 8.9ms뿐이었다.
이를 .547초 이하로 항상 끝난다는 보증으로 해석해서는 안 된다.

`before_publication_audit/after_publication_audit`는 Path 메시지 header stamp도 기록한다.
Path header는 global 메시지를 구성한 시점이며 source stamp나 실제 발행 시각과 다르다.
target Float32에는 stamp가 없다. 운영 Control은 Path header 대신 각 callback의 수신
시각으로 watchdog을 갱신한다. 따라서 원본 관측의 수명 보장은 Planning 종료 gate가 맡는다.

heartbeat의 `published_ns/status_received_ns`와 실제 frame/source stamp·digest는 구분한다.
재전송/heartbeat로 matched frame의 source/steady receipt 나이를 갱신하지 않는다.
기존 `test_fresh_heartbeat_and_replayed_array_do_not_renew_old_data` 등을 모두 통과했다.
ROS clock은 원본 메시지 시간 비교에, monotonic은 경과시간/paused-clock timeout에 쓴다.
계산 중 .65초 지연을 실제로 넣으면 정상 ROS clock과 고정 ROS clock 모두 최종 빈 경로·
0 속도·BLOCKED를 확인했다. 시작 때 신선하다는 이유로 오래된 결과를 발행하지 않는다.

**6. 실제 ROS 실행 주기와 queued callback**

`s4_ros_freshness_probe.py`는 실제 SingleThreadedExecutor, Planning timer 20Hz,
합성 입력 timer 10Hz, 고정 pose를 3초씩 10회 시험한다. `enable_drive=false`,
출력은 memory sink, Control/차량 노드는 없다. 입력 guard를 주기마다 reset하지 않는다.

전 10/10회가 첫 검색 후 timeout 래치였다. 후 10회 중 **5회만 stale 래치 없이 유지**했고,
나머지 5회는 후속 콜백에서 `S4_SENSOR_SENSOR_SOURCE_STALE`로 정지했다.
검색 .447~.476초 동안 50ms timer 여러 회분이 밀린다. 시작 시 이미 .10초 된 관측은
첫 검색을 통과해도 센서 callback보다 다음 Planning이 먼저 실행되면 .6초를 초과한다.
후속 새 관측이 와도 BLOCKED는 기존 정책대로 자동 해제되지 않는다.
첫 검색 자체는 모든 trial에서 50ms period를 초과했다. 반복 고정 경로 단발 p95는
후 직선 .0335, 곡선 좌 .0406, 우 .0485초이며, 곡선 max .0562/.0596초도 관측했다.
검색·steady 양쪽 모두 20Hz schedulability/WCET 보증은 없다.

별도 test-only producer thread는 계산 시작 약 .05초 후 실제 DDS에 error/restart/
fault-history/new-observation을 발행한다. 후의 status callback 배달 지연은 약
.402~.419초였다. 계산 끝 이전에 **DDS에 도착했으나 아직 처리되지 않은 오류**는
동기 노드에서 볼 수 없다. old input으로 nonempty Path를 memory sink에 발행한 뒤,
오류 callback 처리 시 empty Path/속도 0을 발행하고 각각 PROCESSING_ERROR,
SENSOR_RESTARTED, SENSOR_FAULT_HISTORY로 래치했다. 모든 target은 처음부터 0이다.
새로운 정상 frame/pose는 배달·pairing 후 다음 계산에서 처리됐고 BLOCKED를 만들지 않았다.
오류가 callback에 이미 전달된 상황은 별도 hook 시험 3종으로 최종 재검사·래치를 확인했다.
queued 오류가 발행 즉시 감지됐다고 주장하지 않는다.

원시 trial과 발행/처리 시각은 `before_ros_probe_10.json/after_ros_probe_10.json`,
요약은 `comparison/executor_trials.csv`에 있다. 이는 일정한 고정 pose의 scheduling 시험이며
실차 추종 또는 MORAI latency 검증이 아니다.

**7. Stage 5A 44개 회귀와 실제 지연 폐루프**

기존 가상 .02초 callback 지연의 동일 입력 44개를 전후 각각 실행했다.
**44/44 기대 결과 충족, 복귀 12건, 의도된 BLOCKED 32건, 실제 모델 제동 정지 44건**이다.
후보/거부 사유/선택 geometry, 매 tick 궤적, 두 FSM의 시각·전환, 간격·경계·오차·
제동 거리·정지 후 유지까지 비교한 44개가 모두 **정확히 동일**하다.
근거 없는 BLOCKED→승인은 0건이다. 단독 알고리즘 11종의 전체 결과 서명도 모두 같다.

3km/h 주 검증은 8건이며, 10/20km/h 초기 속도 감속 분석 4건은 운용 속도 승인이 아니다.
12건의 회피·복귀 FSM은 `NORMAL → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL`이며
복귀 후 NORMAL 2초, 제동 정지 후 1초를 확인했다. PlanningFSM은 대표 직선/곡선에서
`WAIT_FOR_LOCAL_PATH → NORMAL_DRIVE → WAIT_FOR_LOCAL_PATH → OBSTACLE_AVOID →
NORMAL_DRIVE → GOAL_REACHED`다. 전체 전환 시각은
[`fsm_transitions.csv`](/workspace/logs/s4_stage5c_20261109/comparison/fsm_transitions.csv)와
각 실행 `fsm.json`에 있으며, fault별 BLOCKED 진입 시점도 포함한다.

12건에서 최소 장애물 간격 2.02908m, 최소 경계 여유 .46325m, 최대 횡오차 .02614m다.
운영 `ControlNode.control_loop`/PP/PID와 Sonata 형상, 지연·조향 변화율 제한 bicycle 모델을
그대로 연결했다. 정상 목표 0 정지는 운영 explicit-stop branch, BLOCKED 빈 경로는
safe-stop branch의 제동이다. PID가 이 안전 정지 명령을 계산했다는 의미는 아니다.

32개 의도된 거부의 전후 사유도 같다. sensor source stale 2, 추종 실패 2,
운영 관측 증거 없음 6, 경계 없음 4, 인접 허가/연결 없음 3, 안전 후보 없음 9,
장애물 제동 거리 부족 3, 계산 지연에 따른 status timeout 1, 복귀 공간의 새 장애물 1,
급곡선 STEERING_RATE 1이다. 각 실행의 정지 거리·시간·유지·형상 간격은 summary와
result.json에 저장했고, 모든 실행에서 충돌·도로 이탈 없이 실제 모델이 정지했다.

동일한 **현재 측정 하네스**에 전/후 운영 모듈을 각각 연결한 measured 모드는 virtual
busy 시간을 `max(.02초, 관측 core wall)`로 진행한다. 센서 callback은 busy 동안 대기하고
차량/Control은 계속 진행한다. 실제 ROS publication/serialization 전체를 이식한 것은 아니다.
이 하네스는 busy 종료 후 최신 가상 frame을 전달하는 모델이며, 독립 센서 producer의
모든 원본 frame과 DDS FIFO 깊이/QoS까지 재현하지 않는다. 실제 queued-message 문제는
앞 절의 ROS probe에서 별도로 측정했다.
3km/h 8개 결과는 다음과 같으며, 원래 return 기대값을 false로 남겼다.

| 조건 | 직선 4건 | 곡선 4건 | 모델 정지 |
|---|---|---|---|
| 전 실제 core 지연 | 복귀 4/4 | entry에서 SENSOR_STATUS_TIMEOUT 4/4 | 8/8 |
| 후 실제 core 지연 | 복귀 4/4 | hold에서 STEERING_RATE 4/4 | 8/8 |

후 곡선은 freshness 거부가 사라졌지만, hold 전환의 고정 경로 검증이 약 .10초 걸려
기존 >.05초 정책이 fresh pose 3회 재검증을 요구한다. 빈 경로/속도 0 동안 Control의
제동·조향 0과 모델 지연으로 실제 turn state가 바뀌고, 이후 rollout이 필요한 보정의
조향 변화율을 거부했다. 대표 hold 정지 거리 약 .036m, 시간 .167초였고 충돌·경계
이탈 없이 정지 후 1초 유지했다. rollout 초기 pose/속도와 마지막 planning rows를 보존했다.
STOP-before-search/3 fresh frame/STEERING_RATE를 완화해 성공 처리하지 않았다.

총 주 비교는 가상 지연 행렬 전후 88건 + measured 전후 16건=104건이다.
measured 복귀 성공률은 전후 모두 4/8=50%이며, 후의 곡선 실패 원인은 신선도에서
재검증/조향 응답으로 바뀌었다. 추가 미세 벤치마크와 고정 pose executor trial은 이
폐루프 시나리오 수에 섞지 않는다.

**8. 테스트와 빌드**

전체 **839 passed / 564.19초**, skip/failure 없음.
기존 Stage 5A 801 + Stage 5B 추가 15 + 신규 23이다.
Stage 5B 관련 28개는 기존 hdmap 13 + full-map 15로 전체에 포함되며 중복 가산하지 않는다.
신규는 기하학·cache 18개와 실제 disarmed node 신선도/오류 5개다.
기준 `4c80691`에서 생성한 고정 fixture에는 coverage 4,200개, clipping 2,000개,
거리 2,000개, EPS/tiny/duplicate station sample 13,700개가 있다.
baseline source SHA/seed를 포함하며 운영 경계 파일 형식이 아니다.

[`all_tests.txt`](/workspace/logs/s4_stage5c_20261109/all_tests.txt),
[`all_tests.xml`](/workspace/logs/s4_stage5c_20261109/all_tests.xml),
[`final_new_tests.txt`](/workspace/logs/s4_stage5c_20261109/final_new_tests.txt)에 결과가 있다.
`colcon build --packages-select edgenode_planning --symlink-install` 성공,
1 package / 1.03초이며 [`build.txt`](/workspace/logs/s4_stage5c_20261109/build.txt)에 저장했다.
`git diff --check` 통과. Docker/ROS 설치·패키지 추가는 하지 않았다.

**9. 보호 조건과 Git diff**

Dijkstra, 운영 PP/PID/ControlNode, PlanningNode, 센서·거리 모듈, local_path_planner,
정상 글로벌 패스, 모든 운영 YAML/launch는 기준 파일 hash와 같다.
`enable_drive=false`, 21.19m ROI, .6초 freshness, 1.5초 Planning budget,
승인 없는 boundary/pin/차선 변경 금지, BLOCKED latch를 보존했다.
지도 원본을 수정하거나 운영 승인 파일을 생성하지 않았다.
`protected_files.json`에 시작 시 173개 tracked 파일 hash와 대조 결과가 있다.
`working_tree.diff`는 tracked diff와 신규 텍스트 파일의 diff를 모두 포함한다.
gzip fixture는 별도 SHA/크기를 남겼다. Git HEAD는 그대로이며 commit/push는 하지 않았다.

**10. 남은 구조 개선**

단발 빠르기만으로 현재 timer/executor에서의 관측 수명을 보장할 수 없다.
다음 단계에서는 입력 수신·안전 감시와 긴 후보 검색의 scheduling을 분리하는 설계를 먼저
검증해야 한다. 이번에는 운영 비동기/스레드 구조를 바꾸지 않았다.

설계 시 immutable job에 원본 stamp/ROS·steady 도착 시각, pose·speed·turn,
obstacle digest와 session/frame/fault sequence, route/map revision, boundary SHA,
차량·제약 설정 revision을 고정해야 한다. 발행 직전에 현재 revision/오류 epoch와 일치,
ROS/steady deadline 및 실제 pose의 재검증, 기존 STOP/fresh-3 조건을 모두 확인해야 한다.
새 frame/오류/재시작/경로·지도·치수 변경과 충돌하는 job은 결과를 폐기하고 STOP을 유지한다.
새로운 관측을 오래된 계산의 신선도 갱신에 쓰면 안 된다. 대기 중 전진을 허용하려면
기존 거리·실제 제동·형상 경계를 별도로 충족해야 한다.

향후 계산 kernel의 추가 최적화 또는 native 구현은 현재 고정 fixture와 전체 후보/rollout
대조를 먼저 통과해야 한다. Python thread가 CPU 병목이나 긴 callback의 안전 감시를
자동으로 해결한다고 가정하지 않는다. 우선순위는 bounded ingress/STOP deadline과
결과 폐기 규칙 검증, 그 다음 full closed-loop delay/steering 응답 검증이다.

**11. 파일과 재현 명령**

도구: `tools/s4_timing_audit.py`, `tools/s4_ros_freshness_probe.py`,
`tools/s4_stage5c_comparison.py`, 기존 `tools/s4_closed_loop_report.py`.
추가 테스트: `test_s4_performance_regressions.py`, `test_s4_planning_timing_node.py`,
`test/fixtures/s4_stage5c_geometry_golden.json.gz`.
원본 fixture 생성 스크립트는 실행 폴더 `generate_reference_fixtures.py`에 보존했다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export MPLCONFIGDIR=/tmp/edgenode_stage5c_mpl
python3 tools/s4_timing_audit.py --repeats 40 --out /tmp/s5c_algorithm
python3 tools/s4_timing_audit.py --kind node --repeats 20 --out /tmp/s5c_node
python3 tools/s4_ros_freshness_probe.py --periodic-trials 10 --out /tmp/s5c_ros.json
python3 tools/s4_closed_loop_report.py --out /tmp/s5c_modeled
python3 tools/s4_closed_loop_report.py --timing-mode measured \
  --match A_straight_left_3,A_curve_left_3 --out /tmp/s5c_measured
python3 -m pytest -q -p no:cacheprovider ros2_ws/src/edgenode_perception/test \
  ros2_ws/src/edgenode_planning/test ros2_ws/src/edgenode_control/test
python3 tools/s4_stage5c_comparison.py logs/s4_stage5c_20261109
```

최적화 전 timing/probe는 `--source-root /workspace/logs/s4_stage5c_20261109/baseline_src`를 쓴다.
measured 폐루프의 전 버전은 **현재 하네스**를 유지하고 PYTHONPATH 앞에 baseline_src의
planning/control package root를 추가했다. 초기 measured sources.json은 workspace hash만
기록하므로 기록된 명령과 동일한 PYTHONPATH의 사후 import 경로·hash 대조를
`before_measured_loop/import_provenance.json`에 별도로 저장했다. 이 대조는 실행 도중의
trace가 아니다. 현재 CLI는 실제 import 파일과 hash도 source_digest에 포함한다.
`before_matrix`는 기준 하네스, `after_matrix_final`은 새 하네스의 기본 modeled 모드다.
하네스 계측 필드를 제외한 모든 동작 비교는 정확히 같다.

표·그림: `comparison/performance.csv` (모든 cold/warm/steady CPU·wall 통계),
`phases_seconds.csv`, `source_lifetimes.csv`, `matrix_semantic_regression.csv`,
`measured_closed_loop.csv`, `fsm_transitions.csv`, `executor_trials.csv`, `comparison.json`,
[`timing_and_freshness.png`](/workspace/logs/s4_stage5c_20261109/comparison/timing_and_freshness.png).
각 loop 실행 디렉터리에 `result.json/trajectory.csv/planning.csv/fsm.json/trajectory.png`가 있다.

**12. 실제 MORAI 전에 검증하지 못한 사항**

실제 DDS/ROS/차량을 연결한 가동 시험, 실시간 호스트 부하·장시간 jitter, 동기 clock,
LiDAR 관측 범위·측방/가림·ground truth/digest 전송, Sonata 동특성·마찰·포화·지연,
연속 검증 대기 중 실제 제동·조향 응답은 미검증이다. 실제 차량은 구동하지 않았다.
Stage 5B의 K-City 도로면·외곽/holes·교차 규칙 승인 부족은 계산 최적화와 독립적으로
남아 있다. 합성 A의 50m 가상 관측/합성 경계를 운영 설정에 주입하지 않았다.
B의 21.19m ROI/증거·경계·허가 없음 6건은 계속 BLOCKED·실제 모델 정지다.
센서·지도 문제가 해결되었다거나 실제 K-City Lattice 회피가 가능하다고 결론내리지 않는다.

**13. 필요한 패키지 빌드와 disarmed 재시작**

이 작업에서는 운용 launch를 시작하지 않았다. 아래는 사용자가 별도로 확인할 명령이며,
기존 실행 터미널에서 Ctrl+C로 종료한 뒤 수행한다. Control/차량 명령 노드는 포함하지 않는다.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select edgenode_planning --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
ros2 launch edgenode_bringup s4_validation.launch.py use_sim_time:=false
```

개별 재시작은 위 통합 launch를 중지하고 서로 다른 터미널에서 같은 환경을 source한 뒤 실행한다.

```bash
ros2 launch edgenode_perception lidar_obstacle.launch.py
ros2 run edgenode_planning planning_node --ros-args \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/planning.yaml \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/s4_sensor_validation.yaml \
  -p enable_drive:=false
ros2 param get /planning_node enable_drive
timeout 5s ros2 topic echo --once /planning/s4_sensor_state
timeout 5s ros2 topic echo --once /planning/s4_distance_state
timeout 5s ros2 topic echo --once /planning/avoidance_debug
timeout 5s ros2 topic echo --once /planning/target_speed
```

검증된 실제 관측·경계·차선 변경 증거가 없으면 현재 운용 게이트는 계속 BLOCKED가 맞다.
