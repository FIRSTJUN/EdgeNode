# S4 corridor failure 진단 / 정지 입력 snapshot

아래는 진단 추가 단계의 기록이다. 이후 확보된 live dump의 envelope sampling 수정과
최신 결과는 [s4_corridor_envelope_fix.md](s4_corridor_envelope_fix.md)에 정리했다.

이번 변경은 **관측 기능만 추가**한다. `lane_geometry.py`의 corridor 생성,
`corridor_covers`, clipping tolerance, 후보 생성·선택·승인 조건은 변경하지 않았다.
clearance 1.20 m, curvature/steering limit, 실제 adjacent target 및 Control/Speed 코드는 그대로다.
실제 MORAI 주행과 live node 재시작은 수행하지 않았다.

## 현재 scene에 대해 아직 모르는 것

사용자가 보고한 offset -3.477430 m / clearance 2.913538 m / curvature 0.121888 /m인
정지 입력 묶음은 저장되어 있지 않다. 해당 수치만으로 최초 실패 좌표나 위반 거리를
복원할 수 없다. 기존 `test/fixtures/s4_adjacent_lane_scene.json`은 다른 frozen input이며
수정 전부터 corridor를 통과한다. 이 fixture를 현재 실패 scene이라고 간주하지 않는다.

정지 상태에서 아래 topic과 snapshot을 확보한 뒤 실제 원인이 envelope 생성 실패인지,
centerline/normal 차이인지, 실제 기하 overshoot인지 구분할 수 있다.

## 기존 gate의 코드상 의미

`LanePair.quads(start,end)`는 original station마다 original tangent의 normal을
adjacent polyline과 교차시켜 cross-section을 만든다. original vertices,
adjacent vertices의 original nearest-segment projection station, 0.5 m station을
포함하고, 이웃 cross-section 사이를 convex quad로 연결한다. 어떤 row의 교차점이
없거나 quad가 비볼록이면 전체 envelope를 빈 배열로 반환한다.

`corridor_covers`는 candidate의 **모든 선분 전체**가 quad union에 덮이는지 convex clipping
interval을 합쳐 검사한다. 단일 global lateral scalar 비교 방식은 아니다.
원본 lane과 adjacent lane **center 사이의 reference-point sanity gate**다.
실제 lane width/차도 외곽 경계/body sweep 검증으로 해석하지 않는다.

clipping은 edge `e`, candidate start `a`, quad vertex `p`에 대해
`sign * cross(e, a-p) >= -EPS`인 구간을 사용한다. 기존 `EPS=1e-7`은 cross product에선
면적 단위, interval merge에서는 무차원 fraction이다. 실제 edge별 거리 tolerance는
`EPS / |e|` m다. 이번에는 이 단위 혼용을 포함해 기존 판정을 그대로 보존했다.
normal 변화로 잘못 연결된 quad나 centerline 작은 bulge도 거부될 수 있으며,
수 cm를 float 오차라고 간주해 허용하지 않았다.

## Runtime topics

| Topic | Type | 의미 |
|---|---|---|
| `/planning/lane_change_corridor_safe` | Bool | 기존 gate 결과 |
| `/planning/lane_change_corridor_violation_m` | Float32 | 거부된 candidate vertices에서 quad union까지 거리의 최댓값 [m]. accepted=0, 미평가/유효 envelope 없음=NaN |
| `/planning/lane_change_corridor_failed_index` | Int32 | 최초 거부 vertex의 0-based index. vertex는 모두 통과하고 선분 내부만 실패하면 해당 선분의 시작 index. 미평가/통과=-1 |
| `/planning/lane_change_corridor_failed_point_count` | Int32 | 거부된 candidate vertex 개수. 빈/무효 envelope에서는 검사 불가능한 전체 vertex 개수 |
| `/planning/lane_change_corridor_debug` | String (JSON) | 아래 상세 수치 |

`debug.first_failure`, `debug.maximum_violation_point`에 다음을 함께 발행한다.

- `candidate_point_index`, `world_xy`, `violation_m`
- `original_projection`, `adjacent_projection`: 실제 nearest **segment**의 index,
  station, world XY, tangent/normal, signed lateral 및 longitudinal residual
- `bounds_frame=normal_at_original_projection`, `signed_lateral_m`
- `corridor_lower_m`, `corridor_upper_m`, `allowed_lateral_ranges_m`:
  **실제 quad union**과 해당 original normal 선의 교차 구간. 끊어진 구간도 유지한다.
- `lateral_violation_m`: 위 구간 대비 lateral 위반. `violation_m`은 world 공간에서
  quad union까지의 최단거리이며 end cap/곡선에서는 두 값이 다를 수 있다.
- `nominal_center_bounds_m`: 해당 station의 center-to-center 범위. 실제 gate 구간과
  비교하기 위한 설명값이며 승인 조건에 쓰지 않는다.
- `normal_angle_difference_rad`: 두 lane projection의 tangent/normal 차이
- `nearest_quad`, `nearest_quad_edges`: 실제 polygon과 signed edge distance,
  edge별 기존 numerical tolerance [m]

`first_segment_failure`는 clipping union에서 발견한 최초 공백 내부의 witness를 제공한다.
endpoint들이 통과해도 선분 내부가 corridor 밖이면 실패를 기록한다.
`segment_start_index`, `segment_fraction`, `uncovered_fraction_range`를 함께 제공한다.
**violation topic은 연속 선분 전체의 최대 거리나 차량 swept-body clearance가 아니다.**
특히 선분 내부만 실패하면 vertex 최대값은 0이고 witness의 거리는 별도 필드에 있다.

envelope가 비어 있으면 `reason=EMPTY_OR_INVALID_ENVELOPE`와
`envelope_failure`를 기록한다. `NONCONVEX_QUAD`는 station 범위/quad 좌표/turn 값,
`NORMAL_INTERSECTION_UNAVAILABLE`은 실패 station을 제공한다.
이 경우 정의되지 않은 최대 위반 거리를 0 m로 꾸며내지 않고 null/NaN으로 표시한다.
진단값은 기존 evaluated candidate 수치와 같은 후보 기준이며, 승인된 adjacent lane을
새로 선택했다는 뜻이 아니다. 새로운 planning 입력이 없으면 NOT_EVALUATED로 지운다.

## 정지 상태에서 확보

업데이트한 Planning node를 평소 절차로 실행하고 차량 정지 / `enable_drive=false`를 유지한다.
아래 명령은 관측과 파일 저장만 수행한다.

```bash
ros2 topic echo /planning/lane_change_corridor_debug --field data --once
ros2 topic echo /planning/lane_change_corridor_violation_m --once
ros2 topic echo /planning/lane_change_corridor_failed_index --once
ros2 topic echo /planning/lane_change_corridor_failed_point_count --once
ros2 service call /planning/dump_lane_change_corridor_scene std_srvs/srv/Trigger '{}'
```

service 응답 `success=true`의 message가 생성한 JSON 절대경로다.
기본 디렉터리는 `/workspace/logs/kcity_s4_corridor_diagnostics`이며 시작 parameter
`lane_change_corridor_dump_dir`로 변경할 수 있다. 호출마다 고유 파일을 생성하며,
자동 반복 저장은 하지 않는다. 저장 권한 오류/현재 입력 없음은 `success=false`로 응답한다.

snapshot은 **가장 최근 완료된 Planning cycle이 실제 사용한 입력**이다. 별도 토픽을 나중에
모아 재조합하지 않는다. 각 센서의 측정 시각이 서로 동일하다는 의미는 아니다.
JSON에는 ROS time/frame, Ego XY/yaw, reference path, obstacle footprint와 grouping,
road progress, speed input, current/route IDs, 필요한 실제 MGeo link records,
직전 planner 설정·상태·remembered obstacles·world transition, 평가 candidate 전체,
original/adjacent geometry, 실제 검사 quad와 승인된 canonical local path를 포함한다.
BLOCKED에서는 canonical local path가 비어 있어도 **실패한 후보 geometry**가 남는다.

## Offline 재현

```bash
source /workspace/ros2_ws/install/setup.bash
python3 -m edgenode_planning.corridor_scene /absolute/path/s4_corridor_XXXX.json \
  --output /tmp/s4_corridor_replay.json
```

`matches_recorded=true`이면 결과와 진단을 포함한 전체 planner 결과가 snapshot과 일치한다.
이 명령은 ROS publisher/service를 생성하지 않는다. 원본 MGeo 디렉터리나 삭제된 bag도
필요하지 않다. 재현 schema는 `edgenode_s4_corridor_scene_v1`이다.
ENTRY/HOLD/RECOVERING/BLOCKED/RELEASE_PENDING에 대해 JSON 왕복을 검사했다.

## 기존 fixture 비교와 제한

기존 frozen fixture, speed input=20 km/h의 수정 전/후 수치는 동일하다.

| 값 | 전 / 후 |
|---|---:|
| offset | -3.474670896 m |
| clearance | 2.979798009 m |
| maximum curvature | 0.073466669 /m |
| corridor | true / true |
| state | ENTRY / ENTRY |

별도 합성 곡선에서 actual adjacent centerline HOLD조차 비볼록 quad 때문에 거부되는
기존 문제도 진단 테스트로 확인했다. 이번에는 해당 rejection을 유지했다.
현재 live failure와 같은 원인이라고 단정하지 않으며, live dump 확보 후 수정 근거로 삼는다.
기존 unsafe fixtures, 잘못된 초기 heading, corridor 외부 1 cm 이상의 경로,
선분 내부 공백 및 no-adjacent BLOCKED 판정을 유지한다.

## 검증 결과

- 전체 source tests: **704 passed, 1 skipped, 2 warnings**. skip은 기존 copyright 검사,
  warnings는 기존 logger entry-point deprecation이다.
- `colcon build --symlink-install`: **7 packages finished**.
- `git diff --check`: 통과.
- 수정 전 source와 비교한 기존 frozen input 및 unsafe fixture 3개는 신규 진단 필드를
  제외한 **전체 결과가 동일**했다.
- 합성 wrong-heading 사례는 clearance=3.0 m, curvature=0.099233 /m에서 corridor만
  실패했다. 첫 실패 index=1, XY=(0.246969666, -0.029626360), 최대 vertex 위반
  0.175272066 m, 실패 vertex 16개를 기록했다. **현재 live scene의 수치가 아니다.**
- ROS dump service transport, strict JSON 저장 및 독립 CLI replay의
  `matches_recorded=true`를 확인했다. 테스트 ROS domain=173 / localhost로 격리했다.
- 증적: `/workspace/logs/kcity_s4_corridor_fix/pytest.txt`, `build.txt`,
  `gate_comparison.json`, `synthetic_replay.json`.
- 실제 live 실패 입력을 확보하지 않았으므로 현재 scene의 최초 실패점·최대 위반 거리 및
  직접 원인은 아직 미확정이다. MORAI 주행 / live node 재시작 / commit / push 없음.
