# S4 live NONCONVEX_QUAD #44: sampling 정규화 결과

원본은 `/workspace/logs/kcity_s4_corridor_diagnostics/s4_corridor_vvc82ouy.json`이다.
수정 전 독립 CLI replay에서 `matches_recorded=true`로 전체 결과가 일치했다.
원본 JSON을 변경 없이 gzip 압축해 `test/fixtures/s4_corridor_vvc82ouy.json.gz`로 보존했다.
압축 전 SHA-256은 `9f53e8c6828fce5b54905b29d5ce88544d96eb316842151fd328e76f9e2c233c`다.

**A. 정확한 sampling 원인**

`LanePair.quads`는 map 원본 vertex의 XY arc station, adjacent vertex의 original projection
station, 0.5 m regular grid를 set으로 합쳤다. set은 정확히 같은 float만 제거한다.
dump의 original vertex #15까지 3D arc는 **7.500000000000543 m**, XY arc는
**7.499951351758102 m**다. XY grid의 7.5 m와 일치하지 않아 두 cross-section이 남았다.

station 자체는 strict monotonic이다. 다만 정확한 vertex에서는 incoming tangent,
48.648 µm 뒤 grid에서는 outgoing tangent를 사용했다. 작은 tangent 불연속이 약 3.48 m
떨어진 adjacent 교차점을 이동시켜 **adjacent station이 1.488541 mm 역행**했다.
두 cross-section이 뒤집혀 bow-tie quad가 생겼으며 전체 envelope가 폐기됐다.
해당 horizon의 실제 original/adjacent polyline 선분 교차는 0개다.

이 현상은 float64 underflow나 실제 차도 경계 이탈이 아니다. 원래 quad의 signed-area
절댓값도 약 **0.002503125 m²**로 유한하다. 면적/convexity tolerance를 키우지 않고,
중복에 가까운 supplemental row가 만드는 잘못된 대응을 제거했다.

**B. 수정 전 quad #44의 두 cross-section**

| 값 | 첫 row | 다음 row |
|---|---|---|
| 원인 | original vertex #15 | regular grid #15 |
| original station [m] | 7.499951351758102 | 7.5 |
| original XY | (140.62228102727443, 1429.6629360351312) | (140.62228096216597, 1429.6629846833296) |
| original tangent | (-0.0017804672565090, 0.9999984149669182) | (-0.0013383511763406, 0.9999991044076636) |
| adjacent XY | (144.0991270207211, 1429.6691264553906) | (144.09913585461538, 1429.6676379403323) |
| adjacent station [m] | 7.529238532456814 | 7.527749991185541 |

original station 차이는 `+0.000048648241898 m`, adjacent station 차이는
`-0.001488541271273 m`다. 기존 turn 값은 다음과 같이 부호가 섞였다.

```text
[-0.0001691430293888702, +0.005175391969408567,
 +0.0051753923022466964, -0.00016914269655074075] m²
```

**C. 제거/병합 기준과 유지한 gate**

`LanePair.corridor_envelope`는 다음 조건을 모두 만족하는 **추가 grid station만** 제거한다.

1. 가장 가까운 보존 station과 longitudinal 차이 ≤ **0.0001 m (0.1 mm)**.
2. 두 cross-section의 original/adjacent endpoint 이동이 각각 ≤ **0.002 m (2 mm)**.
3. 두 adjacent intersection 사이에 실제 adjacent map vertex가 없어야 한다.

0.1 mm 기준은 실제 중복 0.048648 mm를 포함하는 sub-millimetre sampling 범위다.
2 mm는 관측한 1.488541 mm normal-intersection 이동을 포함하면서 큰 normal jump를
제거하지 않도록 둔 별도 상한이다. **candidate 밖 허용 거리로 사용하지 않는다.**
가장 가까운 실제 station을 그대로 보존하며 station 평균, endpoint 이동, normal 평균은 없다.

original map vertex, adjacent projection station, start/end cap은 모두 보존한다.
따라서 supplemental grid의 무의미하게 작은 span은 없어지지만, 서로 가까운 **실제**
geometry anchors는 임의로 합치지 않고 기존 convex gate로 검사한다.
strict original station ordering과 adjacent progress ordering을 확인하며,
비볼록 quad, 실제 crossing, 큰 normal jump, map corner를 지우는 병합은 거부한다.
실패하면 부분적으로 만든 quad도 반환하지 않는다.

`segment_interval` / `corridor_covers`의 inclusion 식과 `EPS=1e-7`은 그대로다.
clearance 1.20 m, usable curvature 0.16046692048050848 /m, steering clamp, adjacent target,
Pure Pursuit, PID, Dynamic Lookahead, speed profile 및 RViz는 변경하지 않았다.

**D. 수정 후 envelope**

| 값 | 수정 전 | 수정 후 |
|---|---:|---:|
| station 수 | 122 | 107 |
| 추가 grid row 제거 | 0 | 15 |
| 반환한 유효 quad 수 | 0 (44번에서 전체 폐기) | **106** |
| original station strict monotonic | true | true |
| adjacent cross-section progress | 일부 역행 | strict monotonic |
| 실제 map/projection/cap station 보존 | 전체 | 전체 |

7.499951351758102는 남고 7.5 grid row가 제거됐다. 검사 범위와 두 centerline은 동일하다.
runtime debug JSON의 `envelope`에 quad/station 개수와 실제 제거 목록을 발행한다.

**E. 같은 dump의 실제 candidate 판정**

원래 실패한 **63점 candidate를 그대로** 새 envelope에 검사하면 corridor safe=true,
failed point count=0이다. 최초 reference cross-section의 allowed lateral range는
약 `[-3.453447740079053, 0] m`이며 actual lateral은 `-0.079599670349182 m`다.
runtime에서 통과한 경로도 `reference_cross_section.allowed_lateral_ranges_m`으로 확인된다.

dump의 실제 원래 candidate curvature는 **0.173881692803299 /m**로 기존 제한을 넘는다.
empty envelope일 때는 `start_in_corridor=false`여서 기존 길이 탐색이 첫 후보 뒤 중단됐다.
정규화 후에는 기존 탐색이 계속되어 다음 feasible candidate를 찾는다.

| 새 평가 후보 값 | 결과 |
|---|---:|
| candidate points | 64 |
| ENTRY length | 11.186337379314498 m |
| 실제 adjacent endpoint offset | -3.479201497110865 m |
| clearance | 2.864745016887398 m |
| maximum curvature | 0.156893831002163 /m |
| corridor_safe | **true** |

곡률 제한이나 목표 offset을 조절해 첫 후보를 통과시킨 것이 아니다. 원래 63점 후보는
corridor를 통과하더라도 **steering/curvature에서는 계속 reject**된다.

이 dump에는 PRE-plan `_blocked=true`가 저장돼 있다. 기존 3-cycle release 확인도 그대로
적용되므로 단일 replay의 최종 status는 BLOCKED / **RELEASE_PENDING**이다.
같은 frozen 정지 입력을 오프라인에서 반복하면 다음과 같다.

```text
cycle 1: corridor=true, BLOCKED / RELEASE_PENDING
cycle 2: corridor=true, BLOCKED / RELEASE_PENDING
cycle 3: corridor=true, AVOIDING / ENTRY
```

**F. 실제 maximum candidate violation**

gate tolerance를 적용하지 않은 world-space quad-union distance를 모든 vertex에서 계산했다.
원래 63점 candidate와 새 64점 candidate 모두 최대값은
**2.2737367544323206×10⁻¹³ m**다. 각각 마지막 vertex index 62 / 63에서 발생하는
부동소수점 계산 오차 수준이며 실질적인 geometric overshoot는 없다.
통과한 candidate의 기존 diagnostic `max_violation_m`는 정의에 따라 0 m다.
이는 vertex 기준 값이고 차량 body sweep 최대 위반이라고 해석하지 않는다.

**G. unsafe 유지**

기존 `s4_20kph_blocked_scenes.json` 3개는 모두 BLOCKED다.
세 case의 기존 `ENTRY_DEADLINE+CURVATURE+DRIVABLE_CORRIDOR` reject도 유지한다.
곡률은 각각 0.533956524 / 0.837864972 / 0.838380262 /m다.

새 회귀는 원본 live dump와 좌우 mirror, 1 cm 외부 candidate, 큰 normal jump,
near-duplicate 실제 map vertex 보존, adjacent map corner 보존, 실제 centerline crossing,
기존 곡선 nonconvex rejection 및 기존 ENTRY/HOLD/recovery/no-adjacent를 검사한다.
compressed fixture는 원본 dump 전체를 보존하므로 외부 MGeo/bag 없이 재현된다.

**H. 재현 명령과 증적**

```bash
source /workspace/ros2_ws/install/setup.bash
python3 -m edgenode_planning.corridor_scene \
  /workspace/logs/kcity_s4_corridor_diagnostics/s4_corridor_vvc82ouy.json \
  --output /tmp/s4_corridor_after.json
```

수정 후의 `matches_recorded=false`는 의도한 판정 변경 때문이다. 저장된 이전 BLOCKED 결과와
새 결과 전체가 다르다는 뜻이며, 입력을 바꿨다는 뜻이 아니다.
상세 증적은 `/workspace/logs/kcity_s4_corridor_envelope_fix/`의
`replay_before.json`, `sampling_before.json`, `replay_after.json`, `results.json`에 있다.
검증 결과는 전체 source tests **712 passed, 1 skipped, 2 warnings**,
`colcon build --symlink-install` **7 packages finished**, `git diff --check` 통과다.
skip은 기존 copyright 검사이며 warning 2개는 기존 logger entry-point deprecation이다.
로그는 같은 디렉터리의 `pytest.txt`, `build.txt`, `verification.json`에 보존했다.
MORAI 실제 주행, live node 재시작, commit/push는 수행하지 않았다.
