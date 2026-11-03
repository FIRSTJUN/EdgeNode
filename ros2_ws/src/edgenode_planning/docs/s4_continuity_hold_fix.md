# S4 continuity 재시험: 출발 BLOCKED / HOLD 최소 수정

대상은 `/workspace`, `k-city`, HEAD `81a2add`와 작업 시작 당시 미커밋 변경이다. bag은 `logs/kcity_s4_static_obstacle_20kph_continuity_01`이다. 원본 DB는 읽기 전용으로 열었다. **MORAI 실행·주행, commit/push는 하지 않았다.**

조향 clamp, curvature limit **0.16046692048050848 /m**, clearance **1.20 m**, configured offset **±2.4 m**는 변경하지 않았다. Control/PID/Localization/Dijkstra/Perception self-mask와 runtime speed 동기화 구현도 변경하지 않았다.

production 수정은 `edgenode_planning/lattice_planner.py` 한 파일이다. 테스트는 `test_lattice_hold.py`와 `fixtures/s4_continuity_hold_scenes.json`을 추가하고, `test_lattice_s4_safety.py`의 예전 unconditional-stop 기대를 안전 HOLD/rear gate 기대로 갱신했으며 `test_planning_node.py`에 target 20/0 검증을 추가했다. 이 보고서를 포함해 이번 단계의 source/test/docs 변경은 6개 파일이다.

분석 코드·소스 snapshot·수치 증적은 `logs/kcity_s4_continuity_hold_fix/`에 있다. `analyze.py`, `inspect_cycles.py`, `reproduce_first_stop.py`, `compare.py`가 ROS 노드를 띄우지 않고 원본 bag/고정된 이전 소스/수정 후 순수 함수를 사용한다.

## A. 출발 후 BLOCKED까지 시간

bag은 31.795213129초, 16,664 messages이며 planning 진단 cycle은 390개다. cycle 번호는 아래에서 **0-based**다. 시간은 첫 bag message 기준 수신시각이며 토픽별 수신시각 차이를 구별했다.

- 첫 target=20: **t=5.754583950 s**, cycle **18**.
- 같은 target 토픽의 첫 target=0 복귀: **t=6.651970556 s**, cycle **21**. 차이는 **897.386606 ms**, **3 planning cycles**다.
- 첫 BLOCKED status 수신: **t=6.652251785 s**. 첫 양의 target 수신부터 **897.667835 ms**다. 이유는 **NO_SAFE_FEASIBLE_CANDIDATE**이며 OBSTACLE_NOT_PASSED가 아니다.
- 첫 EMERGENCY_STOP 수신: **t=6.652013525 s**, cycle 21.
- 첫 **OBSTACLE_NOT_PASSED** reason 수신: **t=15.054472084 s**, cycle **54**. 첫 양의 target 수신부터 **9299.888134 ms**, **36 cycles** 뒤다.

`/parameter_events`, parameter service 요청/응답 및 drive_status가 이 bag에 없으므로 **enable_drive가 accepted된 정확한 시각은 확정할 수 없다.** cycle 18은 기록상 첫 drive 허용 cycle이다. cycle 17은 READY/target=0이다. Path header 기준 cycle 18 계산 시작은 약 t=5.441228 s이며, callback 처리/계산/전달 시간이 포함되므로 이를 target 수신시각과 동일시하지 않았다.

실제 Ego speed가 처음 0.1 km/h를 넘은 표본은 t=6.105353801 s다. 이 기준에서 첫 BLOCKED status까지 546.897984 ms다. 첫 BLOCKED 당시 수신 근접 Ego speed는 약 4.438 km/h다. 이후 두 번 재출발했고 bag 최대 속도는 15.509 km/h, 처음/마지막 Ego XY의 순변위는 약 5.631 m다. 따라서 전체 bag을 ‘전혀 움직이지 않고 즉시 OBSTACLE_NOT_PASSED’라고 설명하지 않는다.

| cycle | status 수신 t (s) | status / 이유 | FSM | target | 근접 Ego km/h | actual d (m) | approved | front / rear (m) |
|---:|---:|---|---|---:|---:|---:|---:|---|
| 17 | 5.434958 | AVOIDING | READY | 0 | ≈0 | -0.504533 | -1.8 | 18.419968 / 19.237032 |
| 18 | 5.754888 | AVOIDING | OBSTACLE_AVOID | 20 | ≈0 | -0.504537 | -1.8 | 18.420069 / 19.237133 |
| 21 | 6.652252 | BLOCKED / NO_SAFE_FEASIBLE_CANDIDATE | EMERGENCY_STOP | 0 | 4.438 | -0.504377 | -1.8 | 18.416855 / 19.233919 |
| 22 | 6.682236 | BLOCKED / INVALID_HEADING_BOUNDARY | EMERGENCY_STOP | 0 | 4.998 | -0.503249 | -1.8 | NaN / NaN |
| 33 | 9.351570 | AVOIDING | OBSTACLE_AVOID | 20 | ≈0 | -0.506731 | -1.8 | 17.771070 / 19.108360 |
| 37 | 10.385680 | BLOCKED / NO_SAFE_FEASIBLE_CANDIDATE | EMERGENCY_STOP | 0 | 6.693 | -0.507694 | -1.8 | 17.755598 / 19.092890 |
| 45 | 12.606714 | AVOIDING | OBSTACLE_AVOID | 20 | ≈0 | -0.584983 | -1.8 | 16.783838 / 18.436628 |
| 51 | 14.420069 | BLOCKED / NO_SAFE_FEASIBLE_CANDIDATE | EMERGENCY_STOP | 0 | 15.099 | -0.752817 | -1.8 | 15.824755 / 17.910198 |
| 54 | 15.054363 | BLOCKED / OBSTACLE_NOT_PASSED | EMERGENCY_STOP | 0 | 0.053 | -1.681896 | -1.8 | 12.664767 / 14.750211 |
| 389 | 31.765521 | BLOCKED / OBSTACLE_NOT_PASSED | EMERGENCY_STOP | 0 | ≈0 | -1.751480 | -1.8 | 12.687393 / 14.772837 |

speed는 status 수신에 가장 가까운 Ego status이며 planner callback 입력 속도라고 주장하지 않는다. scalar 진단은 모두 같은 개수/순서의 cycle로 연결했고, 실제 기하 입력은 별도로 offset 및 Path 생성시각을 대조했다. runtime cruise/curve/sharp/max_drive와 recommended speed는 기록 전 구간 모두 20이었다. cached 3.5 재발은 아니다.

## B. OBSTACLE_NOT_PASSED 코드 경로

수정 전 생성 지점은 snapshot `lattice_before.py`의 `plan()` 내부 **한 곳**이다.

```python
relevant = self._update_extent(...)
rear_passed = ego_s > rear_s + obstacle_rear_margin_m
if not relevant:
    if self._rear_point is not None and not rear_passed:
        return self._stop(..., 'OBSTACLE_NOT_PASSED')
```

`relevant`는 단순 obstacle count가 아니라 **현재 relevant group 중 기억한 객체와 연결되는 관측이 있는가**라는 bool이다. association에 실패하거나 해당 객체의 relevant 관측이 사라지면 false다. far cluster의 존재 자체는 이를 true로 만들지 않는다.

최초 해당 cycle 54의 복원 결과:

- active maneuver=true, 마지막 승인 목표=-1.8 m, 직전 유효 승인 Path는 cycle 50의 AVOIDING 경로다.
- 이전 `_maneuver`에는 world reference, 계수, 완료점, progress 상태가 있다. 없음/초기화 때문에 멈춘 것이 아니다.
- 현재 actual=-1.681895596862 m(ROS Float32=-1.681895613670), yaw≈73.906089°다.
- Ego s≈0, front=12.664767440528 m, rear=14.750210601433 m다. rear+0.5 m를 통과하지 않았다.
- 기억한 front world point: `(140.8704533229467, 1444.2708037895882)`, 관측 t=10.434380827 s.
- 기억한 rear world point: `(139.34497221433455, 1446.3489325381424)`, 관측 t=13.100264064 s.
- near cluster의 마지막 관련 관측 후보는 t=13.858158312 s다. 그 뒤 near 관측의 빈 배열/소실과 별도 far 관측이 확인된다. cycle 54 직전 관측 후보들의 reference lateral 거리는 약 **7.3~11.2 m**로 influence 2.5 m 밖이다.
- `_continuation_boundary()`는 실행되지만 `_adaptive_transition()` 후보 평가는 **0회**, collision 후보 검사도 실행되지 않은 채 `_stop()`으로 반환한다.

actual/approved/clearance/front/rear는 Float32 변환 시 bag readback과 동일함을 검증했다. candidate offset/curvature/entry의 NaN 및 reason NOT_EVALUATED도 재현했다. 증적: `replay_verification.json`, `reproduced_054.json`, `comparison.json`.

재현 한계도 분리했다. cycle 0은 bag 시작 약 0.351 s 전에 생성됐고 callback 큐 실행 로그가 없다. 직전 approved profile은 실제 cycle 50 Path에서 역산한 boundary이며 fitting residual은 약 **0.000054 m**다. 모든 private state나 정확한 callback 선택을 유일하게 복원했다고 주장하지 않는다. 다만 이 early-stop 분기는 profile의 세부 계수와 무관하게 위 observable 조건에서 발생하고, 공개 진단은 Float32 수준으로 정확히 일치한다. 직전 far 입력 후보도 모두 relevant=false라 이 분기의 결론은 같다.

## C. 원래 의도와 실제 동작 차이

기존 주석의 의도는 ‘ROI/occlusion 소실이 rear 통과 증거는 아니다’였다. 이 의도는 유지해야 한다. 그러나 실제 분기는 관련 관측 소실 + rear 미통과를 **RECOVERING 금지**와 **안전 AVOIDING 진행 금지**에 동시에 적용했다.

회피 위치에 가깝다는 사실만으로 안전하다고 판단할 수도 없다. 정지 구간의 Ego yaw는 road tangent와 약 16° 다르므로 offset 차이가 5 cm여도 현재 yaw/미분을 무시한 constant offset path는 연속 경계가 아니다. 따라서 관측 소실 상태의 HOLD는 기억한 occupied samples와 전체 현재 관측으로 다시 검사하도록 수정했다.

## D. 후보 평가 생략 이유와 최초 출발 오류

OBSTACLE_NOT_PASSED는 clearance/curvature 불합격 진단이 아니다. 위 `return _stop()`이 candidate list 생성보다 앞에 있어 후보가 아예 없었다. clearance=0은 실제 최소 거리를 계산한 결과가 아니라 early-stop에 넣은 값이다.

출발 직후의 첫 BLOCKED는 별도 numerical geometry 문제도 있었다. cycle 21의 동일 Ego/reference 투영은 **s=-2.04109e-13 m**였다. 이전 `_ego_reference()`가 모든 음수 s를 endpoint extrapolation으로 처리해 길이 약 **2.2737e-13 m**의 사실상 중복 segment를 prepend했다. `_normals()`의 degenerate 처리와 결합하면 시작 normal이 0이 되고, offset 경계가 깨져 실패 후보 곡률이 약 **3.950933 /m**로 커진다. cycle 22는 first segment 길이가 정확히 0으로 계산돼 tangent가 잘못되어 INVALID_HEADING_BOUNDARY로도 이어졌다.

수정은 기존 수치상 degenerate 길이 기준과 같은 **1e-9 m 이내 음의 투영만 s=0으로 정규화**하는 것이다. 의미 있는 음의 progress는 계속 extrapolate한다. 물리적인 steering/curvature gate나 clearance를 완화하지 않는다.

같은 cycle 21 bag-derived 입력의 frozen before/after 비교에서 전에는 failed -2.4 후보로 BLOCKED/κ≈3.950932, 후에는 -1.8 후보가 AVOIDING/κ≈0.159719로 gate를 통과했다. 이 비교의 footprint callback/초기 extent는 위 재현 한계 때문에 원 bag의 최초 clearance/front/rear와 완전히 동일하다고 주장하지 않는다. 중복 segment와 곡률 실패의 인과관계는 같은 입력을 고정한 비교로 분리했다.

## E. 수정한 상태 전이

```text
AVOIDING entry
→ approved target 부근
→ AVOIDING (internal maneuver_phase=HOLD)
→ rear+margin 실제 통과
→ RECOVERING
→ CLEAR
```

관련 관측 소실 + remembered rear 미통과에서도 candidate 평가로 들어간다. 이전 selected offset과 active 방향을 유지하는 HOLD를 먼저 시도하고, unsafe이면 기존 offset 후보들을 평가한다. 모두 실패하면 NO_SAFE_FEASIBLE_CANDIDATE/BLOCKED로 간다. BLOCKED에서 재출발할 때의 기존 3-cycle RELEASE_PENDING은 그대로 유지한다.

공개 `lattice_status`는 HOLD에서도 **AVOIDING**이며 FSM 변경은 없다. 정상 HOLD를 OBSTACLE_NOT_PASSED라는 정지 이유로 표시하지 않는다. source에서 이 정지 이유 생성 분기를 제거했다. 실제 heading/horizon 불능 또는 관련 footprint가 없는 불완전 imported history 등 입력 자체가 부족하면 기존 fail-closed 정책으로 정지하며, 이것을 정상 HOLD로 간주하지 않는다.

## F. HOLD 동작과 충돌 재검사

`abs(actual-selected) <= 기존 recovery_threshold_m(0.15)`이고 같은 방향이면 승인 목표 후보를 먼저 평가한다. threshold는 이번에 조정하지 않았다. 경로는 기존 Hermite generator로 실제 위치/yaw와 전달된 d''를 연결하며, 이전 suffix의 남은 world 완료 경계를 먼저 사용한다. 정확히 constant hold가 가능하면 flat하게 유지하고, heading/미분이 남으면 near-constant boundary transition을 사용한다.

HOLD도 clearance>=1.20, profile 및 world polyline curvature<=0.16046692048050848, entry deadline, driving direction, BLOCKED release gate를 통과해야 한다. 안전 HOLD만 prior approval을 유지하며 통과한다. HOLD 실패 후 다른 후보를 정상적으로 평가하고, 모든 후보가 실패하면 target=0이다.

관측이 사라진 near 객체를 무시한 채 far 점만으로 안전 판정을 풀지 않는다. 관련 객체로 연결된 world footprint samples를 기억하고, 관련 현재 관측이 없고 rear가 남아 있으면 **현재 전체 점 + 기억한 관련 footprint 점**을 collision 검사한다. exact duplicate만 제거하며 간격 downsample이나 raw-return 대체는 하지 않는다. 현재 associated 관측이 있으면 기존 current-observation collision 계약을 유지한다. rear/front의 world point 기억 및 매 cycle 재투영은 그대로다. far 점은 active extent/history에 합치지 않고 current collision set에는 계속 남는다.

기억은 명시 route/CLEAR reset 또는 rear 실제 통과 후 다음 객체로 교체할 때 해제한다. 장시간 소실의 geometry 검사는 보수적인 기존 관측에 근거하며, 관측하지 못한 동적 객체의 새로운 이동까지 증명하지 않는다. 다음 정지 검증에서는 관측 소실 시간과 history 크기/계산 시간을 함께 확인해야 한다.

고정 입력 재평가 결과:

| scene | 전 | 후 candidate 검사 | 후 clearance | 후 κ | 최종 결과 |
|---|---|---:|---:|---:|---|
| 최초 NOT_PASSED cycle 54 | 후보 0회, NaN | HOLD 1회/cycle | 1.213698948 m | 0.160466920096 | 동일 입력의 3-cycle 확인 후 AVOIDING -1.8 |
| 정지 cycle 389 | 후보 0회, NaN | HOLD 1회/cycle | 1.213698948 m | 0.160466920322 | 동일 입력의 3-cycle 확인 후 AVOIDING -1.8 |

기억 footprint는 callback 선택 불확실성을 보수적으로 덮기 위해 bag의 이전 near 관측들을 포함한 superset으로 검사했다. 이는 수정 후 실제 closed-loop 주행을 재현한 결과가 아니며, 실제 다음 bag의 시간축/20 km/h 주행 가능성을 보장하지 않는다. 기존 안전 한계에서 평가 가능한 HOLD가 있음을 보인 오프라인 결과다.

## G. Recovery gate

RECOVERING은 현재 관련 장애물이 없고, 기억한 rear가 있으면 **실제 Ego signed progress > rear_s + 기존 0.5 m margin**일 때만 진입한다. equality에서는 HOLD를 유지한다. 관측이 사라져도 rear memory를 지우지 않는다.

새 회귀는 rear 앞, rear+0.4, rear+0.5에서 AVOIDING을 유지하고 rear+0.6에서 RECOVERING을 확인한다. shortened/missing footprint 회귀도 rear=18 m 기억을 유지하면서 안전 HOLD를 허용하도록 바뀌었다. 종전 ‘무조건 BLOCKED’ 기대만 올바른 계약으로 수정한 것이다.

## H. 기존 unsafe BLOCKED fixture

이전 20 km/h 최초 BLOCKED/최종 진입/terminal fixture와 wide failed-path fixture를 포함한 회귀가 통과했다. 실제 clearance 부족 또는 조향 불가능 scene은 계속 BLOCKED/EMERGENCY_STOP/target=0이다. 수치상 중복점을 제거해도 clearance 부족을 안전 판정으로 바꾸지 않는다.

## I. 신규 regression

신규 테스트 11개(매개변수 scene 3개 포함)가 추가되어 이전 615 pass에서 626 pass가 됐다.

- actual=-1.75, approved=-1.8, rear ahead, 현재 far만 관측: 검사된 HOLD/AVOIDING.
- rear 미통과 동안 recovery 금지, equality에서 HOLD, margin 초과 후 RECOVERING.
- HOLD clearance 실패 후 같은 방향 다른 safe feasible candidate 선택.
- 모든 후보 실패 시 BLOCKED 및 Planning/FSM target/applied cap 0.
- 기존 BLOCKED에서 HOLD candidate가 평가되며 3-cycle release 확인 유지.
- 이미 관측한 unsafe footprint 소실 시 기억 충돌을 무시하지 않음.
- 실제 출발/최초 NOT_PASSED/정지 bag scene의 source 전후 비교 및 회귀.
- 음수 투영 roundoff로 중복 segment를 prepend하지 않음.
- 실제 Planning cycle의 safe HOLD에서 recommended/target=20, unsafe 후 target=0.

기존 runtime 3.5→readback 20→실제 output 20, actual/approved/failed diagnostics, moving Ego continuity, nonzero heading, far association/current collision, Planning/FSM/Control/Localization/Perception 회귀도 유지했다.

## J. 전체 테스트

전체 ROS source 및 기존 lint: **626 passed, 1 skipped, 2 warnings**. skip은 기존 copyright 검사, warnings는 logger의 entry point selectable deprecation이다.

```bash
cd /workspace/ros2_ws/src/edgenode_logger
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q /workspace/ros2_ws/src
```

기존 logger lint가 cwd 전체를 검사하므로 해당 package에서 전체 source path를 전달했다. 테스트 deselect/추가 skip은 없다. 증적은 `pytest.txt`다. ROS transport 테스트는 별도 namespace와 domain/localhost 환경에서만 실행했다.

## K. Build와 diff

`cd /workspace/ros2_ws && colcon build --symlink-install`: **7 packages finished**, 실패/경고 없음.

`git diff --check`: 통과. branch/HEAD는 유지했고 commit/push하지 않았다. 증적은 `build.txt`, `diff_check.txt`, `verification.json`이다.

## L. 다음 MORAI 정지 검증 절차

아래는 다음 작업용 절차이며 이번에는 실행하지 않았다.

1. 실제 차량 정지, enable_drive=false를 확인하고 기존 안전 수치와 ±2.4 설정을 읽는다. runtime 세 속도/max_drive/recommended=20을 확인한다. enable=false에서 target=0은 정상이다.
2. 이번에는 `/parameter_events`, `/planning/drive_status`, local_path_progress_m/local_path_index도 기록해 enable accepted 시각 및 road progress를 관측 가능하게 만든다. 기존 진단에 모든 evaluated candidate clearance/profile/polyline 토픽을 추가한다.
3. 정지 scene에서 actual≈-1.75, approved=-1.8 조건의 HOLD를 검사한다. yaw가 약 16° 어긋나 있으므로 offset 차이만으로 flat path를 승인하지 않는다. path 시작점/첫 heading/미분 연결과 두 곡률 gate를 확인한다.
4. near 관측이 빠지고 far 관측만 남는 입력을 오프라인 또는 정지 상태에서 확인한다. 기억한 near footprint와 current far가 모두 collision 검사에 남고, rear 미통과만으로 NOT_EVALUATED/BLOCKED가 되지 않아야 한다. 이전 BLOCKED 상태라면 RELEASE_PENDING 2회 후 세 번째 안전 확인에서 AVOIDING이 정상이다.
5. HOLD가 unsafe인 입력에서는 다른 후보가 평가되고, 모두 실패하면 FSM EMERGENCY_STOP/target=0/applied cap=0인지 확인한다. unsafe 데이터를 제거했다는 이유만으로 기억 충돌을 건너뛰지 않는지도 확인한다.
6. rear+margin 통과 여부는 bag replay/정지된 Ego 입력 실험으로 먼저 확인한다. equality에서 recovery하지 않고 strict 초과에서만 RECOVERING해야 한다. 실제 MORAI 전진 주행은 별도 승인된 단계다.
7. planning cycle 시간, input 생성→수신/실제 callback 지연, remembered sample 수를 측정한다. 이 bag에서는 AVOIDING 초기 cycle 간격이 약 0.3 s였으며 20 Hz timer 설정과 실제 처리율은 같지 않았다. Ego status와 odometry 속도 차이도 동일 timestamp로 대조한다. 이번에는 해당 모듈을 수정하지 않았다.
8. 실제 주행 전에는 앞 단계 보고서의 Sonata VehicleSpec/차체 기준점/solid occupancy/swept polygon/drivable boundary 요구사항을 유지하고 정지 수단·finite drive lease·collision 및 tracking 기록을 준비한다. 이번 frozen-input replay로 실제 20 km/h 주행 검증을 대체하지 않는다.
