# 차선 후보 사전 생성 방식 비교 기록

2026-11-17 UTC / 2026-11-18 KST. 브랜치 `s4-finish`, HEAD `be6aaec`.

별도 선택 모드의 구현과 실제 MORAI 비교를 수행했다. **기본값은 `lattice`를 유지한다.**
새 방식의 차선 구간 연속성 개선은 확인했지만, 교차로 Offset의 실제 회피·복귀는 실패했다.
1·2단계 통합 안정화 완료로 판정하지 않으며, 3단계 동적 추월·합류 개발은 진행하지 않는다.

## 보존한 기준선

- 전원 종료 당시의 수정 20개/신규 14개는 `local_data/s4_resume_20261117_2128/initial-worktree-files.tar.gz`, `initial.diff`, `initial-manifest.json`에 보존했다.
- 새 방식 착수 직전의 변경 38개는 아래 실험 디렉터리의 `preserved-working-files.tar.gz`, `preserved-working.diff`, `preserved-file-hashes.json`에 다시 보존했다. 당시 기존 실험 파일 268개의 목록도 `preserved-experiment-files.json`에 기록했다. 기존 rosbag/JSONL은 원래 위치에 남아 있다.
- 이번 비교에서 “기존 방식”은 HEAD의 깨끗한 코드가 아니라, 보존한 미커밋 S4 개선 코드다. `baseline-planning.yaml`과 `native-planning.yaml`은 같은 센서/제어/안전 설정을 사용하며 선택 방식만 다르다. 두 방식 모두 continuous/integrated 실험 옵션을 켰다.
- 진행 중이던 진입 시점 최적화는 양쪽 모두 `avoidance_optimize_entry_onset=False`로 고정했다. 미완성 최적화의 효과를 새 구조의 효과로 섞지 않았다.
- 각 실험은 `source-snapshot.tar.gz`, 코드/파일 SHA-256, 파라미터, 실제 배치, 닫힌 rosbag, JSONL, 복원 결과를 포함한다. 커밋·reset·clean·강제 체크아웃은 수행하지 않았다.

실험 디렉터리: `/workspace/local_data/lane_candidate_comparison_20261117_230904/`.
종료 시 전체 변경 목록/해시는 `changed-files-final.json`, 전체 변경 백업은 `worktree-final.tar.gz`와 `worktree-final.diff`다.

## 별도 구현

[`lane_candidate_bank.py`](../ros2_ws/src/edgenode_planning/edgenode_planning/lane_candidate_bank.py)는 MGeo의 실제 동일 진행 방향 인접 관계와 통행 허가를 따라 후보를 만든다. 현재 차선을 포함하며 개수 5를 가정하지 않는다. 반대 방향, 명시적 금지, 연결 불일치는 제외한다. 자연 합류 또는 허용된 역방향 차선 전환의 증거도 준비한다.

[`lane_candidate_planner.py`](../ros2_ws/src/edgenode_planning/edgenode_planning/lane_candidate_planner.py)는 준비한 중심선에서 최신 LiDAR로 통행 가능한 차선을 선택하고, 기존 S4의 부드러운 진입·차체 sweep·Pure Pursuit rollout 검사를 거친다. 선택 후 차선 ID를 유지하며 중심선의 실제 연결을 따라 주행한다. 후보 갱신은 현재 선택 경로를 비우지 않는다. 새 장애물이나 차선 끝 때문에 복귀가 필요할 때 배경 복귀 탐색을 수행한다.

80m 원거리 장애물 미리보기는 별도 작업으로 옮겼다. 현재 차체, 정지/명령 구간, 실제 추종 rollout은 매 tick 최신 관측으로 검사한다. 원거리 결과는 조기 재계획 요청에만 쓰며 주행 허가는 주지 않는다. 센서 시간 한도 0.6초, 출처 시간 짝짓기, fault/restart 검사, 차체·금지 경계·도로 영역 검사를 유지했다. EKF, Pure Pursuit, PID, LiDAR 인지 기능도 유지했다.

차선이 모호한 교차로에서는 기존 Offset 탐색과 진입 전 복귀 가능성 검사를 호출한다. 이 경로가 호출된다는 오프라인 검증과 실제 주행 선택 로그가 있지만, 실제 Offset 회피 성공은 확보하지 못했다.

선택 파라미터는 `static_avoidance_method=lattice|lane_candidates`이며 시작 시 고정된다. [`s4_morai.launch.py`](../ros2_ws/src/edgenode_bringup/launch/s4_morai.launch.py)에 같은 플래그를 추가했다. `lane_candidates`를 고르면 continuous/integrated 옵션도 켜진다. 교차로 실험의 surface/unmarked 옵션은 별도이며, 실제 비교값은 보존 YAML을 기준으로 재현한다. 실행 중인 스택에 Launch를 중복 실행하지 않는다.

계산/발행 증거는 `/planning/lane_candidates`와 `/planning/avoidance_debug`에 기록한다. 실제 MGeo 후보 수는 이 구간에서 3개였다. 15km/h 및 다음 장애물 시험에서는 **장애물 설치 전** 준비 완료·3개 후보를 기록했다. Actor API는 독립 배치·복원·사후 측정에만 사용했고, 위치나 속도를 판단 알고리즘에 넣지 않았다.

## 실제 MORAI 비교

모든 비교는 같은 장애물 위치/방향, 출발점, 속도 설정을 사용했다. 기록된 장애물 위치 차이는 0m, 실제 출발 위치 차이는 최대 약 0.00074m였다. 센서 입력은 각 주행에서 실제 수신했으므로 동일 rosbag 재생 비교는 아니다.

아래 수치는 원시 rosbag 전체 메시지와 실제 EgoStatus 이동을 확인한 최종 집계다. HOLD는 상태에 진입한 횟수다. 같은 HOLD에서 이유가 바뀐 경우를 중복 계산하지 않는다. 정지는 진입 이후 1km/h 이상 움직인 뒤 0.2km/h 미만으로 내려간 횟수다.

| 배치 | 방식 | 센서 HOLD | Local Path 소실 | 실제 정지 | 최대 추종 오차(m) | 판정 |
|---|---|---:|---:|---:|---:|---|
| A301 시작10 / 장애물40 / 0° / 10km/h | 기존 | 32 | 27 | 20 | 0.176 | 연속성 실패 |
| 동일 | 새 방식 | 0 | 0 | 0 | 0.024 | 통과, 자연 합류 |
| A301 시작20 / 장애물55 / 12° / 10km/h | 기존 | 1 | 1 | 1 | 0.120 | 연속성 실패 |
| 동일 | 새 방식 | 0 | 0 | 0 | 0.022 | 통과, 자연 합류 |
| 같은 12° 배치 / S4 15km/h | 기존 | 13 | 10 | 9 | 0.146 | 연속성 실패 |
| 동일 | 새 방식 | 0 | 0 | 0 | 0.035 | 통과, 자연 합류 |
| A301 시작5 / 장애물85 / 0° / S4 15km/h | 기존 | 0 | 0 | 0 | 0.077 | 통과, 자연 합류 |
| 동일 | 새 방식 | 0 | 0 | 0 | 0.046 | 통과, 자연 합류 |
| A301 시작5 / 장애물60＋A302 장애물130 / 접근15→S4 10 | 기존 | 0 | 0 | 0 | 0.072 | 통과, 새 장애물 전 복귀 |
| 동일 | 새 방식 | 0 | 0 | 0 | 0.081 | 통과, 새 장애물 전 복귀 |
| A404 시작10 / 장애물45 / 10km/h | 기존·새 방식 각각 | 0 | 0 | 0 | 0.061 / 0.060 | 복귀 주행 완료, 도로 증거 일부 미확보 |

엄격한 기준을 모두 통과한 배치는 새 방식 5개, 기존 방식 2개다. A404 배치는 양쪽 모두 실제 차선 진입·복귀를 관측했으나 도로 증명 범위 밖인 표본 7개가 남아 엄격 통과에 넣지 않았다. 이것을 물리적 도로 이탈 7건으로 해석하지 않는다.

새 방식의 통과 5개 배치에서 MORAI 충돌 메시지, 실제 차체 금지 경계 접촉, 도로 영역 미포함, 센서 HOLD, 재검증 대기, Local Path 소실, 회피 중 0 목표 속도, 실제 정지는 모두 0이었다. 최소 실제 차체·장애물 간격은 0.873m 이상이었다. 선택 차선 전환은 확정 이후 0회였고, 해당 차선 유지 거리는 자연 합류 배치에서 약 112~148m였다.

새 방식이 모든 수치에서 더 좋지는 않았다. 두 장애물 배치에서는 기존 방식도 정지 없이 복귀했고 최대 추종 오차는 기존 0.072m, 새 방식 0.081m였다. 검출→확정 시간도 일부 배치에서 비슷하거나 새 방식이 조금 길었다. 이 실험 수로 전체 도로·센서 부하에서의 안정성을 보장하지 않는다.

## 진입 거리와 시간

후보 사전 생성과 실제 횡이동 시작은 분리해 기록했다. 거리는 실제 차체 앞끝과 장애물 뒤끝을 Global Path 진행 방향에 투영한 간격이다. 음수는 앞끝이 장애물의 뒤끝을 지나갔다는 뜻이며, 옆 차선에서의 충돌을 의미하지 않는다. 최소 간격은 별도의 실제 차체 polygon 간 거리다.

| 새 방식 배치 | 최초 LiDAR 검출(m) | 감속 cap 감소(m) | 최종 경로 확정(m) | 실제 횡이동 시작(m) | 실제 차선 획득(m) | 최소 간격(m) |
|---|---:|---:|---:|---:|---:|---:|
| 10km/h 기본 | 25.04 | 24.59 | 24.17 | 13.21 | -1.40 | 1.637 |
| 10km/h 12° | 29.85 | 28.49 | 26.82 | 17.21 | 2.86 | 0.889 |
| 15km/h 12° | 29.85 | 29.45 | 29.15 | 14.45 | -0.35 | 0.873 |
| 15km/h 충분한 접근 | 39.36 | 37.18 | 34.47 | 21.54 | 7.43 | 1.663 |
| 접근15→S4 10 / 다음 장애물 | 50.04 | 34.27 | 28.97 | 19.73 | 5.17 | 1.651 |

최초 검출에는 정지 중의 사전 관측도 포함된다. 충분한 접근 배치는 planner 검출 당시 실제 속도가 약 15km/h였고, 짧은 출발 배치는 가속 중이었다. 감속 cap의 첫 감소와 실제 속도 미분으로 측정한 감속 시작도 `entry_timing.json`에 따로 기록했다.

실제 횡이동 시작은 GPS Global Path 횡변위가 초기값보다 0.15m 넘게 변한 상태를 0.15초 유지한 시작점이다. 차선 변경 완료는 움직이는 실제 Ego가 선택한 MGeo 중심선 0.20m 이내·방향 오차 3° 이내를 0.30초 유지한 시점이다. Global Path 복귀도 같은 공간/방향 기준과 NORMAL·센서 준비·실제 이동을 함께 확인했다. 단순 상태 전환 또는 정지 Ego의 `success=True`를 성공 근거로 삼지 않았다.

차선 구간 planner 검출→최종 확정은 새 방식 약 1.04~1.27초였다. 접근15→10 배치는 감속 대기를 포함해 2.04초, 기존은 2.52초였다. 실제 횡이동 시작→차선 획득 시간, LiDAR 최초 인식→확정 시간, 각 시점의 실제 속도는 `comparison.csv`와 각 `entry_timing.json`에 있다. 15~25m 고정 시작 임계값은 추가하지 않았다. 중단한 진입 시점 최적화의 완성으로 판정하지 않는다.

## 다음 장애물과 정상 속도 복귀

새 방식 `native_lane_10_next_obstacle`에서 실제 접근 속도 약 14.89km/h일 때 감속을 요청했다. 회피 차선의 다음 장애물 간격 약 40.60m에서 `NEW_LANE_OBSTACLE_RETURN_SEARCH`를 요청했고, 배경 탐색 중에도 현재 경로와 10km/h 목표를 유지했다. 약 33.76m 차선 유지 후 명시적 복귀를 실행했다. NORMAL 목표는 15km/h로 바뀌었고 종료 전 실제 속도는 약 14.48km/h였다. 따라서 이 배치에서는 회피 후 10km/h cap이 남지 않았다. 복귀 시 실제 60km/h까지 가속한 시험은 아니다.

## 실패·미검증

- `native_offset_10_632`: 교차로 A632 시작2m / 장애물42m / 10km/h. 자동으로 `OFFSET_STATIC`을 선택했지만 `ROAD_AREA_UNPROVEN`으로 후보가 거절됐고, 계속 접근하다 `BRAKING_DISTANCE` 안전 정지했다. 실제 회피 진입은 없었다. 정지까지 포함한 최소 실제 차체 간격은 약 1.759m, 기록된 충돌 메시지는 0이었다. 성공으로 처리하지 않는다. 반복 접근/재탐색과 도로 증거 범위를 함께 분석해야 한다.
- A404의 두 방식 모두 복귀는 했지만 독립 도로 영역 감사에서 7개 표본의 지도 증거가 부족했다. 충돌·금지 경계 접촉은 관측되지 않았다. 도로 이탈 0을 완전히 입증한 배치에 넣지 않는다.
- `native_lane_10`: 잘못된 파라미터 파일 경로로 planner가 기동하지 못한 초기 시도다. 주행 비교에서 제외했다. 중복 노드는 없었고 물리 정지를 확인한 뒤 수정했다.
- `baseline_lane_15_approach`: 링크 시작점0m에서 인접 링크로 매칭돼 `PLANNING_NOT_READY`로 주행하지 못했다. 이후 사전 확인에 현재 링크와 READY/유효 상태를 추가하고, 두 방식 모두 시작5m로 시험했다.
- 교차로 회피/복귀 성공, 차선 기반↔Offset 통합 연속 주행, 전체 저장 미션, 다른 지도·기상·센서 부하, 동적 추월·합류는 완료하지 않았다. 과거 정지 Ego의 tracking-only 성공도 추월 성공으로 계산하지 않는다.

## 오프라인 검증과 감사 방법

- 최종 Planning 안전/노드/후보·선택 유지·배경 탐색·센서·도로 기하 검사: **210 passed**, `final-static-safety-tests.xml`.
- LiDAR 시간 짝짓기/수신 큐/인지 노드 검사: **44 passed**, `final-sensor-queue-tests.xml`.
- Planning/Bringup 빌드 성공: `build-native-candidates.txt`. 분석 도구 문법 검사와 `git diff --check` 통과.
- 이 결과는 오프라인 검증이며 위 실제 MORAI 결과와 별개다.

`s4_live_trial_report.py`는 닫힌 bag의 모든 collision/local-path/HOLD 메시지를 검사하며, 종료 후 물리 정지까지 충돌과 최소 간격을 확인한다. `s4_road_coverage_report.py`는 실제 EgoStatus 차체 sweep을 원본 A3 및 B2 경계의 실제 polygon과 비교한다. 미관측 구멍을 채우거나 도로 폭을 추정하지 않는다. MORAI Actor 차체/장애물 box는 사후 감사에만 사용한다. 이 넓은 감사 mask는 planner에 공급하지 않았다.

비교표는 `comparison.csv`, 사건/짝 비교는 `comparison.json`, 속도·실제 이동 궤적 그림은 각 `*-comparison.png`에 있다. 각 실험의 `live_audit.json`, `road_coverage.json`, `entry_timing.json`이 원시 로그와 측정 정의를 연결한다. 수정 전 파생 감사 결과도 `*-before-complete-stream-audit.json`으로 보존했다.

## 종료 상태와 다음 재개 지점

현재 노드는 MORAI/Localization/LiDAR/Planning/Control 각각 1개이며 `/ctrl_cmd` 발행자는 1개다. Planning의 실행 방식도 `lattice`로 돌려두었다. `enable_drive=False`, NORMAL 60 / SHARP_CURVE 30 / S4 10km/h, target 0, accel 0, brake 0.65, 센서 준비, 실제 정지를 확인했다. 임시 장애물은 모두 제거해 Actor는 원래 Ego만 남았다. 반복 시험의 작은 복원 오차 누적도 착수 전 보존 장면 기준으로 다시 복원했다(`scene-restoration-journal.json`, 마지막 위치 오차 약 0.0067m).

마지막 새 방식 성공 실험은 `native_lane_10_next_obstacle`이고, 마지막 비교 주행은 같은 배치의 `baseline_lane_10_next_obstacle`이다. 다음 작업은 A632 Offset의 도로 증거/진입·복귀 기하와 실패 후 접근 정책 분석이다. 차선 구간의 확인된 성공 배치를 이유 없이 전부 다시 시작하지 않는다. 새 방식의 본격 통합/기본 채택은 이 보고서 검토와 교차로 안정화 이후 판단한다. 그 전에는 3단계 실제 동적 추월·합류로 넘어가지 않는다.
