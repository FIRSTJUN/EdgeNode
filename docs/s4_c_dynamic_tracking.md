# Phase C — LiDAR 이동 차량 추적 검증 완료

정의한 유한 MORAI 추적 조건7개를 동일 최종 소스로 통과했다. 기준 B 완료는 `000045f489f5c7045fcd9ce0247e0dde52a45dc6`, C 지문은 `6aa36ba2836584ea398c7fa056bca06c8785d94c9d179af6ea35be654957718d`다. 같은 `s4-finish`에서 v61/A/B 소스·프로필·기본 Launch·지도·저장 경로·18경유지를 보존했다. C 완료 커밋 SHA는 보존 묶음의 `commit-receipt.json`에 실제 생성 후 기록한다. D→E→F는 이 완료 커밋 이후 순서대로 실제 검증한다.

C 소스는 `ros2_ws/src/edgenode_dynamic_tracking/`, 설정은 `config/s4_c/`, 전용 Launch는 `s4_c_tracking.launch.py`, 결과는 `/workspace/local_data/s4_c_dynamic_tracking_20261119/`다. [기계 판독 근거](validation/s4_c_evidence.json)는 실제 bag·JSONL·실행 전후 소스·프로필·코드 지문·개별 안전 감사와 연결한다.

| 실제 최종 시험 | NPC 수 | 평가에 연결한 새 source 수 | 위치 오차 p95 m | 속도 오차 p95 m/s |
|---|---:|---:|---:|---:|
| c_v4_parked_adjacent_pass | 1 | 153 | 0.091 | 0.010 |
| c_v4_moving_npc10 | 1 | 92 | 0.378 | 0.183 |
| c_v4_stopped_npc10 | 1 | 74 | 0.164 | 0.230 |
| c_v4_other_speed15 | 1 | 54 | 0.347 | 0.520 |
| c_v4_two_ids | 2 | 53/53 | 0.176 / 0.190 | 0.301 / 0.238 |
| c_v4_actual_occlusion | 1 | 65 | 0.335 | 0.241 |
| c_v4_lane_follow | 1 | 62 | 0.185 | 0.280 |

A301/A302의 긴 다차로 직선, A632 native 구간을 사용했다. 정지 Ego/NPC10, 이동 Ego5/NPC10, NPC15, 동시2대, 실제 차폐 차량 가림, 다른 구간 차선 이동, 정차 차량 측면 통과를 구분한다. 이동 Ego 시험 속도는5km/h이며 C 자체의60km/h 성능으로 부르지 않는다. 프로필의 정상 목표60/급커브30과 실제 시험 속도를 구분한다. 정적 횡회피 탐색은 닫혀 있고 C 노드는 차량 명령을 발행하지 않는다. 실제 위험·차체·금지 B2·물리 C3·도로 포함·센서0.6초·안전 제동은 보존한 A NORMAL guard가 계속 검사한다.

각 시험의 충돌 메시지·실차체 B2/C3 접촉·도로 포함 미증명·비정상 Local Path 소실·armed 센서 HOLD는 모두0이다. 모든 소스 신선도와 실행 전후 파일 일치를 확인했다. 가림 차량의 실제 존재 시간은 약0.726초, 미검출 새 source 5개/0.407초에서 관측 시각을 유지하고 신뢰도를 줄였으며 같은 ID로 재관측했다. 후방 ROI를 벗어난 정차 차량은 오래된 추정을 usable로 유지하지 않고1.2초 안에 제거했다. 자세한 시간표는 `occlusion-analysis.json`, `expiry-analysis.json`이다.

621개 선정 회귀검사(v61559+A9+B39+C14)와2패키지 빌드가 통과했다. 완료된 A/B 전체 주행을 C의 추가 전체 주행으로 세지 않는다. 이 단계는 추적의 실제 검증이며 동적 추월 성능은 D 이후 평가한다.

최종 정지 수신 확인은2초 준비+6초 동안 LiDAR73개/새 시각73, GPS40개/새 시각40, IMU390개/새 시각335·중복55, 역행0이었다. 원시 source age 최대는 약0.011초, 최종 matched source age는0.090초였다. 실제 속도 약0.000009km/h·주행 비활성·센서 READY/latched reason 없음·제어 발행자1개를 확인했다. 재기동 초기 주행 비활성 상태의 과거 SENSOR_INPUT_TIMEOUT 사건은 삭제하지 않고 영수증에 유지한다. 현재 수신 복구와 armed 주행 HOLD0을 구분한다.

## 알고리즘과 관측 한계

시각/digest/session 검증된 `/perception/lidar_frame`의 실제 LiDAR 객체를 CV Kalman/Hungarian 방식으로 연결한다. Ego GPS·IMU·ROS 차량 속도의 보존한 B source-time Localization을 같은 관측 시각에 보간한다. ID·세계 위치/속도·상대 속도·방향·차선 추정·이동/정지/불확실 상태·미래1/2/3초 위치와 공분산·관측 시각·신뢰 점수·미검출/가림·만료를 `/dynamic_tracking/tracks`에 발행한다. heartbeat나 중복 source가 관측 시각과 누적 관측 수를 갱신하지 않는다. 신뢰 점수는 보정된 확률이 아니며 추정 차선은 차선 변경 허가가 아니다.

첫 정지 Ego 시험은 속도 오차는 작았지만 보이는 후면 중심과 차체 중심의 위치 차이가 p95 약2.495m였다. 첫 이동 Ego 시험은 차선 밖 구조물의 보이는 중심이 이동하면서 잘못된 MOVING p95=4가 발생해 실패로 보존했다. v3의 정차 차량 실제 측면 통과는 위치p951.097m/속도p950.607m/s/잘못된 MOVING24연결/ID분할로 실패했다. 같은 실패 조건을 최종 소스에서 다시 실제 통과했다. 이미 정지로 추정한 차량은 고정 차체 사전영역과 같은 관측 끝면을 비교하고, 같은 끝면0.45m 변위 또는 고정 영역+관측오차0.6m 이탈 시 고정 가설을 폐기한다. 같은 이전 ID의 분리 표면을 하나의 관측으로 결합한다. 고정 가설 반복으로 위치 공분산이 관측 분산 아래로 줄어들지 않게 유지하며 가설 여부/지원 오차 범위를 함께 발행한다. 이 모형 오차는 안전 충돌 여유가 아니며 움직임이 잠시 모호할 수 있어 원시 위험 검사를 계속 유지한다. 최종 모델은 원래 HDMap A2와0.05m 이내 정합한 native reference 근처의 교통 후보를 구분하고, 관측 면·이전 source 예측·명시한 승용차4.93×2.12m 사전값으로 위치 가설을 만든다. 원시 footprint와 사전값 여부를 함께 발행한다. 비차량·불충분 관측은 별도 불확실 관측이며 원시 Perception 장애물은 안전 판단에 그대로 남긴다. 임의 지도 공백 채우기나 Actor 정답 입력은 없다.

사전 차체 크기는 센서가 증명한 물리 외곽이 아니다. 트럭·부분 관측·임의 차종 전체의 정확도를 보장하지 않는다. D 이후의 간격·복귀 증명은 원시 관측·차체·미래 불확실성과 함께 검증해야 한다. 초기 거리 탐색에서60m 밖 지지가 줄었으므로 최종 정량 자격 범위는50m다. 감지되지 않은 공간을 비어 있다고 보지 않는다. 속도 오차/미래 위치 오차 전체 분포는 근거 JSON에 있다. 사후 정답의 마지막 시각에서 속도 보간에 필요한 미래150ms가 없어 발생한 미분류 마지막 source는 `c-terminal-truth-coverage.json`에서 속도 미분 없이 같은 시각의 차체 위치만으로 다시 대조했다. 정답 위치를 보간할 수 있는 구간의 미연결 MOVING은0이며 정답 범위 밖은 별도 집계한다.

Actor SDK는 실제 배치·속도 설정·복원과 종료 후 시각 정합한 정답 감사에만 사용했다. 추적/Planning에는 Actor 위치·속도·ID를 전달하지 않는다. SDK 시각은 microsecond→nanosecond로 변환하고 RPC 시각 차이를 검사했다. 초기 실패/오프라인 재생/단위 검사를 최종 실제 성공7개에 합산하지 않는다.

## 실행·재현·보존

기존 소유 스택을 주행 비활성·물리 정지로 종료하고 `/ctrl_cmd` 발행자0개를 확인한 다음 실행한다. 중복 기동하지 않는다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_c_tracking.launch.py enable_drive:=false \
  planning_profile_file:=/workspace/config/s4_c/planning_normal.yaml
```

실제5개 노드 파라미터·source READY·실제 코드 지문·제어 발행자1개를 확인한다. C에서 검증한 낮은 Ego 속도로 시작할 때는 `python3 tools/morai_drive.py start --normal-kmh 5 --sharp-kmh 5`, 정지는 `python3 tools/morai_drive.py stop`이다. C는 추적/NORMAL-only 실행 프로필이다. v61/A/B는 각 완료 문서의 독립 명령을 사용한다.

재빌드 시 실행 중인 소유 스택/작업자를 먼저 종료한 뒤 `cd /workspace/ros2_ws && colcon build --symlink-install --packages-select edgenode_dynamic_tracking edgenode_bringup`을 사용한다. 실주행 전에 실제 로드 경로와 지문을 다시 대조한다.

정확한7배치 인자는 결과 루트 `qualification-plan-v4.json`, 실행 감독은 `qualify-c-v4.py`, 개별 실행은 `python3 tools/s4_c_trial.py <해당 argv> --output <새 결과 폴더>`에 보존했다. 시험 도구는 배치·복원·사후 정답을 담당하며 알고리즘 입력과 분리한다. 가림/만료 추가 감사 도구도 Git에 보존한다. SDK 임시 의존성은 기존 v61 묶음 `final-v61-package/morai-trial-runtime.tar.gz`에서 복원하며 그 해시/지도/경로 의존성을 C 보존 index에 연결한다.

`final-c-package/`는 소스·도구 tar, 증거 index·해시, 완료 커밋 bundle과 receipt를 보존한다. 대용량 bag/지도/빌드는 Git에 무차별 추가하지 않는다. 원래 장면·주행 비활성·물리 정지·제어 발행자1개·새 센서 시각 수신을 최종 확인했다. 이력을 다시 쓰거나 push하지 않는다.
