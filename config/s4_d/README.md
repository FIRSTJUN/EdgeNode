# D 전용 실험 프로필 — 아직 완료되지 않은 체크포인트

기존 v61/A/B/C 프로필과 기본 Launch를 덮어쓰지 않는다. 현재 D29는 첫20km/h 실제 추월·복귀와 안전 감사를 통과했고 반복 배치·정적 실제 회귀가 남아 있다. 상세 상태는 `docs/s4_d_dynamic_overtake.md`, 결과는 `/workspace/local_data/s4_d_dynamic_overtake_20261119/RESUME_STATE.json`을 확인한다.

`planning_20.yaml`은 NORMAL/SHARP/회피20km/h, `planning_15.yaml`은 각각15km/h다. 곡률·안전 추종·정적 전략은 이 목표보다 낮은 속도를 선택할 수 있다.15km/h에서는 지원하는 단일 native 쌍의 복귀 공간 부족에 대한 정상 추종을 별도 실제 시험했다. 센서 신선도0.6초·기존 차체·경계·도로·제동 검사는 유지한다. 현재 D 후방 입증의 교통 속도 가정은30km/h이며 F의60km/h 혼합 교통 검증을 대신하지 않는다.

정지·비활성 상태에서 기존 소유 스택/작업자 종료와 `/ctrl_cmd` 발행자0개를 확인하고 하나만 기동한다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_d_overtake.launch.py \
  enable_drive:=false \
  planning_profile_file:=/workspace/config/s4_d/planning_20.yaml
```

실제 설정·코드 지문·원시 센서 source 시각·주/보조 센서 READY·실제 정지·제어 발행자1개를 대조한 뒤 해당 시험만 수행한다. 실행 중 symlink 소스나 빌드 결과를 변경하지 않는다. 이 명령 자체를 D 완료·임의 도로의 성공 보장으로 해석하지 않는다.

MORAI에는 원본4센서를 그대로 보존한 `sensors_d_roof_hdl64_20261120.json`이 필요하다. 추가 ID5/HDL64/10Hz/x0,y0,z2.2m/회전0/`/lidar_rear/points`는 후측방 차체 높이대 광선 입증용이다. `perception_rear.yaml`은 별도 Perception으로 보조 충돌 조각을 전달한다. 원본 센서 내보내기는 `sensors_baseline_20261120.json`이다. `sensors_d_roof_candidate_16.json`은 폐기된 중간 후보이며 실행 프로필이 아니다.

Ubuntu 호스트의 프로젝트 루트에서 다음 도구는 기존 MORAI 센서 저장 폴더에 해시를 포함한 새 파일을 복사한다. 활성 장면이나 차량을 조작하지 않으며 다른 파일은 덮어쓰지 않는다.

```bash
python3 tools/s4_d_sensor_profile.py
```

출력한 `d_roof_hdl64`의 `load_basename`을 MORAI Sensor Edit에서 불러오고 시뮬레이터를 재개한다. SDK `LoadSensorFile`에는 절대 경로가 아닌 basename을 전달해야 한다. 원래 센서 구성으로 복원할 때는 같은 출력의 `original` 파일을 사용한다. 센서 구성과 원시 수신을 바꾼 시험 결과는 혼합하지 않는다.
