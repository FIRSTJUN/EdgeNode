# Phase A NORMAL S자 추종 실험

v61 프로필을 덮어쓰지 않는 별도 실행이다. 기본 Launch는 보존되며 A는 `s4_a_scurve.launch.py`를 명시적으로 선택한다. 정상 속도/Control/Localization/Perception 값은 v61 검증 값과 같다.

현재 후보는 `planning-normal-only-v3.yaml`과 `edgenode_normal_experiment`다. 최초 기준용 `planning.yaml`과 중간 v1/v2 프로필은 재현 기록으로 보존한다. 프로필 이름만으로 소스 버전을 판단하지 말고 각 시험의 실제 모듈 SHA를 확인한다.

`static_avoidance_method=lattice`는 보존된 Planning 초기화용 값이다. 실행 전에 횡탐색기를 닫고 A 전용 NORMAL 안전 어댑터로 교체한다. 실제 모드는 `/planning/phase_a_runtime`, `selected_strategy=NORMAL_SAFETY_ONLY`, `static_avoidance_method=NONE_PHASE_A`로 확인한다. v61 기반 SHA와 A 확장 SHA를 구분한다.

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_a_scurve.launch.py enable_drive:=false \
  planning_profile_file:=/workspace/config/s4_a/planning-normal-only-v3.yaml
```

주행 전 현재 스택을 정지하고 제어 발행자 0개를 확인한다. 중복 기동하지 않는다. 새 스택 준비 후 제어 발행자 1개·source 신선도·센서 READY·실제 모듈 지문을 확인한다. A의 원본 결과는 `/workspace/local_data/s4_a_scurve_20261119`이며 완료 판정과 현재 한계는 `docs/s4_a_scurve.md`를 따른다.
