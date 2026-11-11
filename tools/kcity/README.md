# K-City bounded 검증과 비교

`live_test.py`는 기존 MORAI runner의 gate/lease/중단/정지 확인을 유지한 60/40/30 도구다.
기본 독립 상한은 20km/h다. `--prepare`는 drive를 활성화하지 않는다.
준비·주행에 같은 상한을 지정하며 준비 실패 뒤 주행하지 않는다.

```bash
python3 tools/kcity/live_test.py --prepare --max-speed-kmh 50 --output-dir local_data/my_case
python3 tools/kcity/live_test.py --run --seconds 120 --max-speed-kmh 50 --output-dir local_data/my_case
```

`--legacy-profile`은 60/30/25의 조향 비교용이다. 결과에 Planning/Control 설정,
Global Path 원본 스냅샷과 해시, 센서 header 시간, EKF·경로·조향 진단을 보관한다.
동일 도구의 중복 실행은 process lock으로 거부한다. 실제 주행 중단 기준은 소스의
`immediate_faults`, sustained checks와 `Session.ready/stop`에 있다.

`analyze_drive.py`는 JSONL을 원래 경로 호 길이로 정렬한다.
ACTIVE·양의 목표속도·실제속도 2km/h 초과 표본을 사용하며, 선택적으로 출발 후 일정 시간을 제외한다.
제어 오차와 MORAI 실제 위치의 선분 투영 오차를 별도로 기록한다.
구간 비교에는 원래 경로에서 0.75m 이내이고 진행 방향 차이가 30° 이내인 표본을 사용한다.
전체 주행의 실제 위치 오차는 각 시험에 저장된 실제 계획 경로에 대해 계산한다.
다른 차로/다른 진출 경로의 원래 기준선 거리를 추종 오차로 해석하지 않는다.

조향 변화율은 CtrlCmd header 시각(없으면 수신 시각)을 사용하고, 5–150ms 간격만 계산한다.
긴 공백은 포화 지속시간에 포함하지 않는다. 부호 반전은 ±.05 기준으로 반대 부호에
0.5s 이내 도달한 경우이며, 필요한 S자 방향 전환도 포함한다.
급회전은 기준 경로 315–340m, 회전교차로는 960–1090m다.
추가된 smoothing/slew의 정지 우회와 계산·분석 경계는 함께 테스트한다.

```bash
python3 -m pytest tools/kcity -q
python3 tools/kcity/analyze_drive.py --reference-path path.json \
  --case baseline .7 'local_data/baseline/drive_*.jsonl' \
  --case candidate .85 'local_data/candidate/drive_*.jsonl' \
  --exclude-startup-sec 3 --output local_data/comparison
```

`--max-lateral-accel-mps2 3.0`은 선택적인 실험 중단 감시다. 최신 실제속도와
IMU yaw rate로 `v * gyro_z`를 추정하고 한도 초과가 0.15초 지속되면 기존 정지
절차를 실행한다. 이 옵션을 사용할 때 신선한 IMU도 필수이며 속도 상한을
변경하지 않는다. 목표 횡가속도 1.5m/s²와 이 실험 중단 한도는 서로 다른 값이다.
추정치가 도로 좌표계 가속도나 타이어 한계를 보장하지는 않는다.

```bash
python3 tools/kcity/live_test.py --prepare --max-speed-kmh 60 --max-lateral-accel-mps2 3.0 --output-dir local_data/my_case
python3 tools/kcity/live_test.py --run --seconds 120 --max-speed-kmh 60 --max-lateral-accel-mps2 3.0 --output-dir local_data/my_case
```

로그에는 실제 PID raw/filtered D항, I항과 clamp 전 effort도 기록된다. 브레이크
전환을 비교할 때 경로 구간, 방향, 주행 시간과 ON/OFF 문턱을 맞춘다. 다른
접근 경로나 정지 후 transient를 섞은 전체 횟수는 직접적인 개선 지표로 쓰지 않는다.

`--lateral-budget-mps2`는 명시적인 MORAI 계획 예산 실험 옵션이다. 기본 동작은
기존 1.5m/s² 이하 검증을 유지한다. 실험 범위는 2.0 이하이며 1.5 초과를 선택하면
독립 IMU 중단 감시 3.0 이하가 필수다. prepare/run의 예산도 정확히 맞아야 한다.
이번 1.8 실험은 횡가속도 감시로 중단되어 기본 설정에 반영하지 않았으며,
운영 예산은 1.5로 복구했다.
