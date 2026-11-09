# 어린이보호구역 자율주행 시스템 개발일지

> **프로젝트:** EdgeNode — MORAI K-City · ROS 2 Humble · YOLO26 Nano  
> **기록 기간:** 2026년 10월 말 ~ 2026년 11월 10일  
> **작업 관점:** 인지(Perception) 담당자의 개발·디버깅 경험  
> **GitHub 브랜치:** [`FIRSTJUN/EdgeNode`의 `perception`](https://github.com/FIRSTJUN/EdgeNode/tree/perception)

## 기록 원칙

이 문서는 결과만 소개하는 README가 아니라, 개발 과정에서 **실제로 겪은 문제 → 원인 조사 → 적용한 수정 → 확인 결과 → 배운 점**을 기록한 개발일지이다. 사용자가 공유한 명령어와 로그, 개발 과정의 대화, GitHub의 실제 코드·커밋을 대조했다.

- **[실행 확인]**: 실제 터미널 출력이나 사용자의 직접 확인이 있었다.
- **[코드 확인]**: 현재 GitHub 브랜치에서 구현을 확인했지만 통합 실행 결과는 확인되지 않았다.
- **[검증 필요]**: 코드는 존재하거나 계획했으나 해당 단계의 성공 로그는 확보되지 않았다.
- 날짜를 특정할 수 없는 기록은 ‘11월 초’, ‘11월 8~9일’처럼 기간으로 표시했다. 확인되지 않은 수치나 성공 경험은 만들어 적지 않았다.

---

## 1. 개발 목표와 진행 개요

### 목표 시나리오

MORAI K-City에서 카메라로 어린이보호구역 표지판을 인식하고, ROS 2를 통해 인지 결과를 전달한다. 어린이보호구역에 진입하면 Planning의 최대속도 상한을 30km/h로 제한한다. 빨간 노면 관측 이후 회색 노면으로 전환될 때 종료 **후보**를 생성하고, 시뮬레이션용 별도 종료 확인을 거쳐 기존 속도 상한으로 복귀시키는 것을 목표로 한다.

### 개발 요소

| 계층 | 구성 | 역할 |
|---|---|---|
| 시뮬레이션 | MORAI K-City | 차량·도로·카메라 데이터 제공 |
| 인지 1 | YOLO26 Nano (`school_zone`) | 어린이보호구역 표지판 탐지 |
| 인지 2 | `school_zone_road_node` | 빨간색/회색 노면 분류 |
| 상태 관측 | `road_transition_detector` | RED → 지속 GRAY 전환 후보 검출 |
| 판단 | `school_zone_manager` | 진입·노면 확인·종료 후보·종료 확인 상태 관리 |
| 속도 제한 | `school_zone_speed_limit_node` | Planning의 최대속도 상한 제어 |
| 주행 | Localization / Planning / Control | 위치·경로·조향 및 속도 명령 |
| 검증 | RViz, rqt_image_view, `ros2 topic` | 경로와 탐지 화면, 토픽, 상태 점검 |

### 단계별 진행 경과

| 시기 | 핵심 활동 | 확인 수준 |
|---|---|---|
| 10월 말~11월 초 | MORAI K-City 경로·주행 기반 검토 | 코드/커밋 및 일부 실행 로그 |
| 11/7 전후 | MGeo, GPS/Localization, RViz, Route Editor 문제 해결 | 일부 실행 확인 |
| 11/7~8 | 빨간/회색 노면 및 어린이보호구역 상태 관리 | 코드 확인 |
| 11/8 | 30km/h 제한·종료 후 속도 복구 로직 검토 | 일부 파라미터 출력 확인 |
| 11/8~9 | 경로 재활성화 오류 수정 | 오류 및 Git 커밋 확인 |
| 11/8~9 | 표지판 라벨링·YOLO 모델 준비 | 자료/모델 존재 확인 |
| 11/9 | Python 가상환경·PyTorch·CUDA 문제 해결 | 실행 확인 |
| 11/9 | MORAI 카메라 + YOLO + rqt 시각화 | 실행 및 육안 확인 |
| 11/9 | 최근 5회 중 3회 + 0.80 이상 1회 판별 로직 | 코드 확인 |
| 11/9~10 | GitHub 커밋·Push 및 VS Code 분석 경고 해결 | 원격 커밋·사용자 확인 |

---

## 2. 시행착오 상세 기록

### 01. K-City 지도와 경로 편집기 준비 — 11월 7일 전후

**목표:** MORAI K-City에서 RViz로 경유지를 지정하고 Planning이 경로를 계산하도록 구성.

**문제:** 초기에는 Planning에 필요한 K-City MGeo 지도가 준비되지 않았다. 시뮬레이터를 실행하는 것과 ROS 2에서 지도 노드·링크를 이용할 수 있는 것은 별개였다.

**조치:** SHP 기반 지도 자료를 준비하고 MGeo 작업 경로를 `/workspace/local_data/kcity_mgeo`로 설정했다. Route Editor를 실행해 지도 로딩과 경로 편집 상태를 확인했다.

**결과 [실행 확인]:** Route Editor에서 **712 nodes, 809 directed links**를 읽었다는 로그를 확인했다. 이후 Planning에서 `/planning/map_match_status=MATCHED`, `/planning/state=GLOBAL_PATH_READY` 상태가 확인된 시점이 있었다.

**배운 점:** 지도 파일 존재 여부, 그래프 로딩, 지도 매칭, 전역 경로 계산은 각각 다른 성공 조건이다.

### 02. 위치 정보와 ROS 2 QoS가 맞지 않는 문제 — 11월 7일 전후

**문제:** `/ego_vehicle_status`에서 차량 위치(예: x≈16.18, y≈1104.62)를 수신했지만 초기에 `/localization/odometry`에서는 메시지가 나오지 않았다. `/gps`에 QoS 호환성 경고도 있었다.

**시도:** 입력 토픽 존재 여부, 메시지 타입, QoS를 확인했다. MORAI GPS·차량 상태를 이용해 `/localization/odometry`를 발행하는 별도 시험 스크립트 `scripts/morai_localization_test.py`를 활용하는 방향으로 작업했다.

**결과:** MORAI 차량 상태와 Planning 지도 매칭 관련 일부 신호는 확인했다. 전체 Localization 오차나 모든 데이터 흐름의 정상 동작이 정량 검증된 것은 아니다.

**배운 점:** ROS 2에서는 토픽이 목록에 나타나는 것과 실제 Subscriber가 QoS 호환 상태로 메시지를 수신하는 것이 다르다.

### 03. RViz 경로가 보이지 않고 차량이 제한 시간 뒤 멈춤 — 11월 7일 전후

**문제:** RViz 검은 화면, 마커 중복·경로 표시 문제를 겪었다. Planning의 주행 상태가 `DISABLED → ACTIVE → EXPIRED`로 변하고, 짧게 주행한 후 차량이 정지했다.

**시도:** RViz의 Fixed Frame을 `map`으로 맞췄다. Planning의 `enable_drive` 설정과 주행 시간 제한 파라미터를 확인했다. 목표 속도 및 브레이크 상태도 토픽으로 확인했다.

**결과 [실행 확인]:** 약 **3km/h로 15초 주행한 뒤 시간 만료로 정지**한 시험이 있었다. 당시 `drivable_drive_time=15.0`이 확인되었다. 장시간 연속 주행은 별도로 검증해야 한다.

**배운 점:** 차량 정지에는 경로 미설정, 지도 매칭 실패, 주행 비활성화, 시간 제한 만료, 목표 속도 0, 제어 명령 등 여러 원인이 있다. 한 번에 하나씩 확인해야 한다.

### 04. 노면 색깔만으로 보호구역 종료를 단정할 수 없음 — 11월 7~8일

**목표:** 표지판 인식 후 빨간 노면을 확인하고, 빨강에서 회색으로 바뀌는 상황을 종료 판단에 활용.

**구현 [코드 확인]:**

- `school_zone_road_node`: 카메라의 전방 중앙 ROI를 HSV로 처리해 `RED`, `GRAY`, `UNKNOWN`을 분류하고, 최근 7개 분류 중 5개 일치 조건으로 안정화한다.
- `road_transition_detector`: RED 관측을 누적한 뒤 GRAY가 기본 **1.5초 동안 연속** 유지되면 `RED_TO_GRAY` 후보 이벤트를 발행한다.
- `school_zone_manager`: `NORMAL → SCHOOL_ZONE_WAITING_FOR_RED → SCHOOL_ZONE_RED_CONFIRMED → EXIT_CANDIDATE → EXIT_CONFIRMED` 상태를 관리한다.

**시행착오의 핵심:** 회색 노면은 어린이보호구역 안에서도 나올 수 있다. 따라서 `RED_TO_GRAY`는 **종료 확정이 아니라 종료 후보**로만 취급해야 한다. 현재 구현의 `sim_auto_confirm_on_gray:=true` 옵션은 **MORAI 데모 전용**이며 실제 도로에서의 합법적 보호구역 종료 증거가 아니다.

**배운 점:** 시각 정보의 관측과 규칙적·안전상 판단을 분리해야 한다.

### 05. 30km/h 제한은 걸리는데 종료 후 복구되지 않음 — 11월 8일

**문제 [실행 확인]:** 보호구역 활성화 이후 Planning의 `max_drive_speed_kmh=30.0`이 확인되었고, 상태가 `EXIT_CONFIRMED`와 `active=false`로 변한 뒤에도 상한이 **30.0**인 상황을 겪었다.

**조사:** 복구는 단순히 `active=false`만으로 실행되지 않는다는 점을 파악했다. 현재 속도 제한 어댑터는 정상 구간에서 사전 관측한 기준 상한(`baseline_cap`), 실제 제한 적용 여부, 상태 전환 이력, 명시적 복구 허용, 차량 주행 활성화 상태 등을 확인한다.

**조치 [코드 확인]:** `allow_confirmed_restore` 및 시뮬레이션용 `sim_allow_moving_restore` 옵션, 기준 속도 복구 절차를 점검하고 Planning의 실행 중 속도 파라미터 반영도 수정했다.

**검증 수준:** 당시 30km/h 상한과 복구되지 않은 상태는 관찰됐지만, **복구가 실패한 직접 원인이 어떤 가드였는지는 해당 출력만으로 확정할 수 없다.** 새로운 코드가 실제 차량의 전체 30→50km/h 복귀 주행을 성공했다는 로그도 아직 없다.

**배운 점:** 감속과 속도 복구는 서로 다른 안전 수준을 요구한다. 정상화 신호 하나로 자동 가속을 허용해서는 안 된다.

### 06. 복구 허용 상한과 목표 속도를 혼동할 위험 — 11월 8일

**문제:** 초기 시뮬레이션용 복구 로직이 **40km/h를 넘는 기준 상한**의 복구를 막아, 50km/h 정상 구간으로 되돌아가는 데모 목표와 충돌했다.

**해결 [코드 확인]:** 고정 판단 대신 `max_sim_restore_kmh` 파라미터를 사용하도록 수정했다. 기본값은 **60.0km/h**로 확인된다.

**중요:** `max_sim_restore_kmh=60.0`은 **60km/h 주행 명령이 아니라 복구를 허용할 수 있는 상한값**이다. 이전에 확인한 기준 상한이 50km/h이면 복구 목표는 50km/h다.

**배운 점:** ‘속도 상한’, ‘주행 목표 속도’, ‘차량 실제 속도’는 서로 다른 지표다.

### 07. 저장된 경로를 재활성화할 때 파라미터 오류 — 11월 8~9일

**문제 [실행 확인]:** `/route_editor/load`는 성공했지만 `/route_editor/activate`에서는 다음 메시지가 나왔다.

```text
ACTIVATION_REJECTED: ('Invalid access to undeclared parameter(s)', [])
```

**원인:** Route Editor가 Planning에 `route_activation_id`를 전달하는 반면, 당시 Planning 코드에서는 해당 파라미터를 선언하지 않았다.

**해결 [코드 확인]:** Planning에 `('route_activation_id', '')`를 선언하고, 경로의 식별 키에 이 값을 포함해 **동일한 경로를 다시 활성화해도 새 활성화로 인식**하도록 수정했다.

**근거 커밋:** [`011ad17`](https://github.com/FIRSTJUN/EdgeNode/commit/011ad17ea4ad2ce0ce1cf5822b37f4322c99bc93)

**배운 점:** 노드 간 파라미터 인터페이스가 어긋나면 경로 데이터 자체가 정상이어도 활성화가 실패할 수 있다.

### 08. 124장 라벨링 후 Roboflow 학습 크레딧 부족 — 11월 8~9일

**목표:** MORAI K-City 이미지에 등장하는 어린이보호구역 표지판을 인식하는 전용 YOLO 모델 제작.

**진행 [화면 확인]:** Roboflow의 `school_zone` 데이터셋 **124장**과 **v2** 버전을 준비했다. 당시 표시된 분할은 Train **87장**, Validation **25장**, Test **12장**이었다. RF-DETR Small 대신 YOLO26 Nano를 선택했다.

**시행착오:** 학습 증강 설정에서 표지판의 글자/기호를 비현실적으로 뒤집을 수 있는 Horizontal Flip 사용을 재검토했다. Roboflow에서 학습을 시작하려 했지만 **크레딧 부족**으로 막혔다.

**대응:** YOLO 형식 데이터셋을 외부 학습 환경에서 활용하는 방안을 안내받았고, 이후 `best.pt` 파일을 준비했다.

**중요한 사실 구분:** 모델 파일을 확보한 사실은 확인됐지만, **크레딧 부족 이후 실제 어떤 환경에서 최종 학습했는지, mAP나 학습 손실은 얼마였는지**는 현재 확보한 기록으로 확정할 수 없다.

**배운 점:** 라벨링 완료 ≠ 모델 학습 완료 ≠ 실제 시뮬레이터 성능 검증.

### 09. YOLO 가중치가 있어도 실행용 라이브러리가 없음 — 11월 9일

**문제 [실행 확인]:** `/workspace/local_data/models/school_zone/best.pt` 파일은 약 **5.2MB**로 존재했지만, Python 모듈 확인 결과는 다음과 같았다.

```text
ultralytics: NOT INSTALLED
torch: NOT INSTALLED
cv2: OK
numpy: OK
rclpy: OK
```

**대응:** 기존 ROS 2 Python 패키지와 충돌할 위험을 줄이기 위해 `/workspace/local_data/yolo_venv`에 별도의 Python 가상환경을 구축하기로 했다.

**환경 확인:** Python **3.10.12**, NVIDIA **RTX 4050**, 드라이버 **580.178.04**, `nvidia-smi`에 표시된 CUDA 지원 버전 **13.0**을 확인했다.

**배운 점:** 모델 파일은 추론 실행 환경을 포함하지 않는다. ROS 2와 딥러닝 패키지의 설치 경로를 분리해서 관리해야 한다.

### 10. 가상환경 생성 실패: `python`, `ensurepip`, sudo — 11월 9일

**발생한 오류 [실행 확인]:**

```text
bash: python: command not found
bash: /workspace/local_data/yolo_venv/bin/activate: No such file or directory
The virtual environment was not created successfully because ensurepip is not available.
[sudo] password for dev:
Sorry, try again.
```

**원인 분석:** Dev Container의 기본 환경에는 `python3`만 있었고, `venv`에 필요한 `ensurepip` 구성요소가 없었다. 컨테이너 사용자 `dev`의 sudo 인증은 호스트 Ubuntu 로그인 계정과 별개일 수 있었다. 가상환경 생성에 실패해도 일부 심볼릭 링크는 남아 있을 수 있어 `bin/python3` 존재만으로 성공이라고 볼 수 없었다.

**대응:** 관리자 권한을 요구하지 않는 `virtualenv` 사용을 검토·적용하여 가상환경을 구성했고, 이후 YOLO 설치를 진행했다.

**최종 확인 [실행 확인]:**

```text
PyTorch: 2.9.1+cu128
OpenCV: 4.11.0
ROS 2: OK
CUDA: True
클래스: {0: 'school_zone'}
```

**배운 점:** 설치 성공 기준은 폴더나 명령어의 존재가 아니라 **실제 패키지 import, CUDA 인식, 모델 로딩**이다.

### 11. 카메라 주파수 명령어의 미지원 옵션 — 11월 9일

**문제 [실행 확인]:**

```bash
ros2 topic hz /camera/image/compressed --qos-reliability best_effort
```

위 명령에 대해 `ros2: error: unrecognized arguments: --qos-reliability best_effort`가 출력됐다.

**해결:** 사용 중인 ROS 2 Humble CLI에서 지원하지 않는 옵션을 제거했다.

```bash
ros2 topic hz /camera/image/compressed
```

**결과 [실행 확인]:** `average rate: 29.762`, `29.764` 등 **약 29.76Hz 카메라 수신**이 확인됐다. 이는 입력 카메라 메시지 속도이지 YOLO의 추론 FPS는 아니다.

**배운 점:** QoS 설정과 `topic hz`의 지원 옵션을 구분하고, 실제 사용하는 ROS 2 배포판에서 명령어를 확인해야 한다.

### 12. 탐지 로그만으로는 실제 표지판 인식 여부를 알 수 없음 — 11월 9일

**문제:** 초기 YOLO 테스트의 `SCHOOL ZONE DETECTED` 로그만으로는 **어떤 물체를 인식했는지** 판단하기 어려웠다.

**대응:** 결과 이미지를 `/perception/school_zone/yolo_debug`로 발행하고, Ubuntu 호스트의 `rqt_image_view`에서 이 토픽을 선택해 탐지 박스를 직접 확인했다.

**결과 [사용자 육안 확인]:** 실제 어린이보호구역 표지판에 박스가 표시되어 **잘 인식되는 것으로 보인다**고 확인했다.

**미검증:** 정량 Precision/Recall/mAP, 최초 탐지 거리, 오탐률, 야간·우천 성능은 별도 기록이 없다.

**배운 점:** **터미널 로그 → 이미지 시각화 → 정량 성능 평가**를 각각 구분해야 한다.

### 13. 빠른 주행에서 표지판을 너무 짧게 관측하는 문제 — 11월 9일

**문제:** 처음에는 최대 2FPS(0.5초마다 한 번)로 모델을 추론했기 때문에 차량이 표지판을 빨리 지나가면 충분히 많은 탐지 샘플을 얻기 어렵다는 우려가 생겼다.

**변경 요구사항:**

1. 최근 **5번의 추론** 중 **3번 이상** 표지판 탐지(각 Confidence ≥ **0.50**).
2. 그 3번 이상의 탐지 중 **최소 1번**은 Confidence ≥ **0.80**.
3. 최대 **5FPS**가 되도록 추론 시작 간격을 **0.2초**로 축소.
4. 조건 충족 시 `/perception/school_zone/sign_detected`에 `True` 이벤트를 **1회** 발행.
5. **10회 연속 미탐지** 이후 재발행 허용.

**현재 구현 [코드 확인]:** 이 조건은 `ros2_ws/src/edgenode_perception/edgenode_perception/school_zone_yolo_node.py`에 구현되어 있다. 반면 `tools/school_zone_yolo/school_zone_yolo_bridge.py`는 2FPS이며 0.80 고신뢰 조건이 없는 **이전 버전의 보관본**이다.

**주의:** “최대 5FPS를 목표로 시작 간격을 설정했다”는 의미이지, 실제 GPU 추론 성능이 5FPS로 측정됐다는 뜻은 아니다. Confidence 0.80도 정답 확률이 정확히 80%라는 의미가 아니다.

**배운 점:** 지나친 신뢰도 기준은 오탐을 줄이지만, 빠른 주행에서는 탐지 확정이 늦어질 수 있다. **감속 시작 거리**와 함께 검증해야 한다.

### 14. YOLO 인식과 자동 감속의 연결은 별도 검증이 필요 — 11월 9일 이후

**현재 코드 흐름 [코드 확인]:**

- YOLO → `/perception/school_zone/sign_detected` (`std_msgs/Bool`).
- `school_zone_manager` → `/school_zone/active`, `/school_zone/state`.
- `school_zone_speed_limit_node` → Planning의 `max_drive_speed_kmh` 상한 관리.
- 종료 시에는 과거 RED 관측과 `EXIT_CANDIDATE`, `EXIT_CONFIRMED`, 복구 안전 조건을 추가로 요구한다.

**확인 수준:** 과거 **수동 인식 신호 기반 30km/h 상한 설정**과 **YOLO 탐지 화면**은 각각 확인했다. 하지만 새 YOLO 노드가 발행한 이벤트로 차량이 실제 주행 중 **50→30→50km/h를 연속 완주**했다는 실험 로그는 아직 없다.

**배운 점:** 인지, 상태 전환, Planning 파라미터, 차량 실제 속도를 **서로 독립된 측정 항목**으로 다뤄야 한다.

### 15. Git Push 도중 SSH 실행 오류 — 11월 9일

**오류 [실행 확인]:**

```text
error: cannot run ssh: No such file or directory
fatal: unable to fork
```

**대응:** Git Remote 주소와 인증 방식을 확인하고, Dev Container에서 SSH 실행이 어려운 경우 HTTPS Remote를 사용하는 방법을 검토했다. 수정 파일을 선별적으로 Stage하도록 정리했다.

**최종 확인 [GitHub 확인]:** 원격 `perception` 브랜치에서 다음 커밋을 확인했다.

- [`011ad17`](https://github.com/FIRSTJUN/EdgeNode/commit/011ad17ea4ad2ce0ce1cf5822b37f4322c99bc93): `fix: update planning and school zone control`
- [`ad98612`](https://github.com/FIRSTJUN/EdgeNode/commit/ad98612bfab29b60f6fe14e8be2ac82bd0cbdd4d): `feat: integrate YOLO school zone detection and update perception`

**배운 점:** Commit, Push, GitHub 반영, 로컬 미커밋 변경 여부는 각각 따로 점검해야 한다. 강제 Push로 문제를 우회하지 않는다.

### 16. Git과 무관한 VS Code의 `ultralytics` 주황색 경고 — 11월 9~10일

**문제:** `school_zone_yolo_node.py`의 `from ultralytics import YOLO`에 주황색/노란색 경고가 남았다. Git 변경 표시로 오해하기 쉬웠다.

**해결 [실행 확인]:** VS Code의 **Python: Select Interpreter**에서 `/workspace/local_data/yolo_venv/bin/python`을 선택했고, 사용자가 경고가 사라졌다고 확인했다.

**배운 점:** 에디터가 선택한 인터프리터에서 import를 찾지 못하는 문제와 실제 실행 오류, Git 상태는 서로 다르다. 인터프리터 선택 자체로 이미 작성한 소스 코드나 원격 커밋이 바뀌지는 않는다.

---

## 3. 확인된 결과와 미검증 항목

| 항목 | 상태 | 근거/주의 |
|---|---|---|
| MORAI 카메라 `/camera/image/compressed` | **실행 확인** | 약 29.76Hz 수신 |
| RTX 4050의 PyTorch CUDA 사용 | **실행 확인** | `CUDA: True`, PyTorch `2.9.1+cu128` |
| `best.pt` 로딩 및 `school_zone` 클래스 | **실행 확인** | `{0: 'school_zone'}` |
| rqt에서 실제 표지판 탐지 박스 | **육안 확인** | 사용자가 정상 탐지 확인 |
| 5회 중 3회 + 0.80 이상 1회 로직 | **코드 확인** | 최신 `school_zone_yolo_node.py` |
| 새 YOLO 노드의 `data: true` 발행 | **검증 필요** | 실측 토픽 출력 기록 없음 |
| RED/GRAY 노면 분류 | **코드 확인** | HSV ROI 및 이력 기반 |
| RED→GRAY 종료 후보 생성 | **코드 확인** | 기본 연속 회색 1.5초 |
| School Zone Manager 상태 전환 | **코드 확인** | 상태 머신 존재 |
| Planning 30km/h 상한 적용 | **일부 실행 확인** | 과거 파라미터 출력 |
| 종료 후 기준 상한 복구 | **코드 확인** | 안전 조건·옵트인 구현 |
| YOLO 자동 진입→실제 감속→속도 복구 | **검증 필요** | 연속 주행 로그 미확보 |
| 실제 도로에서 구역 종료 판단 | **검증되지 않음** | 단순 색상 전환 사용 불가 |

---

## 4. 후속 통합 실험 계획

다음 항목은 **앞으로 수행할 계획**으로, 성공했다고 기록하지 않는다.

1. 정지 상태에서 최신 `school_zone_yolo_node` 실행 후 `/perception/school_zone/sign_detected`에 `data: true`가 발행되는지 검증한다.
2. 서로 다른 표지판·거리·조명에서 오탐/미탐과 Confidence 변화를 기록한다.
3. 실제 YOLO 처리 FPS와 첫 인식 **거리·시각**을 측정한다. 설정상 5FPS가 실제 성능을 보장하지는 않는다.
4. `school_zone_manager`가 `NORMAL → WAITING_FOR_RED → RED_CONFIRMED → EXIT_CANDIDATE → EXIT_CONFIRMED`로 진행하는지 각 토픽을 통해 확인한다.
5. Planning의 `max_drive_speed_kmh` 변화와 MORAI 차량의 **실제 속도 변화**를 별도로 기록한다.
6. `allow_confirmed_restore`, `sim_allow_moving_restore`, `max_sim_restore_kmh` 등의 조건이 시뮬레이터에서 의도대로 작동하는지 확인한다.
7. 보호구역 안에서 회색 노면이 나타나는 반례, 카메라 가림, 동일 표지판의 재탐지를 시험한다.
8. 실제 주행에서 사용할 수 있는 종료 검증 근거(지도/위치/도로 표지 등)를 별도로 설계한다.

### 후속 실험 기록 양식

| 항목 | 기입할 값 |
|---|---|
| 수행 날짜 및 Git SHA | |
| MORAI 맵/시작 위치 | |
| 모델 및 conf/추론 주기 | |
| 최초 표지판 검출 거리·시각 | |
| `sign_detected` 이벤트 시각 | |
| `/school_zone/state` 변화 | |
| Planning 상한 변화 | |
| 실제 차량 속도 변화 | |
| 성공/실패와 근거 | |
| 다음 수정 내용 | |

---

## 5. 관련 코드 및 주요 커밋

**주요 코드**

- [YOLO 표지판 인식 노드](../../ros2_ws/src/edgenode_perception/edgenode_perception/school_zone_yolo_node.py)
- [노면 색상 인식 노드](../../ros2_ws/src/edgenode_perception/edgenode_perception/school_zone_road_node.py)
- [노면 전환 감지 노드](../../ros2_ws/src/edgenode_perception/edgenode_perception/road_transition_detector.py)
- [어린이보호구역 상태 관리 노드](../../ros2_ws/src/edgenode_perception/edgenode_perception/school_zone_manager.py)
- [속도 제한 및 복구 노드](../../ros2_ws/src/edgenode_planning/edgenode_planning/school_zone_speed_limit_node.py)
- [Planning 노드](../../ros2_ws/src/edgenode_planning/edgenode_planning/planning_node.py)
- [YOLO 실행 참고](../../tools/school_zone_yolo/README.md)

**확인된 GitHub 커밋**

| SHA | 메시지 |
|---|---|
| [`206060a`](https://github.com/FIRSTJUN/EdgeNode/commit/206060af721a1e361e011f5880bb6a5d83b90421) | `feat: preserve MORAI school-zone automatic speed control v1` |
| [`04bbd79`](https://github.com/FIRSTJUN/EdgeNode/commit/04bbd79e353749b12412727cdb26b5ea41b41e98) | `fix: apply runtime planning speeds and configure MORAI restore cap` |
| [`011ad17`](https://github.com/FIRSTJUN/EdgeNode/commit/011ad17ea4ad2ce0ce1cf5822b37f4322c99bc93) | `fix: update planning and school zone control` |
| [`ad98612`](https://github.com/FIRSTJUN/EdgeNode/commit/ad98612bfab29b60f6fe14e8be2ac82bd0cbdd4d) | `feat: integrate YOLO school zone detection and update perception` |

---

## 6. 전체 회고

1. **문제를 기능·토픽 단위로 분리할 것.** 카메라 수신, 모델 탐지, 상태 전환, 속도 상한, 실제 차량 속도는 다른 문제다.
2. **경로·주행 실패 원인을 구분할 것.** `enable_drive`, 지도 매칭, 목표 속도, 주행 시간 제한을 각각 확인해야 한다.
3. **상태 복구는 감속보다 보수적으로 설계할 것.** 회색 노면 하나로 법적 보호구역 종료를 단정해서는 안 된다.
4. **탐지 로그와 눈으로 확인한 결과, 정량 성능은 다르다.** rqt 시각화가 중요했고, 정확도 평가는 추가 과제다.
5. **ROS 2와 딥러닝의 실행 환경을 구분할 것.** Dev Container, 가상환경, CUDA, VS Code 인터프리터는 각자 다른 설정을 가진다.
6. **개발일지에는 실패의 원인을 남길 것.** `ensurepip`, sudo, Git SSH, QoS/CLI 옵션 문제를 재현 가능한 형태로 기록했다.

> **2026-11-10 최종 상태:** YOLO 기반 어린이보호구역 표지판의 **로컬 모델 로딩과 rqt 시각 인식**은 확인되었고, 고신뢰 다중 프레임 인식 로직은 GitHub 코드에 구현되어 있다. **YOLO 신호부터 30km/h 감속과 기존 속도 복구까지 이어지는 완전한 통합 주행 성공은 아직 별도로 검증해야 한다.**
