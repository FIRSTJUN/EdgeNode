# MORAI OpenCV 차선 인지 시험

`control` 브랜치에서 기존 컨테이너로 실행하는 카메라 인지 시험이다.
원본은 [perception 브랜치의 OpenCV 패키지](https://github.com/FIRSTJUN/EdgeNode/tree/8146beadab2f24beed92ddaae21d35a10184506f/%EC%9D%B8%EC%A7%80%EA%B0%9C%EB%B0%9C/lane_opencv_pkg)다.

- 기준 커밋: `8146beadab2f24beed92ddaae21d35a10184506f`
- 가져온 위치: `ros2_ws/src/lane_opencv_pkg`
- `lane_detector.py` 알고리즘은 원본 그대로다.
- `package.xml`에 기존 환경의 OpenCV/NumPy 및 빌드 도구 의존성 선언만 보완했다.
- Dockerfile, compose, 기존 인지·계획·제어 소스는 변경하지 않는다.

## 실행

MORAI에서 ROS 2 카메라 송출을 켠 뒤, 현재 컨테이너의 프로젝트 루트에서 실행한다.

```bash
bash tools/opencv/run_lane_opencv.sh
```

새 패키지만 빌드하고 `lane_opencv_test` 노드와 차선 검출/마스크 미리보기를 실행한다.
화면에서 **Q 또는 Esc**, 터미널에서 **Ctrl+C**로 종료한다.
차량 제어 노드는 실행하지 않는다.

카메라 기본 입력은 `/camera/image/compressed`, 메시지 형식은
`sensor_msgs/msg/CompressedImage`다. 다른 토픽을 사용하면 다음처럼 지정한다.

```bash
bash tools/opencv/run_lane_opencv.sh --ros-args -p camera_topic:=/image_jpeg/compressed
```

GUI 없이 실행하려면:

```bash
bash tools/opencv/run_lane_opencv.sh --no-view
```

이미 빌드한 패키지는 직접 실행할 수도 있다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
ros2 run lane_opencv_pkg lane_detector --ros-args -r __node:=lane_opencv_test
```

## 수신 및 결과 확인

다른 터미널에서 ROS 환경을 불러온 뒤 확인한다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 topic hz /camera/image/compressed
ros2 topic echo /perception/lane_valid
ros2 topic echo /perception/lane_error
```

| 토픽 | 내용 |
| --- | --- |
| `/perception/lane_error` | 영상 중심 대비 차로 중심의 정규화 오차 |
| `/perception/lane_confidence` | 검출 픽셀 수 기반 신뢰도 |
| `/perception/lane_valid` | 해당 프레임의 검출 결과 유무 |
| `/perception/debug/lane_image` | 검출 결과 영상 (`sensor_msgs/msg/Image`, `bgr8`) |
| `/perception/debug/lane_mask` | 차선 후보 마스크 (`sensor_msgs/msg/Image`, `mono8`) |

영상이 들어오면 노드가 100프레임마다 `Camera OK`를 출력한다.
미리보기가 대기 상태라면 카메라 토픽 이름·메시지 형식·MORAI 송출 여부를 먼저 확인한다.
같은 카메라 토픽으로 rosbag과 MORAI를 동시에 송출하지 않는다.

원본 알고리즘은 HLS/HSV 색상 마스크, sliding window, 직선 피팅을 사용한다.
`lane_valid=True`는 결과가 있다는 뜻이며 높은 정확도를 보장하는 값은 아니다.

## 실행 확인 (2026-10-03)

- 현재 컨테이너에서 새 패키지 빌드 및 미리보기 실행을 확인했다.
- 합성 입력 7개 검사와 저장된 MORAI 원본 프레임 6장 처리를 완료했다.
- 실시간 8초 관측에서 640×480 카메라 영상 239개와 인지 결과 78개를 수신했다.
  수신한 결과 78개는 모두 `lane_valid=True`였고 마지막 값은
  `error=-0.0084`, `confidence=0.5`였다. 저장한 영상은 `LEFT` 모드다.
- 초기 장면에서는 추정 차선 폭이 허용 범위를 넘어 `LANE REJECT`가 발생했다.
  실행과 수신을 확인한 결과이며, 모든 장면의 차선 검출 정확도를 보장하지 않는다.

실시간 캡처는 `debug_frames/opencv_live/`, 오프라인 검증 결과는
`debug_frames/opencv_trial/`에 저장했다. 두 폴더는 Git 추적 대상에서 제외된다.
