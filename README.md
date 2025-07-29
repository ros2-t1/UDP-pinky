# UDP-pinky

## 개요

`UDP-pinky`는 Raspberry Pi 카메라 모듈을 사용하여 실시간 영상 스트림을 UDP 멀티캐스트로 전송하고, 다른 컴퓨터에서 해당 스트림을 수신하여 보거나 저장할 수 있는 파이썬 기반 프로젝트입니다.

## 주요 기능

- **실시간 영상 스트리밍:** Raspberry Pi 카메라의 영상을 네트워크를 통해 실시간으로 전송합니다.
- **UDP 멀티캐스트:** 한 번의 전송으로 여러 클라이언트가 동시에 영상 스트림을 수신할 수 있습니다.
- **이미지 캡처 및 저장:** 스트리밍되는 영상을 보면서 원하는 순간에 스페이스바를 눌러 이미지를 캡처하고 파일로 저장할 수 있습니다.

## 구성 요소

- **`udp_picam_multi_streamer.py`:** Raspberry Pi에서 실행되는 스크립트로, 카메라 영상을 받아 UDP 멀티캐스트로 스트리밍합니다.
- **`udp_image_saver.py`:** 스트림을 수신할 컴퓨터에서 실행되는 스크립트로, 영상을 화면에 표시하고 이미지 저장 기능을 제공합니다.
- **`yolo_model/`:** YOLOv8 모델 파일(`yolov8s.pt`)이 저장되어 있습니다.
- **`DP_03/`:** 3번 Pinky 로봇에서 실행되는 코드입니다.
    - `dp03_yolo_aruco.py`: UDP 멀티캐스트(포트 5003)로 수신된 영상에서 YOLO 객체 탐지와 ArUco 마커 인식을 수행하고, 그 결과를 ROS2 토픽으로 발행합니다.
    - `DP03_image_streamer.py`: 3번 Pinky 로봇의 카메라 영상을 UDP 멀티캐스트(포트 5003)로 스트리밍합니다.
- **`DP_09/`:** 9번 Pinky 로봇에서 실행되는 코드입니다.
    - `dp09_yolo_aruco.py`: UDP 멀티캐스트(포트 5009)로 수신된 영상에서 YOLO 객체 탐지와 ArUco 마커 인식을 수행하고, 그 결과를 ROS2 토픽으로 발행합니다.
    - `DP09_image_streamer.py`: 9번 Pinky 로봇의 카메라 영상을 UDP 멀티캐스트(포트 5009)로 스트리밍합니다.

## 사용 방법

### 1. 기본 영상 스트리밍 및 저장

#### Raspberry Pi 설정
1. Raspberry Pi에 카메라 모듈을 연결하고 활성화합니다.
2. `udp_picam_multi_streamer.py` 파일을 Raspberry Pi에 복사합니다.
3. 다음 명령어를 실행하여 영상 스트리밍을 시작합니다.
   ```bash
   python3 udp_picam_multi_streamer.py
   ```

#### 클라이언트 (수신 컴퓨터) 설정
1. `udp_image_saver.py` 파일을 스트림을 수신할 컴퓨터에 복사합니다.
2. 다음 명령어를 실행하여 영상 스트림을 수신하고 화면에 표시합니다.
   ```bash
   python3 udp_image_saver.py
   ```
3. 영상 스트림 창이 활성화된 상태에서 **스페이스바**를 누르면 현재 프레임이 `~/Documents/img/` 폴더에 `capture_{타임스탬프}.png` 형식으로 저장됩니다.
4. **'q'** 키를 누르면 프로그램을 종료합니다.

### 2. YOLO 및 ArUco 마커 동시 인식 (ROS2 환경)

#### Pinky 로봇 (Raspberry Pi) 설정
1. 각 로봇에 맞는 폴더(`DP_03` 또는 `DP_09`)의 `DPXX_image_streamer.py`를 실행하여 해당 포트로 영상을 스트리밍합니다.
   ```bash
   # 3번 로봇의 경우
   python3 DP_03/DP03_image_streamer.py

   # 9번 로봇의 경우
   python3 DP_09/DP09_image_streamer.py
   ```

#### ROS2 워크스테이션 설정
1. ROS2 환경을 설정합니다.
2. 각 로봇에 맞는 `dpXX_yolo_aruco.py`를 실행합니다.
   ```bash
   # 3번 로봇의 데이터를 처리할 경우
   ros2 run <your_package_name> dp03_yolo_aruco.py

   # 9번 로봇의 데이터를 처리할 경우
   ros2 run <your_package_name> dp09_yolo_aruco.py
   ```
3. 스크립트를 실행하면 YOLO와 ArUco 마커 탐지 결과가 ROS2 토픽 (`/yolo_detections`, `/aruco_rvec_topic`, `/aruco_tvec_topic`)으로 발행됩니다.

## 요구 사항

- Python 3
- OpenCV
- NumPy
- Picamera2 (Raspberry Pi)
- ROS2 (Jazzy Jalisco)
- ultralytics (YOLO)
- vision_msgs (ROS2)

## 향후 개발 계획

- **데이터 융합:** 각 로봇에서 탐지된 YOLO 및 ArUco 데이터를 중앙에서 융합하여 보다 정확한 객체 위치 및 상태를 추정하는 기능 개발
- **다중 로봇 제어:** 탐지된 정보를 기반으로 다중 로봇을 자율적으로 제어하는 알고리즘 구현
- **시각화 개선:** Rviz2 또는 웹 기반 대시보드를 통해 다중 로봇의 탐지 정보와 상태를 통합적으로 시각화하는 기능 개발
- **성능 최적화:** TensorRT와 같은 추론 가속 기술을 적용하여 YOLO 모델의 탐지 속도 향상