# UDP-pinky

## 개요

`UDP-pinky`는 Raspberry Pi 카메라 모듈을 사용하여 실시간 영상 스트림을 UDP 멀티캐스트로 전송하고, 다른 컴퓨터에서 해당 스트림을 수신하여 보거나 저장할 수 있는 파이썬 기반 프로젝트입니다.

## 주요 기능

- **실시간 영상 스트리밍:** Raspberry Pi 카메라의 영상을 네트워크를 통해 실시간으로 전송합니다.
- **UDP 멀티캐스트:** 한 번의 전송으로 여러 클라이언트가 동시에 영상 스트림을 수신할 수 있습니다.
- **이미지 캡처 및 저장:** 스트리밍되는 영상을 보면서 원하는 순간에 스페이스바를 눌러 이미지를 캡처하고 파일로 저장할 수 있습니다.

## 구성 요소

- **`udp_picam_multi_streamer.py` & `udp_image_saver.py`**: 기본적인 UDP 영상 송수신 및 저장 기능을 위한 스크립트입니다.
    - `udp_picam_multi_streamer.py`: Raspberry Pi에서 실행하여 카메라 영상을 UDP 멀티캐스트로 스트리밍합니다.
    - `udp_image_saver.py`: PC에서 실행하여 스트림을 수신하고, 스페이스바를 눌러 이미지를 저장합니다.
- **`yolo_model/`**: YOLOv8 모델 파일(`yolov8s.pt`)이 저장되어 있습니다.
- **`DP_03/`**: 3번 Pinky 로봇 및 연동되는 PC를 위한 코드입니다.
    - `DP03_image_streamer.py`: 3번 Pinky 로봇(Raspberry Pi)에서 실행하여 카메라 영상을 UDP 멀티캐스트(포트 5003)로 스트리밍합니다.
    - `dp03_yolo_aruco.py`: PC(ROS2 워크스테이션)에서 실행하여 3번 로봇의 영상 스트림을 받아 YOLO 객체 탐지 및 ArUco 마커 인식을 수행하고, 결과를 ROS2 토픽으로 발행합니다.
- **`DP_09/`**: 9번 Pinky 로봇 및 연동되는 PC를 위한 코드입니다.
    - `DP09_image_streamer.py`: 9번 Pinky 로봇(Raspberry Pi)에서 실행하여 카메라 영상을 UDP 멀티캐스트(포트 5009)로 스트리밍합니다.
    - `dp09_yolo_aruco.py`: PC(ROS2 워크스테이션)에서 실행하여 9번 로봇의 영상 스트림을 받아 YOLO 객체 탐지 및 ArUco 마커 인식을 수행하고, 결과를 ROS2 토픽으로 발행합니다.

## 사용 방법

### 1. 기본 영상 스트리밍 및 저장

#### Raspberry Pi (송신 측)
1. `udp_picam_multi_streamer.py`를 실행합니다.
   ```bash
   python3 udp_picam_multi_streamer.py
   ```

#### PC (수신 측)
1. `udp_image_saver.py`를 실행합니다.
   ```bash
   python3 udp_image_saver.py
   ```
2. 영상 창에서 **스페이스바**를 누르면 이미지가 저장되고, **'q'** 키로 종료합니다.

### 2. YOLO 및 ArUco 마커 동시 인식 (ROS2 환경)

#### Pinky 로봇 (Raspberry Pi, 송신 측)
1. 각 로봇에 맞는 `DPXX_image_streamer.py`를 실행하여 영상을 스트리밍합니다.
   ```bash
   # 3번 로봇
   python3 DP_03/DP03_image_streamer.py

   # 9번 로봇
   python3 DP_09/DP09_image_streamer.py
   ```

#### PC (ROS2 워크스테이션, 수신 및 처리 측)
1. ROS2 환경을 설정합니다.
2. 로봇에 맞는 `dpXX_yolo_aruco.py`를 ROS2 노드로 실행합니다.
   ```bash
   # 3번 로봇 데이터 처리
   ros2 run <your_package_name> dp03_yolo_aruco.py

   # 9번 로봇 데이터 처리
   ros2 run <your_package_name> dp09_yolo_aruco.py
   ```
3. 실행 후, 각 로봇의 YOLO와 ArUco 마커 탐지 결과가 아래의 네임스페이스를 가진 ROS2 토픽으로 발행되는 것을 확인할 수 있습니다:
   - **3번 로봇:** `/DP03/yolo_detections`, `/DP03/aruco_rvec_topic`, `/DP03/aruco_tvec_topic`
   - **9번 로봇:** `/DP09/yolo_detections`, `/DP09/aruco_rvec_topic`, `/DP09/aruco_tvec_topic`

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