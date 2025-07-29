# UDP-pinky

## 개요

`UDP-pinky`는 Raspberry Pi 카메라 모듈을 사용하여 실시간 영상 스트림을 UDP 멀티캐스트로 전송하는 파이썬 기반 프로젝트입니다. PC에서는 수신된 영상 스트림을 통해 YOLO 객체 탐지 및 ArUco 마커 인식을 수행하고, 그 결과를 ROS2 토픽으로 발행합니다. 또한, 간단한 이미지 저장 기능도 포함하고 있습니다.

## 주요 기능

- **실시간 영상 스트리밍:** Raspberry Pi 카메라의 영상을 UDP 멀티캐스트를 통해 실시간으로 전송합니다.
- **YOLO 객체 탐지:** 수신된 영상에서 YOLOv8 모델을 사용하여 객체를 탐지하고, 결과를 ROS2 토픽으로 발행합니다.
- **ArUco 마커 인식:** 영상에서 ArUco 마커를 탐지하고, 자세 추정(pose estimation) 결과를 ROS2 토픽으로 발행합니다.
- **이미지 캡처 및 저장:** 스트리밍되는 영상을 보면서 원하는 순간에 이미지를 캡처하고 파일로 저장할 수 있습니다.

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
2. 로봇에 맞는 `dpXX_yolo_aruco.py`를 실행합니다.
   ```bash
   # 3번 로봇 데이터 처리
   python3 DP_03/dp03_yolo_aruco.py

   # 9번 로봇 데이터 처리
   python3 DP_09/dp09_yolo_aruco.py
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