#!/usr/bin/env python3
import cv2
import socket
import numpy as np
import time
from picamera2 import Picamera2

# UDP 서버 설정
UDP_IP = "127.0.0.1"  # 수신자 IP 주소 (YOLO 노드가 실행되는 컴퓨터)
UDP_PORT = 5005      # 수신자 포트

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Picamera2 객체 생성
picam2 = Picamera2()

# --- 카메라 설정 ---
# 색상 문제를 해결하기 위해 자동 노출과 자동 화이트 밸런스를 다시 활성화합니다.
config = picam2.create_preview_configuration(
    main={"format": 'RGB888', "size": (640, 480)},
    controls={
        "FrameRate": 30,    # 프레임 속도는 고정
        "AeEnable": True,   # 자동 노출 다시 켜기
        "AwbEnable": True,  # 자동 화이트 밸런스 다시 켜기
    }
)
picam2.configure(config)

# 카메라 시작
picam2.start()

# 카메라 센서가 안정화될 때까지 잠시 대기
print("Waiting for camera to stabilize...")
time.sleep(2)
print(f"Streaming video to {UDP_IP}:{UDP_PORT}")

try:
    while True:
        # 프레임 캡처
        frame = picam2.capture_array()

        # 카메라가 물리적으로 거꾸로 장착된 경우 180도 회전합니다.
        # 필요 없으면 이 줄을 주석 처리(#)하거나 삭제하세요.
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Picamera2는 RGB 형식이므로 OpenCV에서 사용하기 위해 BGR로 변환
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 프레임을 JPEG로 압축 (품질 70)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        result, img_encoded = cv2.imencode('.jpg', frame_bgr, encode_param)

        # 압축된 이미지를 바이트로 변환하여 UDP로 전송
        if result:
            try:
                sock.sendto(img_encoded.tobytes(), (UDP_IP, UDP_PORT))
            except socket.error as e:
                print(f"Socket Error: {e}")
                break
        
except KeyboardInterrupt:
    print("Streaming stopped by user.")
finally:
    # 리소스 해제
    print("Stopping camera and closing socket.")
    picam2.stop()
    sock.close()