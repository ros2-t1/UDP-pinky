#!/usr/bin/env python3
import cv2
import socket
import numpy as np
import time
from picamera2 import Picamera2
import struct

# 카메라 매개변수 (DP09_cam_param.txt 파일 내용 기반)
camera_matrix = np.array([
    [580.18084, 0, 337.87215],
    [0, 576.57957, 222.68874],
    [0, 0, 1]
])
distortion_coefficients = np.array([5.625842e-02, 1.511098e-02, -5.239435e-03, 3.544454e-03, -7.173680e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00])

# 멀티캐스트 그룹 주소 및 포트
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5009

# 소켓 생성 (UDP)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# TTL (Time-To-Live) 설정: 멀티캐스트 패킷이 라우터를 몇 번 거칠 수 있는지 결정
ttl = struct.pack('b', 1) # 1은 로컬 네트워크 내에서만 전송
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

# Picamera2 객체 생성
picam2 = Picamera2()

# --- 카메라 설정 ---
config = picam2.create_preview_configuration(
    main={"format": 'RGB888', "size": (640, 480)},
    controls={
        "FrameRate": 30,
        "AeEnable": True,
        "AwbEnable": True,
    }
)
picam2.configure(config)

# 카메라 시작
picam2.start()

# 카메라 안정화 대기
print("Waiting for camera to stabilize...")
time.sleep(2)
print(f"Streaming video to multicast group {MCAST_GRP}:{MCAST_PORT}")

try:
    # 최적의 새 카메라 매트릭스를 한 번만 계산
    h, w = 480, 640
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w,h), 1, (w,h))
    x, y, w, h = roi

    while True:
        # 프레임 캡처
        frame = picam2.capture_array()

        # 180도 회전 (필요 시 사용)
        frame_rotated = cv2.rotate(frame, cv2.ROTATE_180)

        # 왜곡 보정
        undistorted_frame = cv2.undistort(frame_rotated, camera_matrix, distortion_coefficients, None, new_camera_matrix)

        # ROI(관심 영역)에 맞춰 이미지 자르기
        undistorted_frame = undistorted_frame[y:y+h, x:x+w]

        # RGB -> BGR 변환
        frame_bgr = cv2.cvtColor(undistorted_frame, cv2.COLOR_RGB2BGR)

        # JPEG 압축
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        result, img_encoded = cv2.imencode('.jpg', frame_bgr, encode_param)

        # 압축된 이미지 전송
        if result:
            try:
                sock.sendto(img_encoded.tobytes(), (MCAST_GRP, MCAST_PORT))
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
