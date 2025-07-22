#!/usr/bin/env python3
import cv2
import socket
import numpy as np

# UDP 서버 설정
UDP_IP = "127.0.0.1"  # 수신자 IP 주소 (localhost)
UDP_PORT = 5005      # 수신자 포트

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 비디오 캡처 객체 생성 (0은 기본 웹캠)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

print(f"Streaming video to {UDP_IP}:{UDP_PORT}")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break

    # 프레임 크기 조절 (UDP 패킷 크기 제한 문제 해결)
    frame = cv2.resize(frame, (640, 480))

    # 프레임을 JPEG로 압축
    # 품질을 0-100 사이로 조절할 수 있습니다 (높을수록 화질이 좋지만 데이터 양이 커짐)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
    result, img_encoded = cv2.imencode('.jpg', frame, encode_param)

    # 압축된 이미지를 바이트로 변환
    data = np.array(img_encoded).tobytes()

    # UDP를 통해 데이터 전송
    # UDP 패킷 크기 제한(일반적으로 65507 바이트) 때문에 큰 프레임은 문제가 될 수 있습니다.
    # 여기서는 간단하게 전송하지만, 실제 환경에서는 분할 및 재조립 로직이 필요할 수 있습니다.
    try:
        sock.sendto(data, (UDP_IP, UDP_PORT))
    except socket.error as e:
        print(f"Socket Error: {e}")
        # 여기서 연결이 끊겼다고 가정하고 루프를 중단할 수 있습니다.
        break

    # 화면에 영상 표시
    # cv2.imshow('Camera Stream', frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
sock.close()
cv2.destroyAllWindows()
