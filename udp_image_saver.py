import cv2
import os
import time
import socket
import numpy as np
import struct

# 이미지 저장 디렉토리
IMG_DIR = os.path.join(os.path.expanduser('~'), 'Documents', 'img')

# 디렉토리가 없으면 생성
if not os.path.exists(IMG_DIR):
    os.makedirs(IMG_DIR)

# UDP 소켓 설정
MCAST_GRP = "224.1.1.1"
MCAST_PORT = 5007
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', MCAST_PORT))
group = socket.inet_aton(MCAST_GRP)
mreq = struct.pack('4sL', group, socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# 수신 버퍼 크기 설정 (65535가 UDP의 최대 크기)
buffer_size = 65536

print(f"UDP 서버가 {MCAST_GRP}:{MCAST_PORT}에서 실행 중입니다.")
print("PiCam 스트림을 기다리는 중... 'space' 키를 눌러 이미지를 저장하세요.")
print("'q' 키를 누르면 종료됩니다.")

try:
    while True:
        # 데이터 수신
        data, addr = sock.recvfrom(buffer_size)
        
        # 수신된 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(data, np.uint8)
        
        # numpy 배열을 이미지로 디코딩
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 프레임이 유효한 경우에만 화면에 표시
        if frame is not None:
            cv2.imshow('UDP PiCam Stream', frame)
        else:
            print("수신된 데이터로 이미지를 디코딩할 수 없습니다.")
            continue

        # 키 입력 대기
        key = cv2.waitKey(1) & 0xFF

        # 스페이스바를 누르면 이미지 저장
        if key == ord(' '):
            if frame is not None:
                # 파일명에 타임스탬프 추가하여 중복 방지
                filename = f"capture_{int(time.time())}.png"
                save_path = os.path.join(IMG_DIR, filename)
                cv2.imwrite(save_path, frame)
                print(f"이미지 저장 완료: {save_path}")
            else:
                print("저장할 유효한 프레임이 없습니다.")

        # 'q'를 누르면 종료
        elif key == ord('q'):
            break
finally:
    # 자원 해제
    print("프로그램을 종료합니다.")
    sock.close()
    cv2.destroyAllWindows()
