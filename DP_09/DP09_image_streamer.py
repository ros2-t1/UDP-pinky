#!/usr/bin/env python3
import cv2
import socket
import numpy as np
import time
from picamera2 import Picamera2
import struct
from threading import Thread, Event
from queue import Queue, Full

# --- Frame Capture Thread ---
class FrameCaptureThread(Thread):
    def __init__(self, picam2, queue):
        super().__init__()
        self.picam2 = picam2
        self.queue = queue
        self.stopped = Event()

    def run(self):
        """Continuously captures frames and puts them into the queue."""
        while not self.stopped.is_set():
            try:
                # 프레임 캡처 및 180도 회전
                frame = self.picam2.capture_array()
                frame_rotated = cv2.rotate(frame, cv2.ROTATE_180)
                # 큐가 가득 찼으면, 가장 오래된 프레임을 버리고 새 프레임을 넣음
                if self.queue.full():
                    self.queue.get_nowait()
                self.queue.put_nowait(frame_rotated)
            except Full:
                # 이 경우는 거의 발생하지 않지만, 만약을 위해 처리
                pass
            except Exception as e:
                print(f"Capture Error: {e}")
                self.stop()

    def stop(self):
        """Signals the thread to stop."""
        self.stopped.set()

# --- Main Application ---
def main():
    # 카메라 매개변수 (DP09_cam_param.txt 파일 내용 기반)
    camera_matrix = np.array([[557.97222, 0, 336.50260],
                            [0, 559.41434, 233.20376],
                            [0, 0, 1]])

    distortion_coefficients = np.array([1.372049e-01, -6.539872e-01, -7.222502e-04, 2.262971e-03, 7.711442e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00])

    # 멀티캐스트 그룹 주소 및 포트
    MCAST_GRP = '224.1.1.1'
    MCAST_PORT = 5009

    # 소켓 생성 (UDP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ttl = struct.pack('b', 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    # Picamera2 객체 생성 및 설정
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": 'RGB888', "size": (640, 480)},
        controls={"FrameRate": 30, "AeEnable": True, "AwbEnable": True}
    )
    picam2.configure(config)
    picam2.start()

    print("Waiting for camera to stabilize...")
    time.sleep(2)

    # 왜곡 보정 맵 계산
    h, w = 480, 640
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w,h), 1, (w,h))
    x_roi, y_roi, w_roi, h_roi = roi
    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None, new_camera_matrix, (w,h), cv2.CV_32FC1)

    # 프레임 큐와 캡처 스레드 생성 및 시작
    frame_queue = Queue(maxsize=1)
    capture_thread = FrameCaptureThread(picam2, frame_queue)
    capture_thread.start()

    print(f"Streaming video to multicast group {MCAST_GRP}:{MCAST_PORT}")

    try:
        while True:
            # 큐에서 프레임 가져오기
            frame = frame_queue.get()

            # 왜곡 보정 (remap 사용)
            undistorted_frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            # ROI(관심 영역)에 맞춰 이미지 자르기
            undistorted_frame = undistorted_frame[y_roi:y_roi+h_roi, x_roi:x_roi+w_roi]

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
        # 리소스 정리
        print("Stopping threads and cleaning up...")
        capture_thread.stop()
        capture_thread.join() # 스레드가 완전히 종료될 때까지 대기
        picam2.stop()
        sock.close()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()