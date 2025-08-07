#!/usr/bin/env python3
import cv2
import socket
import numpy as np
import time
from picamera2 import Picamera2
import struct
import threading

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

class PicamStream:
    """
    A thread-safe class to continuously read frames from a Picamera2 source.
    """
    def __init__(self, picam2):
        print("Initializing Picamera2 for threaded capture...")
        self.picam2 = picam2
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        self.new_frame_event = threading.Event()
        print("Picamera2 initialized successfully for threaded capture.")

    def start(self):
        # Start the thread to read frames from the video stream
        print("Starting camera capture thread...")
        t = threading.Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # Keep looping infinitely until the thread is stopped
        frame_count_internal = 0
        start_time_internal = time.time()
        while not self.stopped:
            try:
                frame = self.picam2.capture_array()
                frame_count_internal += 1
                if frame_count_internal % 30 == 0: # Print every 30 frames
                    elapsed_time_internal = time.time() - start_time_internal
                    if elapsed_time_internal > 0:
                        internal_fps = frame_count_internal / elapsed_time_internal
                        print(f"Internal Camera Capture FPS: {internal_fps:.2f}")
                    frame_count_internal = 0
                    start_time_internal = time.time()

                with self.lock:
                    self.frame = frame
                    self.new_frame_event.set()  # Signal that a new frame is available
            except Exception as e:
                print(f"Error in camera thread: {e}")
                self.stopped = True

    def read(self):
        # Wait until a new frame is available
        if not self.new_frame_event.wait(timeout=2):
            print("Warning: Timeout waiting for new frame.")
            return None
        with self.lock:
            if self.frame is None:
                self.new_frame_event.clear()
                return None
            frame = self.frame.copy()
        self.new_frame_event.clear()  # Reset the event for the next frame
        return frame

    def stop(self):
        # Indicate that the thread should be stopped
        self.stopped = True

def main():
    """
    Main function to capture, process, and send frames using optimized methods.
    """
    picam2 = Picamera2()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera_stream = None

    try:
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
        picam2.start()

        print("Waiting for camera to stabilize...")
        time.sleep(2)

        # TTL (Time-To-Live) 설정
        ttl = struct.pack('b', 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        print(f"Streaming video to multicast group {MCAST_GRP}:{MCAST_PORT}")

        camera_stream = PicamStream(picam2).start()

        # --- 최적화를 위해 왜곡 보정 맵 미리 계산 ---
        print("Calculating undistortion map...")
        h, w = 480, 640
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
        map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None, new_camera_matrix, (w, h), 5)
        x, y, w, h = roi
        print("Undistortion map calculated.")

        frame_count = 0 # Add frame count

        while True:
            frame = camera_stream.read()
            if frame is None:
                if camera_stream.stopped:
                    print("Camera stream stopped, exiting main loop.")
                    break
                print("Warning: Could not read new frame from stream. Retrying...")
                continue

            # 1. 180도 회전
            frame_rotated = cv2.rotate(frame, cv2.ROTATE_180)

            # 2. 미리 계산된 맵을 사용하여 왜곡 보정 (훨씬 빠름)
            undistorted_frame = cv2.remap(frame_rotated, map1, map2, cv2.INTER_LINEAR)

            # 3. ROI(관심 영역)에 맞춰 이미지 자르기
            undistorted_frame = undistorted_frame[y:y+h, x:x+w]

            # 4. RGB -> BGR 변환
            frame_bgr = cv2.cvtColor(undistorted_frame, cv2.COLOR_RGB2BGR)

            # 5. JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
            result, img_encoded = cv2.imencode('.jpg', frame_bgr, encode_param)

            # 6. 압축된 이미지 전송
            if result:
                try:
                    # Pack frame_count with image data
                    packed_data = struct.pack('I', frame_count) + img_encoded.tobytes()
                    sock.sendto(packed_data, (MCAST_GRP, MCAST_PORT))
                    frame_count += 1
                except socket.error as e:
                    print(f"Socket Error: {e}")
                    break
    
    except KeyboardInterrupt:
        print("Streaming stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Stopping camera and closing socket.")
        if camera_stream:
            camera_stream.stop()
        picam2.stop()
        sock.close()

if __name__ == '__main__':
    main()
