#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header, Float32MultiArray
import socket
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import queue
import time
import struct

class YoloArucoUdpRosPublisher(Node):
    """
    UDP 멀티캐스트 스트림에서 이미지 프레임을 수신하고,
    YOLO 객체 탐지 및 ArUco 마커 탐지를 수행한 후,
    결과를 ROS2 토픽으로 발행하는 노드 클래스.
    """
    def __init__(self, conf_threshold):
        super().__init__('yolo_aruco_udp_ros_publisher')

        # --- ROS2 퍼블리셔 초기화 ---
        self.yolo_publisher_ = self.create_publisher(Detection2DArray, '/DP09/yolo_detections', 10)
        self.rvec_publisher_ = self.create_publisher(Float32MultiArray, '/DP09/aruco_rvec_topic', 10)
        self.tvec_publisher_ = self.create_publisher(Float32MultiArray, '/DP09/aruco_tvec_topic', 10)
        self.get_logger().info("ROS2 퍼블리셔가 초기화되었습니다.")

        # --- UDP 수신 설정 ---
        self.udp_host = '224.1.1.1'
        self.udp_port = 5009 # yolo_aruco_udp_stream.py에서 사용하던 포트
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.udp_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.udp_host), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f"UDP 멀티캐스트 수신 대기 중: {self.udp_host}:{self.udp_port}")

        # --- 모델 초기화 ---
        self.model = YOLO('../yolo_model/yolov8s.pt')
        self.get_logger().info("YOLOv8 모델이 성공적으로 로드되었습니다.")
        self.yolo_conf_threshold = conf_threshold
        self.get_logger().info(f"YOLO 신뢰도 임계값: {self.yolo_conf_threshold}")

        # --- ArUco 초기화 ---
        self.get_logger().info("ArUco 탐지기 초기화 중...")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_length = 0.1 # 10cm
        self.get_logger().info("ArUco 탐지기가 초기화되었습니다.")

        # --- 카메라 파라미터 (ArUco 자세 추정용) ---
        self.camera_matrix = np.array([[562.75803, 0, 331.28656],
                                       [0, 564.05573, 242.66754],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([1.538044e-01, -7.581010e-01, -2.535588e-04, -7.386026e-04, 9.812706e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00], dtype=np.float32)

        # --- 스레드 및 큐 설정 ---
        self.frame_queue = queue.Queue(maxsize=2)
        self.receive_thread = threading.Thread(target=self._receive_frames, daemon=True)
        self.receive_thread.start()

        self.window_name = "YOLO and ArUco Detection (UDP/ROS2)"
        self.processing_timer = self.create_timer(0.01, self.process_and_publish) # 10ms 마다 처리

    def _receive_frames(self):
        """UDP 패킷을 수신하여 큐에 넣는 스레드 함수"""
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(65536)
                if not self.frame_queue.full():
                    self.frame_queue.put(data)
            except Exception as e:
                self.get_logger().error(f"프레임 수신 중 오류 발생: {e}")

    def process_and_publish(self):
        """큐에서 프레임을 가져와 처리하고 ROS2 토픽을 발행하는 콜백 함수"""
        try:
            data = self.frame_queue.get_nowait()
            # 항상 최신 프레임을 처리하기 위해 큐를 비움
            while not self.frame_queue.empty():
                data = self.frame_queue.get_nowait()

            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warning("프레임 디코딩에 실패했습니다.")
                return

            # --- ArUco 탐지 및 발행 ---
            self.detect_and_publish_aruco(frame)

            # --- YOLO 탐지 및 발행 ---
            annotated_frame = self.detect_and_publish_yolo(frame)
            
            # --- ArUco 시각화 (YOLO 결과 위에) ---
            # (YOLO가 그린 프레임 위에 ArUco 정보를 다시 그립니다)
            aruco_corners, aruco_ids, _ = self.aruco_detector.detectMarkers(frame)
            if aruco_ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                cv2.aruco.drawDetectedMarkers(annotated_frame, aruco_corners, aruco_ids)
                for i in range(len(aruco_ids)):
                    cv2.drawFrameAxes(annotated_frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)


            # --- 화면 표시 ---
            cv2.imshow(self.window_name, annotated_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("'q' 키가 눌려 노드를 종료합니다.")
                self.destroy_node()
                rclpy.shutdown()


        except queue.Empty:
            return # 처리할 프레임이 없으면 반환
        except Exception as e:
            self.get_logger().error(f"처리 및 발행 중 오류 발생: {e}")

    def detect_and_publish_aruco(self, frame):
        """ArUco 마커를 탐지하고 rvec/tvec 정보를 발행합니다."""
        aruco_corners, aruco_ids, _ = self.aruco_detector.detectMarkers(frame)
        if aruco_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i in range(len(aruco_ids)):
                rvec_msg = Float32MultiArray(data=rvecs[i].flatten().tolist())
                tvec_msg = Float32MultiArray(data=tvecs[i].flatten().tolist())
                self.rvec_publisher_.publish(rvec_msg)
                self.tvec_publisher_.publish(tvec_msg)

    def detect_and_publish_yolo(self, frame):
        """YOLO로 객체를 탐지하고 Detection2DArray 메시지를 발행합니다."""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model.track(frame_rgb, persist=True, conf=self.yolo_conf_threshold, classes=0, tracker="botsort.yaml", verbose=False)
        
        annotated_frame = results[0].plot() # BGR 이미지 반환

        detections_msg = Detection2DArray()
        detections_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera")

        if results[0].boxes.id is not None:
            for box in results[0].boxes:
                detection = Detection2D()
                detection.id = str(int(box.id[0]))
                class_id = int(box.cls)
                confidence = float(box.conf)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                x1, y1, x2, y2 = box.xyxy[0]
                bbox = BoundingBox2D()
                bbox.center.position.x = float((x1 + x2) / 2.0)
                bbox.center.position.y = float((y1 + y2) / 2.0)
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                detection.bbox = bbox
                detections_msg.detections.append(detection)

        if detections_msg.detections:
            self.yolo_publisher_.publish(detections_msg)
            
        return annotated_frame

    def destroy_node(self):
        self.get_logger().info("노드를 종료합니다.")
        self.sock.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def get_conf_threshold_input():
    """사용자로부터 YOLO 신뢰도 임계값을 입력받습니다."""
    while True:
        try:
            conf = float(input("YOLO 신뢰도 임계값을 입력하세요 (0.0 ~ 1.0): "))
            if 0.0 <= conf <= 1.0:
                return conf
            else:
                print("잘못된 값입니다. 0.0과 1.0 사이의 값을 입력해주세요.")
        except ValueError:
            print("잘못된 입력입니다. 숫자를 입력해주세요.")

def main(args=None):
    rclpy.init(args=args)
    conf_threshold = get_conf_threshold_input()
    node = YoloArucoUdpRosPublisher(conf_threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
