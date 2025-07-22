#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 우선 간단한 문자열 메시지 사용
import socket
import cv2
import numpy as np
from ultralytics import YOLO

class UdpRosYoloPublisher(Node):
    def __init__(self):
        super().__init__('udp_ros_yolo_publisher')
        # 탐지된 객체 정보를 게시할 퍼블리셔
        self.publisher_ = self.create_publisher(String, 'yolo_detections', 10)
        
        # UDP 서버 설정
        self.udp_host = '127.0.0.1'  # 송신자 IP 주소
        self.udp_port = 5005       # 수신 포트
        
        self.get_logger().info(f"Listening for UDP packets on {self.udp_host}:{self.udp_port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 소켓 버퍼 크기를 늘려 패킷 손실 가능성 감소
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.sock.bind((self.udp_host, self.udp_port))

        # YOLO 모델 로드
        # 사용 가능한 모델 경로로 수정해야 할 수 있습니다.
        self.model = YOLO('../yolo_model/yolov8m.pt')
        self.get_logger().info("YOLOv8 model loaded successfully.")
        # 탐지 신뢰도 임계값 설정 (0.0 ~ 1.0)
        self.yolo_conf_threshold = 0.3
        self.get_logger().info(f"YOLO confidence threshold set to: {self.yolo_conf_threshold}")

    def run(self):
        """UDP 패킷을 수신하고, YOLOv8 추론을 실행한 뒤 결과를 ROS2로 발행합니다."""
        while rclpy.ok():
            try:
                # 데이터 수신 대기 (블로킹)
                data, addr = self.sock.recvfrom(65536) # 버퍼 크기를 크게 설정
                
                # 바이트 데이터를 numpy 배열로 변환
                np_arr = np.frombuffer(data, np.uint8)
                # JPEG 이미지 디코딩
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    self.get_logger().warning("Failed to decode frame.")
                    continue

                # YOLO 모델로 추론 수행
                # results = self.model(frame, conf=self.yolo_conf_threshold)
                results = self.model(frame, conf=self.yolo_conf_threshold, classes=0)

                # 탐지 결과 프레임을 창에 표시
                annotated_frame = results[0].plot()
                cv2.imshow("YOLO Detection", annotated_frame)
                cv2.waitKey(1)

                # 탐지된 객체 정보 처리
                detection_info = ""
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]  # (x1, y1, x2, y2)
                        c = box.cls
                        class_name = self.model.names[int(c)]
                        confidence = float(box.conf)
                        
                        # 콘솔에 정보 출력
                        log_msg = f"Detected: {class_name} with confidence {confidence:.2f}"
                        self.get_logger().info(log_msg)
                        
                        # ROS 메시지로 보낼 문자열 구성
                        detection_info += f"{class_name} (conf: {confidence:.2f}), "

                # 탐지된 객체가 있을 경우 ROS2 메시지 발행
                if detection_info:
                    msg = String()
                    msg.data = detection_info.strip().rstrip(',')
                    self.publisher_.publish(msg)
                    # self.get_logger().info(f'Publishing to /yolo_detections: "{msg.data}"')

            except socket.timeout:
                self.get_logger().info("Socket timeout, waiting for new packets...")
                continue
            except Exception as e:
                self.get_logger().error(f"An error occurred: {e}")
                # 오류 발생 시 잠시 대기
                self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                self.sock.bind((self.udp_host, self.udp_port))


def main(args=None):
    rclpy.init(args=args)
    udp_ros_yolo_publisher = UdpRosYoloPublisher()
    
    try:
        udp_ros_yolo_publisher.run()
    except KeyboardInterrupt:
        udp_ros_yolo_publisher.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # 노드 및 소켓 정리
        cv2.destroyAllWindows()
        udp_ros_yolo_publisher.sock.close()
        udp_ros_yolo_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
