#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
import socket
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import queue

class UdpPicamRosYoloPublisher(Node):
    def __init__(self):
        super().__init__('udp_picam_ros_yolo_publisher')
        self.publisher_ = self.create_publisher(Detection2DArray, 'yolo_detections', 10)
        
        self.udp_host = '127.0.0.1' # YOLO 노드가 실행되는 PC IP
        self.udp_port = 5005
        
        self.get_logger().info(f"Listening for UDP packets on {self.udp_host}:{self.udp_port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.sock.bind((self.udp_host, self.udp_port))

        self.model = YOLO('../yolo_model/yolov8m.pt')
        self.get_logger().info("YOLOv8 model loaded successfully.")
        self.yolo_conf_threshold = 0.7
        self.get_logger().info(f"YOLO confidence threshold set to: {self.yolo_conf_threshold}")

        # 프레임을 저장할 스레드 안전 큐
        self.frame_queue = queue.Queue(maxsize=1)
        # 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self._receive_frames, daemon=True)
        self.receive_thread.start()

    def _receive_frames(self):
        """UDP 패킷을 수신하여 큐에 넣는 역할만 하는 스레드"""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(65536)
                if not self.frame_queue.full():
                    self.frame_queue.put(data)
            except Exception as e:
                self.get_logger().error(f"Receive thread error: {e}")

    def run(self):
        """큐에서 프레임을 가져와 YOLO 처리 및 ROS 발행을 수행"""
        while rclpy.ok():
            try:
                # 큐에서 가장 최신 프레임을 가져옴 (없으면 잠시 대기)
                data = self.frame_queue.get(timeout=1.0)
                self.frame_queue.queue.clear() # 큐를 비워 항상 최신 프레임만 처리

                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    self.get_logger().warning("Failed to decode frame.")
                    continue
                
                frame = cv2.flip(frame, 1)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.model.track(frame_rgb, persist=True, conf=self.yolo_conf_threshold, classes=0, tracker="botsort.yaml")

                # results[0].plot()는 BGR 형식의 numpy 배열을 반환하므로, 별도의 색상 변환이 필요 없습니다.
                annotated_frame_bgr = results[0].plot()
                cv2.imshow("YOLO Detection", annotated_frame_bgr)
                cv2.waitKey(1)

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
                        center_x = (x1 + x2) / 2.0
                        center_y = (y1 + y2) / 2.0
                        w = x2 - x1
                        h = y2 - y1

                        bbox = BoundingBox2D()
                        bbox.center.position.x = float(center_x)
                        bbox.center.position.y = float(center_y)
                        bbox.size_x = float(w)
                        bbox.size_y = float(h)
                        detection.bbox = bbox
                        detections_msg.detections.append(detection)

                if detections_msg.detections:
                    self.publisher_.publish(detections_msg)

            except queue.Empty:
                # 큐가 비어있으면 (새 프레임이 없으면) 아무것도 하지 않음
                continue
            except Exception as e:
                self.get_logger().error(f"An error occurred in run loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UdpPicamRosYoloPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        cv2.destroyAllWindows()
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
