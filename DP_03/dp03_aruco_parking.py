"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
from collections import Counter

class ArucoParkingNode(Node):
    def __init__(self):
        super().__init__('aruco_parking')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rvec_sub = self.create_subscription(
            Float32MultiArray,
            '/DP03/aruco_rvec_topic',
            self.rvec_callback,
            1  # 큐 사이즈 1
        )
        self.tvec_sub = self.create_subscription(
            Float32MultiArray,
            '/DP03/aruco_tvec_topic',
            self.tvec_callback,
            1  # 큐 사이즈 1
        )
        
        # State variables
        self.rvec_data = None
        self.tvec_data = None
        self.rvec_timestamp = None
        self.tvec_timestamp = None
        self.marker_detected = False
        
        # 속도 설정
        self.linear_speed = 0.01  # m/s
        self.angular_speed = 0.02  # rad/s
        
        # 시간 설정 변수
        self.search_rotation_time = 0.2  # 마커 탐색 시 회전 시간
        self.movement_rotation_time = 0.4  # 이동 시 회전 시간
        self.movement_forward_time = 0.2  # 이동 시 직진 시간
        self.correction_rotation_time = 0.4  # 보정 회전 시간
        self.pitch_rotation_time = 0.2  # Pitch 조정 시 회전 시간
        self.backward_time = 1.0  # 후진 시간
        
        # 상태 관리 - POSITION_ADJUSTMENT와 ORIENTATION_ADJUSTMENT 분리
        self.state = "SEARCHING"  # SEARCHING, SAMPLING, POSITION_PROCESSING, POSITION_ADJUSTMENT, ORIENTATION_PROCESSING, ORIENTATION_ADJUSTMENT, EXECUTING, PARKING_COMPLETE
        self.last_action_time = 0
        
        # 샘플링 변수
        self.x_samples = []
        self.z_samples = []
        self.pitch_samples = []
        self.sample_count = 0
        self.max_samples = 50
        
        # 타이머 생성 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("ArUco Parking Node 시작")

    def rvec_callback(self, msg):
        # 회전 벡터 콜백
        if len(msg.data) >= 3:
            self.rvec_data = np.array(msg.data[:3])
            self.rvec_timestamp = time.time()
            self.marker_detected = True

    def tvec_callback(self, msg):
        # 이동 벡터 콜백
        if len(msg.data) >= 3:
            self.tvec_data = np.array(msg.data[:3])
            self.tvec_timestamp = time.time()

    def rodrigues_to_euler(self, rvec):
        # 로드리게스 벡터를 오일러 각도로 변환
        theta = np.linalg.norm(rvec)
        
        if theta < 1e-6:
            return 0.0, 0.0, 0.0
            
        k = rvec / theta
        
        K = np.array([[0, -k[2], k[1]],
                      [k[2], 0, -k[0]],
                      [-k[1], k[0], 0]])
        
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])  # Roll
            y = math.atan2(-R[2,0], sy)     # Pitch
            z = math.atan2(R[1,0], R[0,0])  # Yaw
        else:
            x = math.atan2(-R[1,2], R[1,1])  # Roll
            y = math.atan2(-R[2,0], sy)      # Pitch
            z = 0.0                          # Yaw
            
        return math.degrees(x), math.degrees(y), math.degrees(z)

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        # Twist 메시지 발행
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        # 로봇 정지
        self.publish_twist(0.0, 0.0)

    def is_data_synchronized(self):
        # 데이터 동기화 확인
        if (self.rvec_data is not None and self.tvec_data is not None and
            self.rvec_timestamp is not None and self.tvec_timestamp is not None):
            return abs(self.rvec_timestamp - self.tvec_timestamp) < 0.1
        return False

    def get_most_frequent_value(self, data_list):
        # 최빈값 계산 (소수점 넷째 자리까지 반올림)
        if not data_list:
            return None
        
        rounded_data = [round(x, 4) for x in data_list]
        counter = Counter(rounded_data)
        most_common = counter.most_common(1)
        return most_common[0][0] if most_common else None

    def reset_to_search(self):
        # 탐색 상태로 리셋
        self.state = "SEARCHING"
        self.marker_detected = False
        self.last_action_time = 0
        self.sample_count = 0
        self.x_samples = []
        self.z_samples = []
        self.pitch_samples = []
        
        # 액션 관련 변수 정리
        if hasattr(self, 'action_type'):
            delattr(self, 'action_type')
        if hasattr(self, 'sequence_type'):
            delattr(self, 'sequence_type')
        if hasattr(self, 'action_phase'):
            delattr(self, 'action_phase')
        if hasattr(self, 'pitch_action_time'):
            delattr(self, 'pitch_action_time')
        if hasattr(self, 'pitch_action_phase'):
            delattr(self, 'pitch_action_phase')

    def search_marker(self):
        # 마커 탐색 상태
        current_time = time.time()
        
        # 마커가 감지되고 데이터가 동기화되었는지 확인
        if self.marker_detected and self.is_data_synchronized():
            self.get_logger().info("마커 감지! 샘플링 시작")
            self.state = "SAMPLING"
            self.sample_count = 0
            self.x_samples = []
            self.z_samples = []
            self.pitch_samples = []
            self.stop_robot()
            return
        
        # 0.2초 CW 회전 후 정지 패턴
        if self.last_action_time == 0:
            # 첫 회전 시작
            self.get_logger().info(f"마커 감지 실패! -> {self.search_rotation_time}초 CW 회전 후 정지")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            self.last_action_time = current_time
            self.action_phase = "rotating"
            
        elif self.action_phase == "rotating" and current_time - self.last_action_time >= self.search_rotation_time:
            # 회전 완료, 정지
            self.stop_robot()
            self.last_action_time = current_time
            self.action_phase = "stopping"
            
        elif self.action_phase == "stopping" and current_time - self.last_action_time >= 0.1:
            # 짧은 정지 후 다시 회전 시작
            self.get_logger().info(f"마커 감지 실패! -> {self.search_rotation_time}초 CW 회전 후 정지")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            self.last_action_time = current_time
            self.action_phase = "rotating"

    def sample_data(self):
        # 데이터 샘플링 상태
        if not self.is_data_synchronized():
            self.get_logger().warn("데이터 동기화 실패, 재탐색")
            self.reset_to_search()
            return
            
        # 현재 데이터 샘플링
        x_value = round(self.tvec_data[0], 4)  # 소수점 4째 자리까지
        z_value = round(self.tvec_data[2], 4)  # 소수점 4째 자리까지
        
        roll, pitch, yaw = self.rodrigues_to_euler(self.rvec_data)
        pitch_value = round(pitch, 4)  # 소수점 4째 자리까지
        
        self.x_samples.append(x_value)
        self.z_samples.append(z_value)
        self.pitch_samples.append(pitch_value)
        self.sample_count += 1
        
        if self.sample_count % 10 == 0:
            self.get_logger().info(f"샘플링 진행: {self.sample_count}/{self.max_samples}")
            self.get_logger().info(f"X: {x_value}m, Z: {z_value}m, Pitch: {pitch_value}°")
        
        if self.sample_count >= self.max_samples:
            self.state = "POSITION_PROCESSING"
            self.get_logger().info("샘플링 완료, 위치 데이터 처리 중...")

    def process_position_samples(self):
        # 위치 샘플 데이터 처리 및 행동 결정 (XZ 좌표만 고려)
        x_mode = self.get_most_frequent_value(self.x_samples)
        z_mode = self.get_most_frequent_value(self.z_samples)
        
        self.get_logger().info(f"위치 최빈값 - X: {x_mode}m, Z: {z_mode}m")
        
        # Z 좌표가 15cm 미만이면 후진
        if z_mode < 0.20:
            self.get_logger().info(f"거리가 너무 가까움 -> {self.backward_time}초간 후진")
            self.execute_action("BACKWARD", self.backward_time)
            
        # X 좌표 기반 행동 결정
        elif x_mode > 0.01:
            self.get_logger().info(f"(마커 시점에서) 카메라가 왼쪽에 위치 -> CCW 회전 후 직진 후 CW 보정 회전")
            self.execute_sequence("CCW_FORWARD_CW")
            
        elif x_mode < -0.01:
            self.get_logger().info(f"(마커 시점에서) 카메라가 오른쪽에 위치 -> CW 회전 후 직진 후 CCW 보정 회전")
            self.execute_sequence("CW_FORWARD_CCW")
            
        else:
            self.get_logger().info("위치 정렬 완료! 자세 샘플링 시작")
            self.state = "ORIENTATION_PROCESSING"

    def process_orientation_samples(self):
        # 자세 샘플 데이터 처리 및 행동 결정 (Pitch 값만 고려)
        pitch_mode = self.get_most_frequent_value(self.pitch_samples)
        
        self.get_logger().info(f"자세 최빈값 - Pitch: {pitch_mode}°")
        
        # Pitch 범위 확인
        if -3.0 <= pitch_mode <= 3.0:
            self.get_logger().info(f"자세 정렬 완료! (Pitch: {pitch_mode}°)")
            self.get_logger().info("주차 완료!")
            self.stop_robot()
            self.state = "PARKING_COMPLETE"
        else:
            self.get_logger().info("자세 조정이 필요합니다. 자세 조정 단계로 이동")
            self.state = "ORIENTATION_ADJUSTMENT"

    def execute_action(self, action, duration):
        # 단일 액션 실행
        self.action_start_time = time.time()
        self.action_type = action
        self.action_duration = duration
        self.state = "EXECUTING"
        
        if action == "BACKWARD":
            self.publish_twist(-self.linear_speed, 0.0)

    def execute_sequence(self, sequence):
        # 3단계 연속 액션 실행 (회전 - 직진 - 보정회전)
        self.sequence_type = sequence
        self.sequence_step = 0
        self.action_start_time = time.time()
        self.state = "EXECUTING"
        
        if sequence == "CW_FORWARD_CCW":
            self.get_logger().info(f"{self.movement_rotation_time}초 CW 회전 시작")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
        elif sequence == "CCW_FORWARD_CW":
            self.get_logger().info(f"{self.movement_rotation_time}초 CCW 회전 시작")
            self.publish_twist(0.0, self.angular_speed)   # CCW 회전

    def handle_execution(self):
        # 액션 실행 처리
        current_time = time.time()
        elapsed = current_time - self.action_start_time
        
        if hasattr(self, 'action_type'):  # 단일 액션
            if elapsed >= self.action_duration:
                self.stop_robot()
                self.get_logger().info("액션 완료, 재탐색 시작")
                self.reset_to_search()
                
        elif hasattr(self, 'sequence_type'):  # 3단계 연속 액션
            if self.sequence_step == 0 and elapsed >= self.movement_rotation_time:
                # 1단계 회전 완료, 직진 시작
                self.get_logger().info(f"1단계 회전 완료, {self.movement_forward_time}초 직진 시작")
                self.publish_twist(self.linear_speed, 0.0)
                self.sequence_step = 1
                self.action_start_time = current_time
                
            elif self.sequence_step == 1 and elapsed >= self.movement_forward_time:
                # 2단계 직진 완료, 보정 회전 시작
                if self.sequence_type == "CW_FORWARD_CCW":
                    self.get_logger().info(f"직진 완료, {self.correction_rotation_time}초 CCW 보정 회전 시작")
                    self.publish_twist(0.0, self.angular_speed)  # CCW 보정 회전
                elif self.sequence_type == "CCW_FORWARD_CW":
                    self.get_logger().info(f"직진 완료, {self.correction_rotation_time}초 CW 보정 회전 시작")
                    self.publish_twist(0.0, -self.angular_speed)  # CW 보정 회전
                
                self.sequence_step = 2
                self.action_start_time = current_time
                
            elif self.sequence_step == 2 and elapsed >= self.correction_rotation_time:
                # 3단계 보정 회전 완료
                self.stop_robot()
                self.get_logger().info("3단계 연속 액션 완료, 재탐색 시작")
                self.reset_to_search()

    def adjust_orientation(self):
        # 자세 조정 상태 (Pitch만 고려)
        if not self.is_data_synchronized():
            self.get_logger().warn("데이터 동기화 실패, 재탐색")
            self.reset_to_search()
            return
        
        # 현재 Pitch 값 확인
        roll, pitch, yaw = self.rodrigues_to_euler(self.rvec_data)
        current_pitch = round(pitch, 4)
        
        current_time = time.time()
        
        # Pitch 범위 확인
        if -5.0 <= current_pitch <= 5.0:
            self.get_logger().info(f"자세 조정 완료! (현재 Pitch: {current_pitch}°)")
            self.get_logger().info("주차 완료!")
            print("주차 완료")
            self.stop_robot()
            self.state = "PARKING_COMPLETE"
            return
        
        # 0.2초 회전 후 정지 패턴
        if not hasattr(self, 'pitch_action_time'):
            self.pitch_action_time = 0
            self.pitch_action_phase = "ready"
        
        if self.pitch_action_phase == "ready":
            if current_pitch < -5.0:
                self.get_logger().info(f"Pitch가 너무 낮음 ({current_pitch}°) -> {self.pitch_rotation_time}초 CCW 회전 후 정지")
                self.publish_twist(0.0, self.angular_speed)  # CCW 회전
            else:  # current_pitch > 5.0
                self.get_logger().info(f"Pitch가 너무 높음 ({current_pitch}°) -> {self.pitch_rotation_time}초 CW 회전 후 정지")
                self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            
            self.pitch_action_time = current_time
            self.pitch_action_phase = "rotating"
            
        elif self.pitch_action_phase == "rotating" and current_time - self.pitch_action_time >= self.pitch_rotation_time:
            # 회전 완료, 정지
            self.stop_robot()
            self.pitch_action_time = current_time
            self.pitch_action_phase = "stopping"
            
        elif self.pitch_action_phase == "stopping" and current_time - self.pitch_action_time >= 0.1:
            # 짧은 정지 후 다시 확인
            self.pitch_action_phase = "ready"

    def timer_callback(self):
        # 메인 타이머 콜백 (10Hz)
        if self.state == "SEARCHING":
            self.search_marker()
            
        elif self.state == "SAMPLING":
            self.sample_data()
            
        elif self.state == "POSITION_PROCESSING":
            self.process_position_samples()
            
        elif self.state == "POSITION_ADJUSTMENT":
            # 현재는 사용하지 않음 (EXECUTING 상태에서 처리)
            pass
            
        elif self.state == "ORIENTATION_PROCESSING":
            self.process_orientation_samples()
            
        elif self.state == "ORIENTATION_ADJUSTMENT":
            self.adjust_orientation()
            
        elif self.state == "EXECUTING":
            self.handle_execution()
            
        elif self.state == "PARKING_COMPLETE":
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArucoParkingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램이 종료되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if 'node' in locals():
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
from collections import Counter

class ArucoParkingNode(Node):
    def __init__(self):
        super().__init__('aruco_parking')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rvec_sub = self.create_subscription(
            Float32MultiArray,
            '/DP03/aruco_rvec_topic',
            self.rvec_callback,
            1  # 큐 사이즈 1
        )
        self.tvec_sub = self.create_subscription(
            Float32MultiArray,
            '/DP03/aruco_tvec_topic',
            self.tvec_callback,
            1  # 큐 사이즈 1
        )
        
        # State variables
        self.rvec_data = None
        self.tvec_data = None
        self.rvec_timestamp = None
        self.tvec_timestamp = None
        self.marker_detected = False
        
        # 속도 설정
        self.linear_speed = 0.01  # m/s
        self.angular_speed = 0.02  # rad/s
        
        # 시간 설정 변수
        self.search_rotation_time = 0.2  # 마커 탐색 시 회전 시간
        self.movement_rotation_time = 0.2  # 이동 시 회전 시간
        self.movement_forward_time = 0.2  # 이동 시 직진 시간
        self.correction_rotation_time = 0.2  # 보정 회전 시간
        self.pitch_rotation_time = 0.2  # Pitch 조정 시 회전 시간
        self.backward_time = 1.0  # 후진 시간
        
        # 상태 관리 - POSITION_ADJUSTMENT와 ORIENTATION_ADJUSTMENT 분리
        self.state = "SEARCHING"  # SEARCHING, SAMPLING, POSITION_PROCESSING, POSITION_ADJUSTMENT, ORIENTATION_PROCESSING, ORIENTATION_ADJUSTMENT, FINAL_POSITIONING, EXECUTING, PARKING_COMPLETE
        self.last_action_time = 0
        
        # 적응형 샘플링 변수
        self.x_samples = []
        self.z_samples = []
        self.pitch_samples = []
        self.sample_count = 0
        self.min_samples = 30  # 최소 샘플링 횟수
        self.additional_samples = 10  # 추가 샘플링 횟수
        self.confidence_threshold = 0.6  # 최빈값 신뢰도 임계값
        
        # 타이머 생성 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("ArUco Parking Node 시작")

    def rvec_callback(self, msg):
        # 회전 벡터 콜백
        if len(msg.data) >= 3:
            self.rvec_data = np.array(msg.data[:3])
            self.rvec_timestamp = time.time()
            self.marker_detected = True

    def tvec_callback(self, msg):
        # 이동 벡터 콜백
        if len(msg.data) >= 3:
            self.tvec_data = np.array(msg.data[:3])
            self.tvec_timestamp = time.time()

    def rodrigues_to_euler(self, rvec):
        # 로드리게스 벡터를 오일러 각도로 변환
        theta = np.linalg.norm(rvec)
        
        if theta < 1e-6:
            return 0.0, 0.0, 0.0
            
        k = rvec / theta
        
        K = np.array([[0, -k[2], k[1]],
                      [k[2], 0, -k[0]],
                      [-k[1], k[0], 0]])
        
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])  # Roll
            y = math.atan2(-R[2,0], sy)     # Pitch
            z = math.atan2(R[1,0], R[0,0])  # Yaw
        else:
            x = math.atan2(-R[1,2], R[1,1])  # Roll
            y = math.atan2(-R[2,0], sy)      # Pitch
            z = 0.0                          # Yaw
            
        return math.degrees(x), math.degrees(y), math.degrees(z)

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        # Twist 메시지 발행
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        # 로봇 정지
        self.publish_twist(0.0, 0.0)

    def is_data_synchronized(self):
        # 데이터 동기화 확인
        if (self.rvec_data is not None and self.tvec_data is not None and
            self.rvec_timestamp is not None and self.tvec_timestamp is not None):
            return abs(self.rvec_timestamp - self.tvec_timestamp) < 0.1
        return False

    def get_most_frequent_value_with_confidence(self, data_list):
        # 최빈값과 신뢰도 계산
        if not data_list:
            return None, 0.0
        
        rounded_data = [round(x, 4) for x in data_list]
        counter = Counter(rounded_data)
        most_common = counter.most_common(1)
        
        if not most_common:
            return None, 0.0
            
        most_frequent_value = most_common[0][0]
        most_frequent_count = most_common[0][1]
        confidence = most_frequent_count / len(data_list)
        
        return most_frequent_value, confidence

    def reset_to_search(self):
        # 탐색 상태로 리셋
        self.state = "SEARCHING"
        self.marker_detected = False
        self.last_action_time = 0
        self.sample_count = 0
        self.x_samples = []
        self.z_samples = []
        self.pitch_samples = []
        
        # 액션 관련 변수 정리
        if hasattr(self, 'action_type'):
            delattr(self, 'action_type')
        if hasattr(self, 'sequence_type'):
            delattr(self, 'sequence_type')
        if hasattr(self, 'action_phase'):
            delattr(self, 'action_phase')
        if hasattr(self, 'pitch_action_time'):
            delattr(self, 'pitch_action_time')
        if hasattr(self, 'final_action_time'):
            delattr(self, 'final_action_time')
        if hasattr(self, 'final_action_phase'):
            delattr(self, 'final_action_phase')

    def search_marker(self):
        # 마커 탐색 상태
        current_time = time.time()
        
        # 마커가 감지되고 데이터가 동기화되었는지 확인
        if self.marker_detected and self.is_data_synchronized():
            self.get_logger().info("마커 감지! 샘플링 시작")
            self.state = "SAMPLING"
            self.sample_count = 0
            self.x_samples = []
            self.z_samples = []
            self.pitch_samples = []
            self.stop_robot()
            return
        
        # 0.2초 CW 회전 후 정지 패턴
        if self.last_action_time == 0:
            # 첫 회전 시작
            self.get_logger().info(f"마커 감지 실패! -> {self.search_rotation_time}초 CW 회전 후 정지")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            self.last_action_time = current_time
            self.action_phase = "rotating"
            
        elif self.action_phase == "rotating" and current_time - self.last_action_time >= self.search_rotation_time:
            # 회전 완료, 정지
            self.stop_robot()
            self.last_action_time = current_time
            self.action_phase = "stopping"
            
        elif self.action_phase == "stopping" and current_time - self.last_action_time >= 0.1:
            # 짧은 정지 후 다시 회전 시작
            self.get_logger().info(f"마커 감지 실패! -> {self.search_rotation_time}초 CW 회전 후 정지")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            self.last_action_time = current_time
            self.action_phase = "rotating"

    def sample_data(self):
        # 적응형 데이터 샘플링 상태
        if not self.is_data_synchronized():
            self.get_logger().warn("데이터 동기화 실패, 재탐색")
            self.reset_to_search()
            return
            
        # 현재 데이터 샘플링
        x_value = round(self.tvec_data[0], 4)  # 소수점 4째 자리까지
        z_value = round(self.tvec_data[2], 4)  # 소수점 4째 자리까지
        
        roll, pitch, yaw = self.rodrigues_to_euler(self.rvec_data)
        pitch_value = round(pitch, 4)  # 소수점 4째 자리까지
        
        self.x_samples.append(x_value)
        self.z_samples.append(z_value)
        self.pitch_samples.append(pitch_value)
        self.sample_count += 1
        
        if self.sample_count % 10 == 0:
            self.get_logger().info(f"샘플링 진행: {self.sample_count}회")
            self.get_logger().info(f"X: {x_value}m, Z: {z_value}m, Pitch: {pitch_value}°")
        
        # 최소 샘플링 완료 후 신뢰도 검사
        if self.sample_count >= self.min_samples:
            # 각 데이터의 최빈값과 신뢰도 계산
            x_mode, x_confidence = self.get_most_frequent_value_with_confidence(self.x_samples)
            z_mode, z_confidence = self.get_most_frequent_value_with_confidence(self.z_samples)
            pitch_mode, pitch_confidence = self.get_most_frequent_value_with_confidence(self.pitch_samples)
            
            # self.get_logger().info(f"신뢰도 검사 - X: {x_confidence:.2%}, Z: {z_confidence:.2%}, Pitch: {pitch_confidence:.2%}")
            
            # 모든 데이터가 신뢰도 임계값을 넘는지 확인
            if (x_confidence >= self.confidence_threshold and 
                z_confidence >= self.confidence_threshold and 
                pitch_confidence >= self.confidence_threshold):
                
                self.get_logger().info(f"샘플링 완료! (총 {self.sample_count}회 샘플링)")
                self.get_logger().info(f"최종 신뢰도 - X: {x_confidence:.2%}, Z: {z_confidence:.2%}, Pitch: {pitch_confidence:.2%}")
                self.state = "POSITION_PROCESSING"
                return
            
            # 추가 샘플링이 필요한 경우 (10회씩 추가)
            elif self.sample_count % self.additional_samples == 0:
                self.get_logger().info(f"신뢰도 부족으로 {self.additional_samples}회 추가 샘플링 수행")
                # 계속 샘플링 진행

    def process_position_samples(self):
        # 위치 샘플 데이터 처리 및 행동 결정 (XZ 좌표만 고려)
        x_mode, _ = self.get_most_frequent_value_with_confidence(self.x_samples)
        z_mode, _ = self.get_most_frequent_value_with_confidence(self.z_samples)
        
        self.get_logger().info(f"위치 최빈값 - X: {x_mode}m, Z: {z_mode}m")
        
        # Z 좌표가 15cm 미만이면 후진
        if z_mode < 0.15:
            self.get_logger().info(f"거리가 너무 가까움 -> {self.backward_time}초간 후진")
            self.execute_action("BACKWARD", self.backward_time)
            
        # X 좌표 기반 행동 결정 (수정된 부분)
        elif x_mode > 0.01:
            self.get_logger().info(f"(마커 시점에서) 카메라가 왼쪽에 위치 -> CCW 회전 후 직진 후 CW 보정 회전")
            self.execute_sequence("CCW_FORWARD_CW")
            
        elif x_mode < -0.01:
            self.get_logger().info(f"(마커 시점에서) 카메라가 오른쪽에 위치 -> CW 회전 후 직진 후 CCW 보정 회전")
            self.execute_sequence("CW_FORWARD_CCW")
            
        else:
            self.get_logger().info("위치 정렬 완료! 자세 샘플링 시작")
            self.state = "ORIENTATION_PROCESSING"

    def process_orientation_samples(self):
        # 자세 샘플 데이터 처리 및 행동 결정 (Pitch 값만 고려)
        pitch_mode, _ = self.get_most_frequent_value_with_confidence(self.pitch_samples)
        
        self.get_logger().info(f"자세 최빈값 - Pitch: {pitch_mode}°")
        
        # Pitch 범위 확인
        if -5.0 <= pitch_mode <= 5.0:
            self.get_logger().info(f"자세 정렬 완료! (Pitch: {pitch_mode}°)")
            self.get_logger().info("최종 위치 조정 단계로 이동")
            self.state = "FINAL_POSITIONING"
        else:
            self.get_logger().info("자세 조정이 필요합니다. 자세 조정 단계로 이동")
            self.state = "ORIENTATION_ADJUSTMENT"

    def execute_action(self, action, duration):
        # 단일 액션 실행
        self.action_start_time = time.time()
        self.action_type = action
        self.action_duration = duration
        self.state = "EXECUTING"
        
        if action == "BACKWARD":
            self.publish_twist(-self.linear_speed, 0.0)

    def execute_sequence(self, sequence):
        # 3단계 연속 액션 실행 (회전 - 직진 - 보정회전)
        self.sequence_type = sequence
        self.sequence_step = 0
        self.action_start_time = time.time()
        self.state = "EXECUTING"
        
        if self.sequence_type == "CW_FORWARD_CCW":
            self.get_logger().info(f"{self.movement_rotation_time}초 CW 회전 시작")
            self.publish_twist(0.0, -self.angular_speed)  # CW 회전
        elif self.sequence_type == "CCW_FORWARD_CW":
            self.get_logger().info(f"{self.movement_rotation_time}초 CCW 회전 시작")
            self.publish_twist(0.0, self.angular_speed)   # CCW 회전

    def handle_execution(self):
        # 액션 실행 처리
        current_time = time.time()
        elapsed = current_time - self.action_start_time
        
        if hasattr(self, 'action_type'):  # 단일 액션
            if elapsed >= self.action_duration:
                self.stop_robot()
                self.get_logger().info("액션 완료, 재탐색 시작")
                self.reset_to_search()
                
        elif hasattr(self, 'sequence_type'):  # 3단계 연속 액션
            if self.sequence_step == 0 and elapsed >= self.movement_rotation_time:
                # 1단계 회전 완료, 직진 시작
                self.get_logger().info(f"1단계 회전 완료, {self.movement_forward_time}초 직진 시작")
                self.publish_twist(self.linear_speed, 0.0)
                self.sequence_step = 1
                self.action_start_time = current_time
                
            elif self.sequence_step == 1 and elapsed >= self.movement_forward_time:
                # 2단계 직진 완료, 보정 회전 시작
                if self.sequence_type == "CW_FORWARD_CCW":
                    self.get_logger().info(f"직진 완료, {self.correction_rotation_time}초 CCW 보정 회전 시작")
                    self.publish_twist(0.0, self.angular_speed)  # CCW 보정 회전
                elif self.sequence_type == "CCW_FORWARD_CW":
                    self.get_logger().info(f"직진 완료, {self.correction_rotation_time}초 CW 보정 회전 시작")
                    self.publish_twist(0.0, -self.angular_speed)  # CW 보정 회전
                
                self.sequence_step = 2
                self.action_start_time = current_time
                
            elif self.sequence_step == 2 and elapsed >= self.correction_rotation_time:
                # 3단계 보정 회전 완료
                self.stop_robot()
                self.get_logger().info("3단계 연속 액션 완료, 재탐색 시작")
                self.reset_to_search()

    def adjust_orientation(self):
        # 자세 조정 상태 (Pitch만 고려)
        if not self.is_data_synchronized():
            self.get_logger().warn("데이터 동기화 실패, 재탐색")
            self.reset_to_search()
            return
        
        # 현재 Pitch 값 확인
        roll, pitch, yaw = self.rodrigues_to_euler(self.rvec_data)
        current_pitch = round(pitch, 4)
        
        current_time = time.time()
        
        # Pitch 범위 확인
        if -5.0 <= current_pitch <= 5.0:
            self.get_logger().info(f"자세 조정 완료! (현재 Pitch: {current_pitch}°)")
            self.get_logger().info("최종 위치 조정 단계로 이동")
            self.state = "FINAL_POSITIONING"
            return
        
        # 0.2초 회전 후 정지 패턴
        if not hasattr(self, 'pitch_action_time'):
            self.pitch_action_time = 0
            self.pitch_action_phase = "ready"
        
        if self.pitch_action_phase == "ready":
            if current_pitch < -5.0:
                self.get_logger().info(f"Pitch가 너무 낮음 ({current_pitch}°) -> {self.pitch_rotation_time}초 CCW 회전 후 정지")
                self.publish_twist(0.0, self.angular_speed)  # CCW 회전
            else:  # current_pitch > 5.0
                self.get_logger().info(f"Pitch가 너무 높음 ({current_pitch}°) -> {self.pitch_rotation_time}초 CW 회전 후 정지")
                self.publish_twist(0.0, -self.angular_speed)  # CW 회전
            
            self.pitch_action_time = current_time
            self.pitch_action_phase = "rotating"
            
        elif self.pitch_action_phase == "rotating" and current_time - self.pitch_action_time >= self.pitch_rotation_time:
            # 회전 완료, 정지
            self.stop_robot()
            self.pitch_action_time = current_time
            self.pitch_action_phase = "stopping"
            
        elif self.pitch_action_phase == "stopping" and current_time - self.pitch_action_time >= 0.1:
            # 짧은 정지 후 다시 확인
            self.pitch_action_phase = "ready"

    def final_positioning(self):
        # 최종 위치 조정 상태 (Z 좌표를 0.19~0.21로 조정)
        if not self.is_data_synchronized():
            self.get_logger().warn("데이터 동기화 실패, 재탐색")
            self.reset_to_search()
            return
        
        current_z = round(self.tvec_data[2], 4)
        current_time = time.time()
        
        # 목표 Z 범위 확인 (0.19 ~ 0.21)
        if 0.19 <= current_z <= 0.21:
            self.get_logger().info(f"최종 위치 조정 완료! (현재 Z: {current_z}m)")
            self.get_logger().info("주차 완료!")
            print("주차 완료")
            self.stop_robot()
            self.state = "PARKING_COMPLETE"
            return
        
        # 0.1초 이동 후 정지 패턴
        if not hasattr(self, 'final_action_time'):
            self.final_action_time = 0
            self.final_action_phase = "ready"
        
        if self.final_action_phase == "ready":
            if current_z < 0.19:
                self.get_logger().info(f"Z가 너무 가까움 ({current_z}m) -> 0.1초 후진")
                self.publish_twist(-self.linear_speed, 0.0)  # 후진
            else:  # current_z > 0.21
                self.get_logger().info(f"Z가 너무 멀음 ({current_z}m) -> 0.1초 전진")
                self.publish_twist(self.linear_speed, 0.0)  # 전진
            
            self.final_action_time = current_time
            self.final_action_phase = "moving"
            
        elif self.final_action_phase == "moving" and current_time - self.final_action_time >= 0.1:
            # 이동 완료, 정지
            self.stop_robot()
            self.final_action_time = current_time
            self.final_action_phase = "stopping"
            
        elif self.final_action_phase == "stopping" and current_time - self.final_action_time >= 0.1:
            # 짧은 정지 후 다시 확인
            self.final_action_phase = "ready"

    def timer_callback(self):
        # 메인 타이머 콜백 (10Hz)
        if self.state == "SEARCHING":
            self.search_marker()
            
        elif self.state == "SAMPLING":
            self.sample_data()
            
        elif self.state == "POSITION_PROCESSING":
            self.process_position_samples()
            
        elif self.state == "POSITION_ADJUSTMENT":
            # 현재는 사용하지 않음 (EXECUTING 상태에서 처리)
            pass
            
        elif self.state == "ORIENTATION_PROCESSING":
            self.process_orientation_samples()
            
        elif self.state == "ORIENTATION_ADJUSTMENT":
            self.adjust_orientation()
            
        elif self.state == "FINAL_POSITIONING":
            self.final_positioning()
            
        elif self.state == "EXECUTING":
            self.handle_execution()
            
        elif self.state == "PARKING_COMPLETE":
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArucoParkingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램이 종료되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if 'node' in locals():
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()