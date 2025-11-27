#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler, quaternion_multiply

    # z축 회전행렬, 각속도/선가속도 벡터에 곱할 때 사용
def Rz(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[ c,-s, 0.0],
                     [ s, c, 0.0],
                     [0.0,0.0, 1.0]], dtype=float)

    # 쿼터니언 단위화,  수치오차로 인한 NaN/발산 방지
def normalize_quat(q):
    # q = [x,y,z,w]
    n = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
    if n <= 0.0:
        return [0.0,0.0,0.0,1.0]
    return [q[0]/n, q[1]/n, q[2]/n, q[3]/n]

class ImuOffsetNode(Node):
    def __init__(self):
        super().__init__('imu_offset_node')
        # 파라미터
        self.declare_parameter('yaw_offset', 0.0)   # rad
        self.declare_parameter('enable', True)

        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.enable = bool(self.get_parameter('enable').value)

        # 입력/출력
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 50) # 가제보: /imu, 현실: /imu/data
        self.pub_imu = self.create_publisher(Imu, '/imu_cal', 50)

        # 오프셋 실시간 갱신 채널
        self.sub_offset = self.create_subscription(Float64, '/yaw_offset', self.set_offset_cb, 10)
        self.get_logger().info('[IMU_OFFSET] yaw_offset=%.4f rad' % self.yaw_offset)

    def set_offset_cb(self, msg: Float64):
        self.yaw_offset = float(msg.data)
        self.get_logger().info('[IMU_OFFSET] set yaw_offset=%.4f rad' % self.yaw_offset)

    # 정규화: [-pi, pi] 유지
    @staticmethod
    def _norm_angle(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def imu_cb(self, m: Imu):
        # enable이 꺼져 있거나 yaw_offset==0이면 패스스루
        if (not self.enable) or abs(self.yaw_offset) < 1e-12:
            self.pub_imu.publish(m)
            return

        try:
            # 회전 행렬/오프셋 쿼터니언
            R = Rz(self.yaw_offset)
            q_off = quaternion_from_euler(0.0, 0.0, self.yaw_offset)  # [x,y,z,w]

            # orientation 보정: q_cal = q_off ⊗ q_raw
            q_raw = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
            q_cal = quaternion_multiply(q_off, q_raw)
            q_cal = normalize_quat(q_cal)

            # 각속도/선가속도 보정: v_cal = R * v_raw
            w_raw = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z], dtype=float)
            a_raw = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z], dtype=float)
            w_cal = R @ w_raw
            a_cal = R @ a_raw

            # 메시지 구성
            out = Imu()
            out.header = m.header  # stamp, frame_id 그대로
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_cal
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = float(w_cal[0]), float(w_cal[1]), float(w_cal[2])
            out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = float(a_cal[0]), float(a_cal[1]), float(a_cal[2])

            # 공분산: 그대로 복사 yaw var=0 경고 뜨면 작은 값 넣어줘도 됨.
            out.orientation_covariance    = m.orientation_covariance
            out.angular_velocity_covariance = m.angular_velocity_covariance
            out.linear_acceleration_covariance = m.linear_acceleration_covariance

            self.pub_imu.publish(out)
            # 과거 yaw(degree) 와 비교한 현재 yaw(degree) 한 번만 로그
            self.get_logger().info(f'[IMU_OFFSET] yaw_offset={self.yaw_offset * 180.0 / math.pi:.4f} deg', once=True)

        except Exception as e:
            # 실패 시 원본 패스(로그만)
            self.get_logger().warn(f'[IMU_OFFSET] imu/cal failed, passthrough. err={e}')
            self.pub_imu.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOffsetNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
