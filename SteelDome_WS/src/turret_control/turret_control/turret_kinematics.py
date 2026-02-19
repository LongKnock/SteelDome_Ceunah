import rclpy
from rclpy.node import Node
from turret_msgs.msg import TargetState, TurretState
from std_msgs.msg import Float64MultiArray
import math

class TurretKinematics(Node):
    def __init__(self):
        super().__init__('turret_controller')

        # Subsriber
        self.turret_cmd_pos_sub_ = self.create_subscription(TurretState, '/turret/cmd_pos', self.turretPosCallback, 10)
        self.motor_pos_sub_ = self.create_subscription(Float64MultiArray, '/motor/pos', self.motorPosCallback, 10)

        # Publisher
        self.motor_cmd_pos_pub_ = self.create_publisher(Float64MultiArray, '/motor/cmd_pos', 10)
        self.turret_pos_pub_ = self.create_publisher(TurretState, '/turret/pos', 10)

        # State Variables
        self.current_pitch = 0
        self.current_yaw = 0
        self.current_pitch_vel = 0
        self.current_yaw_vel = 0

    def turretPosCallback(self, msg):
        motor_pos = self.calcIK(msg.pitch, msg.yaw)

        motor_pos_msg = Float64MultiArray()
        motor_pos_msg.data = motor_pos
    
        self.motor_cmd_pos_pub_.publish(motor_pos_msg)

    def motorPosCallback(self, msg):
        motor_pos = msg.data

        turret_pos = self.calcFK(motor_pos)

        turret_pos_msg = TurretState()
        turret_pos_msg.pitch = turret_pos[0]
        turret_pos_msg.yaw = turret_pos[1]

        self.turret_pos_pub_.publish(turret_pos_msg)

    def calcIK(self, pitch, yaw):
        motor_pos = [0.0, 0.0]
        motor_pos[0] = (14.0 / 3.0) * (pitch + yaw)
        motor_pos[1] = (14.0 / 3.0) * (pitch - yaw)

        return motor_pos
    
    def calcFK(self, motor_pos):
        pitch = (3.0 / 14.0) * (motor_pos[0] + motor_pos[1]) / 2.0
        yaw = (3.0 / 14.0) * (motor_pos[0] - motor_pos[1]) / 2.0

        return pitch, yaw
    
def main(args=None):
    rclpy.init(args=args)
    turret_controller = TurretKinematics()
    rclpy.spin(turret_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
