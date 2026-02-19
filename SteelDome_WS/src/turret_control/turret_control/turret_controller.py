import rclpy
from rclpy.node import Node
from turret_msgs.msg import TargetState, TurretState
from geometry_msgs.msg import Point
import math

class TurretController(Node):
    def __init__(self):
        super().__init__('turret_controller')

        # Subsriber
        self.target_state_sub_ = self.create_subscription(TargetState, '/target/state', self.targetStateCallback, 10)
        self.turret_pos_sub_ = self.create_subscription(TurretState, '/turret/pos', self.turretPosCallback, 10)
        self.turret_vel_sub_ = self.create_subscription(TurretState, '/turret/vel', self.turretVelCallback, 10)

        # Publisher
        self.turret_pos_cmd_pub_ = self.create_publisher(TurretState, '/turret/cmd_pos', 10)
        self.turret_vel_cmd_pub_ = self.create_publisher(TurretState, '/turret/cmd_vel', 10)

        # State Variables
        self.current_pitch = 0
        self.current_yaw = 0
        self.current_pitch_vel = 0
        self.current_yaw_vel = 0

    def turretPosCallback(self, msg):
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw

    def turretVelCallback(self, msg):
        self.current_pitch_vel = msg.pitch
        self.current_yaw_vel = msg.yaw

    def targetStateCallback(self, msg):
        # Extract data
        px, py, pz = msg.position.x, msg.position.y, msg.position.z
        vx, vy, vz = msg.velocity.x, msg.velocity.y, msg.velocity.z

        distance = math.sqrt(px**2 + py**2 + pz**2)

def main(args=None):
    rclpy.init(args=args)
    turret_controller = TurretController()
    rclpy.spin(turret_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
