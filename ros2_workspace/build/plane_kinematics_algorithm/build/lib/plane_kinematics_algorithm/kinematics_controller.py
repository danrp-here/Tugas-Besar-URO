import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math

class KinematicsController(Node):
    def __init__(self):
        super().__init__('kinematics_controller')
        
        # Subscribe to plane pose topic to receive its position and orientation
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/plane/pose',   # We'll assume Gazebo or another component publishes plane's pose
            self.pose_callback,
            10
        )
        
        # Publisher for the payload drop signal
        self.payload_drop_publisher = self.create_publisher(Bool, '/plane/drop_payload', 10)

        self.get_logger().info("Kinematics Controller for Plane Initialized!")

    def pose_callback(self, msg):
        """
        This callback will receive the plane's position and calculate when to drop the payload based on velocity and position.
        """
        # Calculate velocity based on the change in position over time
        plane_position = msg.position.z  # Z position (altitude)
        plane_velocity = math.sqrt(msg.position.x ** 2 + msg.position.y ** 2)  # Basic kinematic velocity (X and Y motion)

        # Determine the conditions for releasing the payload based on kinematics
        drop_threshold_height = 20.0  # z position (altitude) at which to consider releasing payload
        drop_threshold_velocity = 10.0  # Velocity above which to trigger payload drop
        
        # If the plane's altitude is above the threshold and the velocity is significant, drop the payload
        if plane_position >= drop_threshold_height and plane_velocity >= drop_threshold_velocity:
            self.get_logger().info("Payload dropping due to plane velocity and altitude!")
            drop_msg = Bool()
            drop_msg.data = True
            self.payload_drop_publisher.publish(drop_msg)

def main(args=None):
    rclpy.init(args=args)
    kinematics_controller = KinematicsController()
    rclpy.spin(kinematics_controller)
    kinematics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

