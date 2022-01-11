from msg_interface.msg import RVD
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3, Twist



class RvdToCmd(Node):

    def __init__(self):
        self.radius = 1.0
        self.velocity = 0.0
        self.direction = True
        self.linear = None
        self.angular = None
        self.twist = None
        self.linear_x = 0.0
        self.angular_z = 0.0

        super().__init__('rvd_to_cmd')

        qos_profile = QoSProfile(depth = 10)

        self.rvd_sub = self.create_subscription(
            RVD,
            'rad_vel_dir',
            self.subscribe_rvd_msg,
            qos_profile)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            qos_profile)

        self.timer = self.create_timer(1.0, self.publish_cmd_msg)
        self.count = 0

    def subscribe_rvd_msg(self,msg):
        self.radius = msg.radius
        self.velocity = msg.velocity
        self.direction = msg.direction

    def publish_cmd_msg(self):
        self.twist = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

        self.linear_x = self.velocity
        if self.direction == True:
            self.angular_z = self.velocity / self.radius
        elif self.direction != True:
            self.angular_z = -(self.velocity)/(self.radius)

        self.linear.x = self.linear_x
        self.linear.y = 0.0
        self.linear.z = 0.0

        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = self.angular_z

        self.twist.linear = self.linear
        self.twist.angular = self.angular

        self.cmd_pub.publish(self.twist)





def main(args=None):
    rclpy.init(args=args)
    try:
        rvd_to_cmd = RvdToCmd()
        try:
            rclpy.spin(rvd_to_cmd)
        except KeyboardInterrupt:
            rvd_to_cmd.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            rvd_to_cmd.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


