import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ros2_basic_msgs.msg import Age 



class CustomInterfacePublisher(Node):

    def __init__(self):
        super().__init__('custom_interfaces_publisher')
        self.publisher_ = self.create_publisher(Age, 'age', 10)
        timer_period = 0.5  # seconds
        self.age = Age()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.age.year = 1996
        self.age.month = 9
        self.age.day = 13
        self.publisher_.publish(self.age)
        self.get_logger().info('Publishing: Day: {}, Month {}, Year {}'.format(self.age.day, self.age.month, self.age.year))


def main(args=None):
    rclpy.init(args=args)
    custom_interface_publisher = CustomInterfacePublisher()
    rclpy.spin(custom_interface_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    custom_interface_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()