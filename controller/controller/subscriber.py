import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/model/bot/cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        message = Twist()
        message.linear.x = 0.0
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.y = 0.0
        if msg.data==16777235:
            message.linear.x = -1.0
            self.get_logger().info('conditon_enable')

            self.publisher_.publish(message)
            self.get_logger().info('published message')
          
        elif msg.data == 16777234:
            message.angular.z = 1.0
            self.publisher_.publish(message)
        elif msg.data == 16777237:
            message.linear.x = +1.0
            self.publisher_.publish(message)
        elif msg.data == 16777236:
            message.angular.z = -1.0
            self.publisher_.publish(message)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()