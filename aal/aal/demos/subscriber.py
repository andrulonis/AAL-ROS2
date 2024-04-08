import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

SUB_TOPIC_PARAM = "subscriber_topic_name"

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.declare_parameter(SUB_TOPIC_PARAM, 'lifecycle_chatter')

        self.topic_name = self.get_parameter(SUB_TOPIC_PARAM).get_parameter_value().string_value

        self.create_subscriber()
        self.add_on_set_parameters_callback(self.parameter_changed_callback)

        
    def create_subscriber(self):
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.listener_callback,
            10)


    def parameter_changed_callback(self, params):
        # do some actions, validate parameters, update class attributes, etc.
        for param in params:
            if(param.name == SUB_TOPIC_PARAM and param.value != self.topic_name):
                self.topic_name = param.value
                self.destroy_subscription(self.subscription)
                self.create_subscriber() #replace prev subscriber
                self.get_logger().info('Replaced current subscriber with subscriber to topic: "%s"' % param.value)


        return SetParametersResult(successful=True)
    
    def listener_callback(self, msg):
        
        self.get_logger().info('I heard: "%s"' % msg.data)


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
