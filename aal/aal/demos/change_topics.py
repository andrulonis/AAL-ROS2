import sys

from aal_msgs.srv import AdaptArchitecture
from aal_msgs.msg import Adaptation
from rcl_interfaces.msg import Parameter, ParameterValue
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node
import copy
import time

TOPIC_TO_CHANGE_TO = "some_other_topic"

ACTIVATE_TRANSITION = Transition()
ACTIVATE_TRANSITION.id = Transition.TRANSITION_ACTIVATE

PUB_TOPIC_PARAM = rclpy.parameter.Parameter("publisher_topic_name",value=TOPIC_TO_CHANGE_TO).to_parameter_msg()
SUB_TOPIC_PARAM = rclpy.parameter.Parameter("subscriber_topic_name",value=TOPIC_TO_CHANGE_TO).to_parameter_msg()


class ChangeTopicsClient(Node):

    def __init__(self):
        super().__init__('change_topics_client')
        self.cli = self.create_client(AdaptArchitecture, '/adapt_architecture')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AdaptArchitecture.Request()

        

    def send_request(self, adaptations):
        self.req.adaptations = adaptations
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    change_topic_cli = ChangeTopicsClient()



    activate_publisherB = Adaptation()
    activate_publisherB.adaptation_target = Adaptation.STATETRANSITION
    activate_publisherB.node_name = "publisher_B"
    activate_publisherB.lifecycle_adaptation = ACTIVATE_TRANSITION

    response = change_topic_cli.send_request([activate_publisherB])

    change_topic_sub = Adaptation()
    change_topic_sub.adaptation_target = Adaptation.ROSPARAMETER
    change_topic_sub.node_name = "minimal_subscriber"
    change_topic_sub.parameter_adaptation = SUB_TOPIC_PARAM

    response = change_topic_cli.send_request([change_topic_sub])

    change_topic_cli.get_logger().info(
        'Changing topic of subscriber to %s success?: %s' %
        (TOPIC_TO_CHANGE_TO,response.success))
    
    change_topic_cli.get_logger().info("The subscriber should now not be receiving anything anymore..")
    time.sleep(10)

    change_topic_pub = copy.deepcopy(change_topic_sub)
    change_topic_pub.node_name = "publisher_B"
    change_topic_pub.parameter_adaptation = PUB_TOPIC_PARAM

    response = change_topic_cli.send_request([change_topic_pub])

    change_topic_cli.get_logger().info(
        'Changing topic of publisher to %s success?: %s' %
        (TOPIC_TO_CHANGE_TO,response.success))
    
    change_topic_cli.get_logger().info("The subscriber should now once again be receiving messages as the publisher matches")
    time.sleep(20)
    


    change_topic_cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
