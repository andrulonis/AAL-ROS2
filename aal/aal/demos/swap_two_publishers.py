import sys

from aal_msgs.srv import OfflineAdaptation
from aal_msgs.msg import Adaptation
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node
import copy
import time

ACTIVATE_TRANSITION = Transition()
ACTIVATE_TRANSITION.id = Transition.TRANSITION_ACTIVATE

DEACTIVATE_TRANSITION = Transition()
DEACTIVATE_TRANSITION.id = Transition.TRANSITION_DEACTIVATE

class SwapPublishersClient(Node):

    def __init__(self):
        super().__init__('swap_publishers_client')
        self.cli = self.create_client(OfflineAdaptation, '/offline_adaptation')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OfflineAdaptation.Request()

        

    def send_request(self, adaptations):
        self.req.adaptations = adaptations
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    swap_pub_cli = SwapPublishersClient()

    activate_publisherA = Adaptation()
    activate_publisherA.adaptation_target = Adaptation.STATETRANSITION
    activate_publisherA.node_name = "publisher_A"
    activate_publisherA.lifecycle_adaptation = ACTIVATE_TRANSITION

    
    response = swap_pub_cli.send_request([activate_publisherA])

    swap_pub_cli.get_logger().info(
        'Activating publisher_A success?: %s' %
        (response.success))

    time.sleep(10)

    deactivate_publisherA = copy.deepcopy(activate_publisherA)
    deactivate_publisherA.lifecycle_adaptation = DEACTIVATE_TRANSITION

    activate_publisherB = copy.deepcopy(activate_publisherA)
    activate_publisherB.node_name = "publisher_B"
    print(activate_publisherB.lifecycle_adaptation)

    response = swap_pub_cli.send_request([deactivate_publisherA,activate_publisherB])

    swap_pub_cli.get_logger().info(
        'Deactivating publisher_A and activating publisher B success?: %s' %
        (response.success))
    
    time.sleep(10)

    deactivate_publisherB = copy.deepcopy(deactivate_publisherA)
    deactivate_publisherB.node_name = "publisher_B"

    response = swap_pub_cli.send_request([deactivate_publisherB,activate_publisherA])

    swap_pub_cli.get_logger().info(
        'Deactivating publisher_B and activating publisher A success?: %s' %
        (response.success))
    


    swap_pub_cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
