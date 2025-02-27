import sys

from aal_msgs.srv import AdaptArchitectureExternal
from aal_msgs.msg import Adaptation, AdaptationOptions
from rcl_interfaces.msg import Parameter, ParameterValue
import rclpy
from rclpy.node import Node
import time
import numpy as np


class ChangeMaxVelocity(Node):

    def __init__(self):
        super().__init__('swap_publishers_client')
        self.cli = self.create_client(AdaptArchitectureExternal, '/adapt_architecture_external')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AdaptArchitectureExternal.Request()

        var_params = []
        variable_param = AdaptationOptions();


        variable_param.name = "max_velocity"
        variable_param.node_name = "velocity_smoother"
        variable_param.adaptation_target_type = Adaptation.ROSPARAMETER
        default_y_velocity = 0.0; #The robot in question can not move along its y axis independently.
        default_theta_velocity = 2.5; #We are not interested in modifying the rotation speed here, but could be extended to do so.
        for val in [0.10, 0.18, 0.26]:
            speed_vector = [val,default_y_velocity,default_theta_velocity]
            par_val = rclpy.parameter.Parameter('speed', value=speed_vector).get_parameter_value() #Here we wrap it with default values
            variable_param.possible_values.append(par_val)
        
        var_params.append(variable_param) #vector of AdaptationOptions   

        self.req.adaptation_space = var_params


        

    def send_request(self, strategy_name):
        self.req.utility_previous = [np.random.random()]
        self.req.adaptation_strategy = strategy_name
        self.req.task_identifier = "test_task"

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    swap_pub_cli = ChangeMaxVelocity()

    
    response = swap_pub_cli.send_request("ucb")

    swap_pub_cli.get_logger().info(
        'Activating publisher_A success?: %s' %
        (response.success))

    time.sleep(10)


    swap_pub_cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
