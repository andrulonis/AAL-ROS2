#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aal_msgs.srv import AdaptArchitectureExternal, AdaptArchitecture
from rcl_interfaces.msg import Parameter
from aal_msgs.msg import AdaptationState, Configuration, Adaptation
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from aal.adaptation_strategies import create_strategy
from lifecycle_msgs.srv import ChangeState
from itertools import product


def value_from_param(param_msg):
    param_type = param_msg.value.type

    if(param_type == 1): return param_msg.value.bool_value
    if(param_type == 2): return param_msg.value.integer_value
    if(param_type == 3): return param_msg.value.double_value
    if(param_type == 4): return param_msg.value.string_value
    if(param_type == 5): return param_msg.value.byte_array_value
    if(param_type == 6): return param_msg.value.bool_array_value
    if(param_type == 7): return param_msg.value.integer_array_value
    if(param_type == 8): return param_msg.value.double_array_value
    if(param_type == 9): return param_msg.value.string_array_value

class AdaptationManager(Node):
    

    EXT_SERV_NAME = '/adapt_architecture_external'
    INT_SERV_NAME = '/adapt_architecture'

    def __init__(self): 
        super().__init__('adaptation_manager')

        self.task_to_strategy_map = {}

        self.srv_ext_adapt = self.create_service(AdaptArchitectureExternal,self.EXT_SERV_NAME,self.ext_adaptation_requested)
        self.srv_adapt = self.create_service(AdaptArchitecture, self.INT_SERV_NAME,self.adaptation_requested)

        self.reporting = [0,0]

        self.bounds_dict = {}
        self.set_parameter_client_dict = {}
        self.change_state_client_dict = {}
        self.get_state_client_dict = {}
        self.reporting_dict = {}
        self.get_logger().info("AAL initialized, now providing %s and %s services" % (self.INT_SERV_NAME, self.EXT_SERV_NAME))
        

    def make_configurations(self, adaptation_options_list):
        param_to_node = {}
        param_to_target = {}

        
        possible_configurations = []

        possible_configurations = []
        list_of_list_param = []
        for adaptation_options in adaptation_options_list:
            #AdaptationOptions.msg:
            # #name of the parameter
            # string name
            # #name of the node (if any) it belongs to
            # string node_name
            # #The type of adaptation being done to the target, this is constrained choice specified in Adaptation.msg
            # int8 adaptation_target_type
            # #A presumed finite set of acceptable values for the parameter to hold.
            # rcl_interfaces/ParameterValue[] possible_values 

            decomposed = []

            for pos_val in adaptation_options.possible_values:
                param = Parameter()
                param.name = adaptation_options.name
                param.value = pos_val
                param_to_node[str((param.name, param.value))] = adaptation_options.node_name
                param_to_target[str((param.name, param.value))] = adaptation_options.adaptation_target_type
                decomposed.append(param)
            list_of_list_param.append(decomposed)
        
        self.get_logger().info('list of list param ' + str(list_of_list_param))
        possible_configurations = list(product(*list_of_list_param))
        #here's where you'd apply constraints to remove invalid configurations
        print(len(possible_configurations))
        print(possible_configurations)
        config_list = []
        for possible_config in possible_configurations:
            if(len(possible_config) != 0):
                possible_config_list = list(possible_config)
                config_msg = Configuration()
                config_msg.node_names = [param_to_node[str((param.name, param.value))] for param in possible_config_list]
                config_msg.adaptation_target_types = [param_to_target[str((param.name, param.value))] for param in possible_config_list]
                config_msg.configuration_parameters = possible_config
                config_list.append(config_msg)
        #The arms should consists of a list of Parameter, name value pairs of each parameter given.
        return config_list


    def create_set_param_client(self, node_name):
        self.set_parameter_client_dict[node_name] = self.create_client(SetParameters, '/' + node_name + '/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())

    def create_change_state_client(self, node_name):
        self.change_state_client_dict[node_name] = self.create_client(ChangeState, '/' + node_name + '/change_state', callback_group=MutuallyExclusiveCallbackGroup())


    def execute_rp_adaptation(self, param_msg, node_name):
        if((param_msg is None) or (node_name is None)):
                self.get_logger().error("Unknown or unspecified type of adaptation")
                response.success = False
                return response
        
        if node_name not in self.set_parameter_client_dict: 
            self.create_set_param_client(node_name)  

        client = self.set_parameter_client_dict[node_name]
        

        if type(param_msg) is not list:
            param_msg = [param_msg]

        for par in param_msg:
            self.reporting_dict[par.name] = value_from_param(par)

        req_rp_exec = SetParameters.Request()

        req_rp_exec.parameters = param_msg

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_param service not available, waiting again...')
            
        response = client.call(req_rp_exec)
        
        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Adaptation Manager, see reason(s):' + str(response.results))
            return False
        
        self.get_logger().info('ros param adaptation complete.')
        return True
    
    def execute_lc_adaptation(self, transition, node_name):
        self.get_logger().info("\n\n\ LC adaptation \n\n\n\n\n")

        if((transition is None) or (node_name is None)):
                self.get_logger().error("Unknown or unspecified type of adaptation")
                response.success = False
                return response

        if node_name not in self.change_state_client_dict: 
            self.create_change_state_client(node_name)

        client = self.change_state_client_dict[node_name]


        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('change_state service not available, waiting again...')
        
        self.get_logger().info("\n\n\Sending LC adaptation request\n\n\n\n\n")


        req_lc_exec = ChangeState.Request()
        req_lc_exec.transition = transition

        response = client.call(req_lc_exec)

        if(not response.success):
            self.get_logger().warning('A request to change a state was unsuccessful in the Adaptation Manager')
        

        self.get_logger().info("\n\n\nFinishing LC adaptation\n\n\n\n\n")

        return response.success
            

    def execute_adaptation(self, adaptation):
        target_of_adaptation =  adaptation.adaptation_target
        if(target_of_adaptation == Adaptation.STATETRANSITION):
            self.get_logger().info('\n\n ros_lc adaptation \n\n')
            return self.execute_lc_adaptation(adaptation.lifecycle_adaptation,adaptation.node_name)                
        elif(target_of_adaptation == Adaptation.ROSPARAMETER):
            self.get_logger().info('\n\n ros_param adaptation \n\n')
            return self.execute_rp_adaptation(adaptation.parameter_adaptation,adaptation.node_name)
        elif(target_of_adaptation == Adaptation.CONNECTION):
            self.get_logger().info('\n\n ros_param adaptation \n\n')
            return self.execute_rp_adaptation(adaptation.connection_adaptation,adaptation.node_name)

    def adaptation_requested(self, request, response):
        # rebet_msgs/Adaptation[] adaptation
        # ---
        # bool success
        self.get_logger().info('\n\n offline adaptation \n\n')
        
        self.get_logger().info(str(len(request.adaptations)))

        self.get_logger().info(str(request.adaptations))

        for adaptation_to_execute in request.adaptations:
            adaptation_results = []

            is_exec_success = self.execute_adaptation(adaptation_to_execute)
            # if(target_of_adaptation is None): 
            #     self.get_logger().error("Unknown or unspecified type of adaptation")
            #     response.success = False
            #     return response


            # if(target_of_adaptation == Adaptation.STATETRANSITION):
            #     self.get_logger().info('\n\n ros_lc adaptation \n\n')
            #     is_exec_success = self.execute_lc_adaptation(adaptation_to_execute.lifecycle_adaptation,adaptation_to_execute.node_name)                
            # elif(target_of_adaptation == Adaptation.ROSPARAMETER):
            #     self.get_logger().info('\n\n ros_param adaptation \n\n')
            #     is_exec_success = self.execute_rp_adaptation(adaptation_to_execute.parameter_adaptation,adaptation_to_execute.node_name)

            adaptation_results.append(is_exec_success)


        response.success = all(adaptation_results)
        return response 
       
    
    def ext_adaptation_requested(self, request, response):
        # rebet_msgs/AdaptationOptions[] adaptation_space
        # string task_identifier
        # string adaptation_strategy
        # ---
        # bool success

        task_identifier = str(request.task_identifier)
        adaptation_strategy = str(request.adaptation_strategy)
        utilities = request.utility_previous
        response.success = False
        adapt_state = AdaptationState()
        self.get_logger().info(str(request))
        adaptation_at_system_level = "system" in task_identifier #TODO: make this more robust.
            
        if(list(utilities) == []):
            self.get_logger().info("assuming  this is the first time, filling utility with dummy value")
            utilities = [0.0]

        adapt_state.current_utility = utilities

        if(list(request.qrs) == []):
            self.get_logger().info("assuming  this is the first time, filling qrs with dummy value")
            adapt_state.qrs = []
        else:
            adapt_state.qrs = request.qrs

        self.get_logger().info('\n\n sys util \n\n' + str(utilities))

        adapt_state.possible_configurations = self.make_configurations(request.adaptation_space)

  
        #new task or new strategy for the same task.

        if ( (task_identifier not in self.task_to_strategy_map) or ( (task_identifier in self.task_to_strategy_map) and (adaptation_strategy != self.task_to_strategy_map[task_identifier].get_name()) ) ):
            self.task_to_strategy_map[task_identifier] = create_strategy(adaptation_strategy)
        elif(request.adaptation_strategy == "reset"):
            #reset of same strategy during task.
            self.get_logger().info('\n\n RESET OF STRATEGY \n\n')

            self.task_to_strategy_map[task_identifier] = create_strategy(self.task_to_strategy_map[task_identifier].get_name())
        else:
            self.get_logger().info('\n\n REUSE OF STRATEGY FOR TASK:' + task_identifier+'\n\n')


        self.get_logger().info('\n\n REUSE OF STRATEGY FOR TASK:' + str(adapt_state)+'\n\n')
        
        suggested_configuration = self.task_to_strategy_map[task_identifier].suggest_adaptation(adapt_state)
        #Configuration.msg
        #string[] node_names
        #int8[] adaptation_target_types
        #rcl_interfaces/Parameter[] configuration_parameters
        response.applied_adaptations = []

        for i in range(len(suggested_configuration.node_names)):
            node_name = suggested_configuration.node_names[i]
            type_of_adaptation = suggested_configuration.adaptation_target_types[i]

            adap = Adaptation()
            adap.adaptation_target = type_of_adaptation
            adap.node_name = node_name

            if(type_of_adaptation == Adaptation.ROSPARAMETER):
                adap.parameter_adaptation = suggested_configuration.configuration_parameters[i]
            elif(type_of_adaptation == Adaptation.STATETRANSITION):
                adap.lifecycle_adaptation = suggested_configuration.configuration_transitions[i]
            
            adaptation_responses = []
            is_exec_success = self.execute_adaptation(adap)

        
            adaptation_responses.append(is_exec_success)
            if(is_exec_success):
                response.applied_adaptations.append(adap)

        response.success = all(adaptation_responses)


        return response


def main(args=None):
    rclpy.init()

    adapt_manage_node = AdaptationManager()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(adapt_manage_node)
    
    mt_executor.spin()
   

    
    
    adapt_manage_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()