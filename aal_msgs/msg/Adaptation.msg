#Targest of adaptations supported.
int8 STATETRANSITION=0
int8 ROSPARAMETER=1
int8 CONNECTION=2

int8 adaptation_target

lifecycle_msgs/Transition lifecycle_adaptation
rcl_interfaces/Parameter parameter_adaptation
rcl_interfaces/Parameter connection_adaptation

#In the case of parameter and lifecycle adaptation, the ros2 node they are to be applied to.
string node_name
