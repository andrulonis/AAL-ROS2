from aal_msgs.srv import RuntimeLaunch

import rclpy
from rclpy.node import Node
from launch import LaunchDescription 
from launch import LaunchIntrospector 
from launch import LaunchService

import launch_ros.actions  

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(RuntimeLaunch, 'runtime_launch', self.launch_callback)

    def launch_callback(self, request, response):
        ld = LaunchDescription([
        launch_ros.actions.Node(
            package='aal', executable='demo_subscriber'),])
    
	    # ls = LaunchService(debug=True)
        ls = LaunchService()
        ls.include_launch_description(ld)
        print("about to run")

        ret = ls.run()
        print("after run")



        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
