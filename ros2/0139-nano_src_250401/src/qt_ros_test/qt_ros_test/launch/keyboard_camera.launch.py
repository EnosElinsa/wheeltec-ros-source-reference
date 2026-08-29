from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():
  return LaunchDescription([

        launch_ros.actions.Node(
            package='qt_ros_test', 
            executable='qt_ros_test', 
            parameters=[{'user_passward': 'dongguan'},		#虚拟机root密码
            		{'rgbtopic': '/camera/color/image_raw/compressed'},
            		{'depthtopic': '/camera/depth/image_raw'}],   
            output='screen')
  ])
