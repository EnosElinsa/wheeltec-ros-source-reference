from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from wheeltec_vision_sim.launching import build

def generate_launch_description():
    def compose(context):
        def val(k):return LaunchConfiguration(k).perform(context)
        return build(val('mode'),val('estimator'),val('database_path'),val('output'),
            val('headless')=='true',val('supervised_reverse')=='true',val('geometry'))
    return LaunchDescription([
        DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization']),
        DeclareLaunchArgument('estimator',default_value='baseline',choices=['baseline','stereo','vio']),
        DeclareLaunchArgument('database_path',default_value='/tmp/wheeltec_sim.db'),
        DeclareLaunchArgument('output'),DeclareLaunchArgument('geometry',default_value=''),
        DeclareLaunchArgument('headless',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('supervised_reverse',default_value='false',choices=['true','false']),OpaqueFunction(function=compose)])
