import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    urdf = os.path.join(package_dir, 'rover_joint.urdf')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',),
        
        # Gazebo Related Stuff required to launch the files
        ExecuteProcess(
            cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], # Defines where input would be spawn
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner', #name of this package
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "dolly"]), #robot description is the name of the parameter in the urdf file
    ]) 
         