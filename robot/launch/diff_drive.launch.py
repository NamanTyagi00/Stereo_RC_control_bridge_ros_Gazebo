
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node


def generate_launch_description():
#ros2 launch ros_gz_sim ros_gz_sim.launch.py world_sdf_file:=empty.sdf bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> use_composition:=True create_own_container:=True
    pkg_robot = get_package_share_directory('robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_robot, 'models', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

  

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_robot,
            'worlds',
            'tugbot_warehouse.sdf'
        ])}.items()
    )
    gz_topic = '/model/bot'
    joint_state_gz_topic = '/world/world_demo' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
   
    rviz = Node(
       package='rviz2',
       executable='rviz2'
       
      
    )
    controller_node = Node(
        package='controller',
        executable='key_responder'
    )

        # Bridge
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock (Gazebo -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (Gazebo -> ROS2)
                joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Link poses (Gazebo -> ROS2)
                link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Velocity and odometry (Gazebo -> ROS2)
                gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                #camera1and2
                '/world/world_demo/model/bot/link/camera_visual/sensor/camera_left/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/world_demo/model/bot/link/camera_visual/sensor/camera_right/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32'
            ],
            remappings=[
                (joint_state_gz_topic, 'joint_states'),
                (link_pose_gz_topic, '/tf'),
                (link_pose_gz_topic + '_static', '/tf_static'),
            ],
            parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
            output='screen'
        )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        controller_node,
        rviz
    ])