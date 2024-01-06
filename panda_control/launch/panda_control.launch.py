from launch import LaunchDescription, LaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    '''
    Launches simple_move, vision stuff, and rviz.

    # With the real robot:
    ros2 launch moveBot botchocolate.launch.py real:=true

    # With fake hardware/only rviz on your computer:
    ros2 launch moveBot botchocolate.launch.py real:=false

    '''
    panda_control_path = get_package_share_path('panda_control')
    default_rviz_config_path = panda_control_path / 'urdf.rviz'
    
    real = DeclareLaunchArgument(
        "real",
        default_value="true",
        description="Choose whether to use the real robot"
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )



    # launches the simple_move launchfile
    simple_move = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("movebot"),"/launch/","simple_move.launch.py"]
        )
    )
    

    # starts the trajectory node 
    panda_control = Node(
        package='panda_control',
        executable='panda_control',
        name='panda_control',
    )

    tag_tf = Node(
        package='panda_control',
        executable='tag_tf',
        name='tag_tf',
    )


    # Fake hardware rviz
    rviz_fake = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "franka_moveit_config",
            "moveit.launch.py",
            "robot_ip:=dont-care",
            "use_fake_hardware:=true"
        ],
        condition = LaunchConfigurationEquals("real","false")
    )


    # Rviz when connected to real robot
    rviz_real = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "franka_moveit_config",
            "rviz.launch.py",
            "robot_ip:=panda0.robot",
            # "-d", LaunchConfiguration('rvizconfig')
        ],
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition = LaunchConfigurationEquals("real","true")

    )
    
    # aruco_params = os.path.join(
    #     get_package_share_directory('ros2_aruco'),
    #     'config',
    #     'aruco_parameters.yaml'
    #     )

    # aruco_node = Node(
    #     package='ros2_aruco',
    #     executable='aruco_node',
    #     parameters=[aruco_params],
    #     remappings=[
    #         ('/image_raw', '/camera/color/image_raw'),
    #         ('/camera_info', '/camera/color/camera_info'),
    #     ]
    # )

    dwtag_node = Node(
        package='camera',
        executable='camera',
        name='panda_camera'

        # remappings=[
        #     ('/image_raw', '/camera/color/image_raw'),
        #     ('/camera_info', '/camera/color/camera_info'),
        # ]
    )
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera')),
            '/launch/rs_launch.py']),
            launch_arguments={
                'rgb_camera.profile': '1920x1080x30',
            }.items()
    )


    # Starts the vision launchfile
    # vision = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("bot_vis"),"/launch/","launch_vision.py"]
    #     ),
    #     condition = LaunchConfigurationEquals("real","true")
    # )

    realsense_frame_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', '0.048', '--y', '-0.02', '--z', '0.065', '--yaw', '0', '--pitch', '-1.570796327', '--roll', '3.141592654', '--frame-id', 'panda_hand', '--child-frame-id', 'camera_link']
            )
    
    track_gen_node = Node(
                package='track_manager',
                executable='track_gen',
                name='track_gen'
            )


    ld = LaunchDescription([
        realsense_node,
        rviz_arg,
        real,
        dwtag_node,
        simple_move,
        # vision,
        tag_tf,
        rviz_real,
        rviz_fake,
        panda_control,
        realsense_frame_node,
        track_gen_node
    ])

    return ld
