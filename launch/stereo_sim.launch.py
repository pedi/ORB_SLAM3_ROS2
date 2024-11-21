from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'),
                'vocabulary',  # Assuming your vocab file is in the vocabulary directory
                'ORBvoc.txt'   # Replace with your actual vocabulary file name
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            default_value='sm2_stereosim.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            output='screen',
            namespace='orbslam3',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
                    'stereo',
                    LaunchConfiguration('yaml_file')
                ]),
                'False'
            ],
            remappings=[
                ('camera/left', '/stereo_left'),
                ('camera/right', '/stereo_right')
            ]
        ),
        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--yaw', '-1.570796327',
        #          '--roll', '-1.5707963270',
        #          '--pitch', '0',
        #          '--frame-id', 'orbslam3',
        #          '--child-frame-id', 'left_camera'],
        #     output='screen',
        # ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                    # '--yaw', '-1.570796327',
                    # '--roll', '-1.5707963270',
                    # '--pitch', '0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'orbslam3'],
            output='screen',
        ),


    
        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--yaw', '-1.570796327',
        #          '--roll', '-1.5707963270',
        #          '--pitch', '0',
        #          '--frame-id', 'map',
        #          '--child-frame-id', 'down'],
        #     output='screen',
        # )
    ])
