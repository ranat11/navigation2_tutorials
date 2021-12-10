from ament_index_python.packages import get_package_share_directory


import launch
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
import launch_ros
import os

def generate_launch_description():
    # For display launch
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav_config.rviz')
    default_world_path = os.path.join(pkg_share, 'world/nav_world.sdf')
    
    # For navigation
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Get the launch directory
    bringup_dir = get_package_share_directory('sam_bot_description')
    nav_dir = get_package_share_directory('nav2_bringup')

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        # AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items()), #TODO add maps

        # Navigation
        #TODO can not add with launch file directly has to lauch seperately
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'navigation_launch.py')),
        #     launch_arguments={'namespace': namespace,
        #                     #   'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_lifecycle_mgr': 'false',
        #                       'map_subscribe_transient_local': 'true'}.items()),
        
        # ros2 launch nav2_bringup navigation_launch.py params_file:=dev_ws/src/navigation2_tutorials/sam_bot_description/config/nav2_params.yaml 
        # use_sim_time:=True #TODO can not use_sim_time
    ])

    # Voxel for dectied objects
    marker_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_markers',
        output='screen',
        parameters=[{'voxel_grid': '/local_costmap/voxel_grid',
                    'visualization_marker': '/my_marker',
                    'use_sim_time': use_sim_time,
                    }]
        )


    return launch.LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='world', default_value=default_world_path,
                                            description='Absolute path to robot world file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        DeclareLaunchArgument('use_namespace', default_value='false',
                                            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument('namespace', default_value='', 
                                            description='Top-level namespace'),
        DeclareLaunchArgument('slam', default_value='True', 
                                            description='Whether run a SLAM'),
        DeclareLaunchArgument('map', default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
                                            description='Full path to map yaml file to load'), 
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'), 
                                            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument('autostart', default_value='true', 
                                            description='Automatically startup the nav2 stack'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("sam_bot_description"), "launch", "display.launch.py")]),
            launch_arguments=[('gui', LaunchConfiguration('gui')),
                ('model', LaunchConfiguration('model')),
                ('rvizconfig', LaunchConfiguration('rvizconfig')),
                ('use_sim_time', LaunchConfiguration('use_sim_time'))
                ]
        ),

        bringup_cmd_group,
        marker_node
    ])
