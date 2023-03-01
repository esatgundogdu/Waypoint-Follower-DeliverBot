import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    worldFileName = 'NMaze-1.world'
    worldPath = os.path.join("~/ros2_ws/src/a2/",'worlds', worldFileName)
    map_file = "my_slam_map_newMaze_alt1.yaml"
    rviz_config_dir =  "~/ros2_ws/src/a2/config/nav.rviz"
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    executeProcessGazebo = ExecuteProcess(
        cmd=[['gazebo'], [worldPath]],
        shell = True,
    )
    executeProcessRViz = ExecuteProcess(
        cmd=[['rviz2'], [" -d"],[ rviz_config_dir], [" --ros-args --remap use_sim_time:=true"]],
        shell = True,
    )
    nav2 = Node(package='nav2_map_server',
            executable='map_server',
            name='map_server',
            
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file},
                        {'topic_name':'map'},
                        {'frame_id':'map'}, 
                       ])
    amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'initial_pose':{'x':5.5,'y':-0.5,'z':0.0,'yaw':3.15}},
            {'set_initial_pose': True},
            {'first_map_only': True}]
        )
    lifecycle = Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['amcl','map_server']}])
    
    robotStatePublisherLD = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    executeReferee = ExecuteProcess(
        cmd=[['ros2',' run ','a2 ','a2_referee']],
        shell=True)
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_argument)
    
    ld.add_action(executeProcessGazebo)
    ld.add_action(robotStatePublisherLD)
    ld.add_action(amcl)
    ld.add_action(nav2)
    
    ld.add_action(executeProcessRViz)
    ld.add_action(lifecycle)
    ld.add_action(executeReferee)

    return ld