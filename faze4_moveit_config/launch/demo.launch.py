import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command


def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware, without serial device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_device",
            default_value="/dev/ttyUSB0",
            description="Serial device name.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="115200",
            description="Serial device baudrate.",
        )
    )

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    serial_device = LaunchConfiguration("serial_device")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    # Find and parse URDF xacro
    robot_description_content = Command(
        [
            "xacro",
            " ",
            os.path.join(
                get_package_share_directory("faze4_description"), "urdf", "faze4.urdf.xacro"
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "serial_device:=",
            serial_device,
            " ",
            "serial_baudrate:=",
            serial_baudrate,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Find and parse SRDF xacro
    srdf_xacro_path = os.path.join(
        get_package_share_directory("faze4_moveit_config"), "config", "faze4_arm.srdf.xacro"
    )

    robot_description_semantic = {
        "robot_description_semantic": Command(["xacro", " ", srdf_xacro_path])
    }

    kinematics_yaml = load_yaml("faze4_moveit_config", "config", "kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("faze4_moveit_config", "config", "ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(
            "faze4_moveit_config", "config", "controllers.yaml"
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("faze4_moveit_config"), "rviz", "demo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "faze4_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory("faze4_moveit_config"),
                "config",
                "controller_manager.yaml",
            ),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # return LaunchDescription(declared_arguments + [rviz_node, static_tf, robot_state_publisher, run_move_group_node])

    return LaunchDescription(
        declared_arguments
        + [
            rviz_node,
            static_tf,
            ros2_control_node,
            robot_state_publisher,
            run_move_group_node,
            joint_controller_spawner_started,
            joint_state_broadcaster_spawner_started,
        ]
    )
