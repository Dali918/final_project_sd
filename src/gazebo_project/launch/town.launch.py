import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_description = "gazebo_project"
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim
    install_dir_path = get_package_prefix(package_description) + "/share"
    model_path = os.path.join(package_directory, "model")
    mesh_path = os.path.join(package_directory, "meshes")

    def get_model_paths():
        paths = []
        for model_dir in os.listdir(model_path):
            model_full_path = os.path.join(model_path, model_dir)
            if os.path.isdir(model_full_path):
                paths.append(model_full_path)
                # Add materials/textures and materials/scripts path if it exists
                textures_path = os.path.join(model_full_path, "materials", "textures")
                if os.path.exists(textures_path):
                    paths.append(textures_path)
                scripts_path = os.path.join(model_full_path, "materials", "scripts")
                if os.path.exists(scripts_path):
                    paths.append(scripts_path)
                # Add meshes path
                meshes_path = os.path.join(model_full_path, "meshes")
                if os.path.exists(meshes_path):
                    paths.append(meshes_path)
                # Add thumbnails path
                thumbnails_path = os.path.join(model_full_path, "thumbnails")
                if os.path.exists(thumbnails_path):
                    paths.append(thumbnails_path)
                
        return paths

    gazebo_resource_paths = [
        install_dir_path, 
        model_path,
        mesh_path,
    ] + get_model_paths()

    fuel_servers = [
        "https://fuel.gazebosim.org",
        "https://fuel.ignitionrobotics.org"  # Legacy URL for some models
    ]

    if "IGN_FUEL_URL" in os.environ:
        os.environ["IGN_FUEL_URL"] += ":" + ":".join(fuel_servers)
    else:
        os.environ["IGN_FUEL_URL"] = ":".join(fuel_servers)

        
    
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += ":" + resource_path
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)

    # Load Demo World SDF from Robot Description Package
    world = "town_world"

    world_file = f"{world}.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file_path], description="SDF World File"
    )

    # Declare Gazebo Sim Launch file
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": world_config}.items(),
    )

    # Load the urdf
    urdf_file = "robot.urdf"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro ", robot_desc_path]),
            }
        ],
    )

    # declare robot spawn position
    declare_spawn_x = DeclareLaunchArgument(
        "x", default_value="14.5075", description="Model Spawn X Axis Value"
    )
    declare_spawn_y = DeclareLaunchArgument(
        "y", default_value="59.814", description="Model Spawn Y Axis Value"
    )
    declare_spawn_z = DeclareLaunchArgument(
        "z", default_value="1.0", description="Model Spawn Z Axis Value"
    )
    declare_spawn_R = DeclareLaunchArgument(
        "R", default_value="0.0", description="Model Spawn Roll Value"
    )
    declare_spawn_P = DeclareLaunchArgument(
        "P", default_value="0.0", description="Model Spawn Pitch Value"
    )
    declare_spawn_Y = DeclareLaunchArgument(
        "Y", default_value="-1.5708", description="Model Spawn Yaw Value"
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name",
            "my_robot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            f"/world/{world}/model/my_robot/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            # "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            f"/world/{world}/pose/info"
            + "@geometry_msgs/msg/PoseArray"
            + "[ignition.msgs.Pose_V",
        ],
        remappings=[
            (f"/world/{world}/model/my_robot/joint_state", "/joint_states"),
            (f"/world/{world}/pose/info", "/pose_info"),
        ],
        output="screen",
    )

    cam_tf_node = Node(
        name="camera_stf",
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "1.5707",
            "-1.5707",
            "0",
            "oakd_rgb_camera_optical_frame",
            "/my_robot/oakd_rgb_camera_frame/rgbd_camera",
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    robot_name = "my_robot"

    oakd_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points"
                + "@sensor_msgs/msg/PointCloud2"
                + "[ignition.msgs.PointCloudPacked",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info"
                + "@sensor_msgs/msg/CameraInfo"
                + "[ignition.msgs.CameraInfo",
            ],
        ],
        remappings=[
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image",
                ],
                "oakd/rgb/image_raw",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image",
                ],
                "oakd/rgb/depth",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points",
                ],
                "oakd/rgb/depth/points",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info",
                ],
                "oakd/rgb/camera_info",
            ),
        ],
    )

    odometry_tf = Node(
        package="gazebo_project",
        executable="odometry_tf",
        name="odometry_tf",
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            declare_world_arg,
            gz_sim,
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            declare_spawn_R,
            declare_spawn_P,
            declare_spawn_Y,
            gz_spawn_entity,
            ign_bridge,
            cam_tf_node,
            oakd_camera_bridge,
            odometry_tf,
        ]
    )
