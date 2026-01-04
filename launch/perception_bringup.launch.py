# launch/bringup_local.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Public args ----
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    show_visualization = DeclareLaunchArgument(
        "show_visualization", default_value="true"
    )

    use_exg = DeclareLaunchArgument("use_exg", default_value="false")

    # ---- Optional static TF: parent -> camera ----
    # Set publish_cam_tf:=true ONLY if your robot does NOT already publish this TF.
    publish_cam_tf = DeclareLaunchArgument("publish_cam_tf", default_value="true")

    cam_parent_frame = DeclareLaunchArgument(
        "cam_parent_frame", default_value="base_link"
    )
    cam_child_frame = DeclareLaunchArgument(
        "cam_child_frame", default_value="cam2_link"
    )

    cam_x = DeclareLaunchArgument("cam_x", default_value="0.55")
    cam_y = DeclareLaunchArgument("cam_y", default_value="0.0")
    cam_z = DeclareLaunchArgument("cam_z", default_value="0.80")
    cam_roll = DeclareLaunchArgument("cam_roll", default_value="1.5708")
    cam_pitch = DeclareLaunchArgument("cam_pitch", default_value="0.0")
    cam_yaw = DeclareLaunchArgument("cam_yaw", default_value="1.5708")

    # ---- RTAB-Map input selection + topics ----
    rgbd_mode = DeclareLaunchArgument(
        "rgbd_mode", default_value="false"
    )  # true => use rgbd_topic
    run_vo = DeclareLaunchArgument("run_vo", default_value="false")

    rgb_topic = DeclareLaunchArgument(
        "rgb_topic", default_value="/cam2/color/image_raw"
    )
    depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/cam2/aligned_depth_to_color/image_raw"
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="/cam2/color/camera_info"
    )
    rgbd_topic = DeclareLaunchArgument(
        "rgbd_topic", default_value="/cam2/rgbd_image_raw"
    )
    gps_topic = DeclareLaunchArgument("gps_topic", default_value="/emlid/fix")
    odom_topic = DeclareLaunchArgument("odom_topic", default_value="/odom")

    pkg = FindPackageShare("futuraps_perception")

    # ---- Optional static TF from parent -> camera ----
    static_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_cam_tf",
        # x y z roll pitch yaw parent child
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_roll"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_parent_frame"),
            LaunchConfiguration("cam_child_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_cam_tf")),
        output="screen",
    )

    # ---- Includes ----
    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "includes", "rtabmap_local.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rgbd_mode": LaunchConfiguration("rgbd_mode"),
            "run_vo": LaunchConfiguration("run_vo"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rgbd_topic": LaunchConfiguration("rgbd_topic"),
            "gps_topic": LaunchConfiguration("gps_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
        }.items(),
    )

    include_point_cloud_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg, "launch", "includes", "point_cloud_filter.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_exg": LaunchConfiguration("use_exg"),
        }.items(),
    )

    include_parameter_extractor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg, "launch", "includes", "parameter_extractor.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    include_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "includes", "visualization.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("show_visualization")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    return LaunchDescription(
        [
            # Public args
            use_sim_time,
            show_visualization,
            use_exg,
            rgbd_mode,
            run_vo,
            rgb_topic,
            depth_topic,
            camera_info_topic,
            rgbd_topic,
            gps_topic,
            odom_topic,
            # Optional TF args
            publish_cam_tf,
            cam_parent_frame,
            cam_child_frame,
            cam_x,
            cam_y,
            cam_z,
            cam_roll,
            cam_pitch,
            cam_yaw,
            # Nodes
            static_cam_tf,
            include_rtabmap,
            include_point_cloud_filter,
            include_parameter_extractor,
            include_visualization,
        ]
    )
