from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_exg = LaunchConfiguration("use_exg")

    pkg = FindPackageShare("futuraps_perception")

    crop_box_filter_cfg = PathJoinSubstitution(
        [pkg, "config", "pointcloud_filters", "crop_box_filter.yaml"]
    )
    local_filter_cfg = PathJoinSubstitution(
        [pkg, "config", "pointcloud_filters", "local_map_filter.yaml"]
    )

    nodes = [
        # ------- Crop Box -------
        Node(
            package="futuraps_perception",
            executable="crop_box_filter_node",
            name="crop_box_filter",
            output="screen",
            parameters=[
                crop_box_filter_cfg,
                {
                    "output_frame": "base_link",
                    "publish_frame": "map",
                    "leaf_size": 0.03,
                    "use_latest_tf": True,
                    "tf_timeout_sec": 0.2,
                    "publish_marker": True,
                    "marker_alpha": 0.15,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=[
                ("input", "/octomap_occupied_space"),
                ("output", "/octomap_cloud/local_vox"),
            ],
        ),
        # ------- Local Map Filter -------
        Node(
            package="futuraps_perception",
            executable="local_map_filter_node",
            name="local_map_filter",
            output="screen",
            parameters=[
                local_filter_cfg,
                {"use_sim_time": use_sim_time, "use_exg": use_exg},
            ],
        ),
    ]

    return LaunchDescription(
        [
            *nodes,
        ]
    )
