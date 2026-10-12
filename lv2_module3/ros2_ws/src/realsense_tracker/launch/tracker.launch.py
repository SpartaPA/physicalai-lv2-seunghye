"""모듈 3 개인 과제 — RealSense 및 인지 노드 실행.

기존 EYE4 tracker.launch.py의 비전 실행 구조를 재사용한다.

실행 구성:
    RealSense
        → realsense2_camera (Raw Image 발행)
        → perception_node
            ├── /perception/mask
            ├── /target
            └── /perception/debug_image (설정 시)

설정 파일:
    config/camera.yaml  : 카메라 해상도, FPS, 토픽 이름
    config/tracker.yaml : HSV, Contour 면적, 검출 설정

Launch 인자:
    start_realsense : Camera 입력 실행 여부
    camera_config   : 카메라 설정 경로
    tracker_params  : 인지 설정 경로

이번 개인 과제에서는 Control, OpenCR, Dynamixel을
실행하지 않는다.

참고:
    Raw Image는 realsense2_camera가 발행한다.
    Mask는 perception_node가 /perception/mask로 발행한다.
    검출 결과는 /target으로 발행한다.
    Overlay는 publish_debug_image=true일 때 발행한다.
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def realsense(context):
    """camera.yaml의 width/height/fps로 wrapper 프로파일을 만든다 (해상도 값의 Source of Truth는 camera.yaml 하나)."""
    path = LaunchConfiguration("camera_config").perform(context)
    with open(path, "r", encoding="utf-8") as f:
        cam = yaml.safe_load(f) or {}
    profile = f"{int(cam['width'])},{int(cam['height'])},{int(cam['fps'])}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={
            # 컬러 영상은 이번 과제의 Raw Image 입력이므로 사용
            "enable_color": "true",
            # HSV·Contour 비교에 Depth는 필요하지 않으므로 비활성화
            "enable_depth": "false",
            # Depth를 컬러 영상에 정렬하는 후처리 기능도 비활성화
            "align_depth.enable": "false",
            # camera.yaml의 width, height, fps 적용
            "rgb_camera.color_profile": profile,
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_realsense")),
    )]


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare("realsense_tracker"), "config"])

    perception = Node(
        package="realsense_tracker",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[LaunchConfiguration("tracker_params"),
                    {"camera_config": LaunchConfiguration("camera_config")}],
    )
   
    return LaunchDescription([
        DeclareLaunchArgument("camera_config",
                              default_value=PathJoinSubstitution([config_dir, "camera.yaml"]),
                              description="Camera profile and wrapper topic names (plain YAML)."),
        DeclareLaunchArgument("tracker_params",
                              default_value=PathJoinSubstitution([config_dir, "tracker.yaml"]),
                              description="perception_node parameter file."),
        DeclareLaunchArgument("start_realsense", default_value="true",
                              description="Start the official realsense2_camera wrapper."),
        OpaqueFunction(function=realsense),
        perception,

    ])
