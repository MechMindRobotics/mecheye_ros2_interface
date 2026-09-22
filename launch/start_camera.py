from math import atan2, cos, radians, sin, sqrt
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CALIBRATION_FILE = (
    Path(__file__).resolve().parents[1] / "config" / "mecheye_calibration_2026_04_07.yaml"
)


def load_calibration(calibration_file):
    calibration_path = Path(calibration_file).expanduser()
    if not calibration_path.exists():
        raise FileNotFoundError(f"Calibration file '{calibration_path}' does not exist.")

    with calibration_path.open("r", encoding="utf-8") as handle:
        calibration = yaml.safe_load(handle) or {}

    camera_frame = calibration.get("camera_frame", "mechmind_camera_link")
    camera_optical_frame = calibration.get("camera_optical_frame", "mechmind_camera_optical_frame")
    external_calibration = calibration.get("external_calibration", {})
    camera_optical_transform = calibration.get("camera_optical_transform", {})

    external_frame_id = external_calibration.get("frame_id", "tool_base")
    external_child_frame_id = external_calibration.get("child_frame_id", camera_frame)
    if camera_frame not in {external_frame_id, external_child_frame_id}:
        raise ValueError(
            "The external calibration must reference 'camera_frame' as either frame_id or "
            "child_frame_id."
        )

    return {
        "camera_frame": camera_frame,
        "camera_optical_frame": camera_optical_frame,
        "external_calibration": _parse_transform(
            external_calibration,
            default_frame_id=external_frame_id,
            default_child_frame_id=external_child_frame_id,
        ),
        "camera_optical_transform": _parse_transform(
            camera_optical_transform,
            default_frame_id=camera_frame,
            default_child_frame_id=camera_optical_frame,
            default_roll=-1.57079632679,
            default_pitch=0.0,
            default_yaw=-1.57079632679,
        ),
    }


def transform_arguments(transform, frame_id, child_frame_id):
    args = [
        "--x",
        str(transform["x"]),
        "--y",
        str(transform["y"]),
        "--z",
        str(transform["z"]),
    ]
    if "qw" in transform:
        args += [
            "--qx",
            str(transform["qx"]),
            "--qy",
            str(transform["qy"]),
            "--qz",
            str(transform["qz"]),
            "--qw",
            str(transform["qw"]),
        ]
    else:
        args += [
            "--roll",
            str(transform["roll"]),
            "--pitch",
            str(transform["pitch"]),
            "--yaw",
            str(transform["yaw"]),
        ]
    args += [
        "--frame-id",
        frame_id,
        "--child-frame-id",
        child_frame_id,
    ]
    return args


def _parse_transform(
    transform_config,
    *,
    default_frame_id,
    default_child_frame_id,
    default_roll=0.0,
    default_pitch=0.0,
    default_yaw=0.0,
):
    translation = transform_config.get("translation", {})
    x = translation.get("x", transform_config.get("x", 0.0))
    y = translation.get("y", transform_config.get("y", 0.0))
    z = translation.get("z", transform_config.get("z", 0.0))

    rotation = transform_config.get("rotation", transform_config.get("rotation_quaternion"))
    if rotation and {"w", "x", "y", "z"} <= rotation.keys():
        return {
            "frame_id": transform_config.get("frame_id", default_frame_id),
            "child_frame_id": transform_config.get("child_frame_id", default_child_frame_id),
            "x": x,
            "y": y,
            "z": z,
            "qx": rotation["x"],
            "qy": rotation["y"],
            "qz": rotation["z"],
            "qw": rotation["w"],
        }

    if {"roll", "pitch", "yaw"} <= transform_config.keys():
        roll = transform_config["roll"]
        pitch = transform_config["pitch"]
        yaw = transform_config["yaw"]
    else:
        roll, pitch, yaw = _parse_axis_angle_rotation(
            transform_config,
            default_roll=default_roll,
            default_pitch=default_pitch,
            default_yaw=default_yaw,
        )

    return {
        "frame_id": transform_config.get("frame_id", default_frame_id),
        "child_frame_id": transform_config.get("child_frame_id", default_child_frame_id),
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def _parse_axis_angle_rotation(transform_config, *, default_roll, default_pitch, default_yaw):
    axis = transform_config.get("rotation_axis")
    if not axis:
        return default_roll, default_pitch, default_yaw

    axis_x = axis.get("x", 0.0)
    axis_y = axis.get("y", 0.0)
    axis_z = axis.get("z", 0.0)
    axis_norm = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if axis_norm == 0.0:
        raise ValueError("rotation_axis must be non-zero.")

    axis_x /= axis_norm
    axis_y /= axis_norm
    axis_z /= axis_norm

    if "rotation_angle_deg" in transform_config:
        angle = radians(transform_config["rotation_angle_deg"])
    else:
        angle = transform_config.get("rotation_angle_rad", 0.0)

    return _axis_angle_to_rpy(axis_x, axis_y, axis_z, angle)


def _axis_angle_to_rpy(axis_x, axis_y, axis_z, angle):
    cos_angle = cos(angle)
    sin_angle = sin(angle)
    one_minus_cos = 1.0 - cos_angle

    r00 = cos_angle + axis_x * axis_x * one_minus_cos
    r01 = axis_x * axis_y * one_minus_cos - axis_z * sin_angle
    r02 = axis_x * axis_z * one_minus_cos + axis_y * sin_angle
    r10 = axis_y * axis_x * one_minus_cos + axis_z * sin_angle
    r11 = cos_angle + axis_y * axis_y * one_minus_cos
    r12 = axis_y * axis_z * one_minus_cos - axis_x * sin_angle
    r20 = axis_z * axis_x * one_minus_cos - axis_y * sin_angle
    r21 = axis_z * axis_y * one_minus_cos + axis_x * sin_angle
    r22 = cos_angle + axis_z * axis_z * one_minus_cos

    sy = sqrt(r00 * r00 + r10 * r10)
    singular = sy < 1e-6

    if not singular:
        roll = atan2(r21, r22)
        pitch = atan2(-r20, sy)
        yaw = atan2(r10, r00)
    else:
        roll = atan2(-r12, r11)
        pitch = atan2(-r20, sy)
        yaw = 0.0

    return roll, pitch, yaw


def _launch_setup(context, *args, **kwargs):
    calibration = load_calibration(LaunchConfiguration("calibration_file").perform(context))
    camera_ip = LaunchConfiguration("camera_ip").perform(context)
    save_dir = LaunchConfiguration("save_dir").perform(context)

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=transform_arguments(
                calibration["external_calibration"],
                calibration["external_calibration"]["frame_id"],
                calibration["external_calibration"]["child_frame_id"],
            ),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=transform_arguments(
                calibration["camera_optical_transform"],
                calibration["camera_frame"],
                calibration["camera_optical_frame"],
            ),
        ),
        Node(
            package="mecheye_ros_interface",
            executable="start",
            name="mechmind_camera_publisher_service",
            output="screen",
            parameters=[
                {"save_file": True},
                {"save_dir": save_dir},
                {"camera_ip": camera_ip},
                {"use_external_intri": False},
                {"fx": 1727.4641025602748},
                {"fy": 1727.4586926701952},
                {"u": 655.8180825729554},
                {"v": 516.6306500606158},
                {"color_frame_id": calibration["camera_optical_frame"]},
                {"depth_frame_id": calibration["camera_optical_frame"]},
                {"point_cloud_frame_id": calibration["camera_optical_frame"]},
                {"textured_point_cloud_frame_id": calibration["camera_optical_frame"]},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_ip",
                default_value="192.168.1.108",
                description="IP address of the Mech-Eye camera",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(DEFAULT_CALIBRATION_FILE),
                description="YAML file that defines the external calibration TFs",
            ),
            DeclareLaunchArgument(
                "save_dir",
                default_value="",
                description="If set, every capture is also saved here, timestamped: "
                "<stamp>_point_cloud.ply, and with capture_all also _color.png and _depth.tiff",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
