import os
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def apply_algorithm_defaults(context):
    """Pin the retained algorithm unless the caller already exported a value."""
    defaults = {
        "LIO_SAMPLE_MODE": "normal",
        "LIO_SAMPLE_BUDGET": "1500",
        "PGO_RETRY_CANDIDATES": "1",
        "PGO_DIRECTION_MODE": "weak100",
    }
    chosen = {}
    actions = []
    for name, value in defaults.items():
        current = os.environ.get(name)
        if current:
            chosen[name] = current
        else:
            chosen[name] = value
            actions.append(SetEnvironmentVariable(name, value))

    audit = os.environ.get("PGO_DIRECTION_AUDIT_PATH")
    if not audit:
        audit_dir = Path.cwd() / "mapping_runs" / "launch_default"
        audit_dir.mkdir(parents=True, exist_ok=True)
        audit = str(audit_dir / "direction_audit.jsonl")
        actions.append(SetEnvironmentVariable("PGO_DIRECTION_AUDIT_PATH", audit))
    chosen["PGO_DIRECTION_AUDIT_PATH"] = audit
    summary = " ".join(f"{name}={value}" for name, value in chosen.items())
    actions.append(LogInfo(msg="algorithm settings: " + summary))
    return actions


def generate_launch_description():
    lio_config = PathJoinSubstitution([FindPackageShare("fastlio2"), "config", "lio.yaml"])
    pgo_config = PathJoinSubstitution([FindPackageShare("pgo"), "config", "pgo.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("pgo"), "rviz", "pgo.rviz"])

    return launch.LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz2"),
        OpaqueFunction(function=apply_algorithm_defaults),
        Node(
            package="fastlio2", namespace="fastlio2", executable="lio_node",
            name="lio_node", output="screen",
            parameters=[{"config_path": lio_config.perform(launch.LaunchContext())}],
        ),
        Node(
            package="pgo", namespace="pgo", executable="pgo_node",
            name="pgo_node", output="screen",
            parameters=[{"config_path": pgo_config.perform(launch.LaunchContext())}],
        ),
        Node(
            package="rviz2", namespace="pgo", executable="rviz2", name="rviz2",
            output="screen", arguments=["-d", rviz_config.perform(launch.LaunchContext())],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
