"""
PIPELINE 3 (copied from pipeline 2) copy of nero_mtc_task's mtc_pick_and_place.launch.py.

Identical MoveIt parameter building (same _moveit_config_builder, same parameter file, same node name), with ONE difference: it starts the pipeline-2 script
p2_mtc.py (next to this file) instead of the installed `mtc_pick_and_place` executable, so pipeline 1's installed package is not touched at all.
Use:  ros2 launch ~/nero_pipeline_launch/pipeline3/p3_mtc.launch.py arm_type:=nero effector_type:=agx_gripper tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"
"""
P3_MTC_LAUNCH_VERSION = "p3-mtclaunch-v1"
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction

P3_MTC_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "p3_mtc.py")   # PIPELINE 3: its own copy


def _load_agx_moveit_config_builder():
    """
    Reuse agx_arm_moveit's own _moveit_config_builder.build_moveit_config,
    same proven pattern already used for gsam2_ros/contact_graspnet_ros's
    MoveItPy integration earlier in this project - MTC's PipelinePlanner
    and ComputeIK need the full moveit config (kinematics.yaml, ompl.*,
    robot_description) on the node, exactly like move_group does; a bare
    rclcpp.Node has none of this by default, which is why "No kinematics
    solver instantiated for group 'arm'" happens without it.
    """
    moveit_share = get_package_share_directory("agx_arm_moveit")
    builder_path = Path(moveit_share) / "launch" / "_moveit_config_builder.py"

    spec = importlib.util.spec_from_file_location(
        "agx_moveit_config_builder", builder_path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["agx_moveit_config_builder"] = module
    spec.loader.exec_module(module)
    return module


def _tuples_to_lists(value):
    """
    moveit_config.to_dict() can contain Python tuples for array-valued
    parameters. Passing a raw dict to Node(parameters=[...]) lets
    launch_ros re-serialize it with a `!!python/tuple` YAML tag that ROS 2's
    own parameter parser can't read - writing the file ourselves with
    yaml.safe_dump (which has no representer for a tuple at all) forces
    genuinely clean output. Same fix already proven for gsam2_ros.
    """
    if isinstance(value, (tuple, list)):
        return [_tuples_to_lists(v) for v in value]
    if isinstance(value, dict):
        return {k: _tuples_to_lists(v) for k, v in value.items()}
    return value


def _write_params_yaml(moveit_params, node_name):
    content = {node_name: {"ros__parameters": _tuples_to_lists(moveit_params)}}
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="mtc_moveit_params_", delete=False
    )
    yaml.safe_dump(content, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    agx_builder = _load_agx_moveit_config_builder()
    moveit_config = agx_builder.build_moveit_config(context)
    config_dict = moveit_config.to_dict()
    # Prevent move_group's underlying pipeline-loading machinery from even
    # attempting Pilz/CHOMP, which are broken system libraries on this
    # machine - a failed load attempt was corrupting error-handling for
    # the OMPL pipeline we actually use, even though we never explicitly
    # requested Pilz/CHOMP ourselves.
    config_dict["planning_pipelines"] = ["ompl"]
    config_dict["ompl"]["planning_plugin"] = "ompl_interface/OMPLPlanner"
    config_dict["planning_plugin"] = "ompl_interface/OMPLPlanner"
    config_dict.pop("chomp", None)
    config_dict.pop("pilz_industrial_motion_planner", None)
    params_yaml_path = _write_params_yaml(config_dict, "nero_mtc_pick_and_place")

    # Same as the Node(...) action of the original launch file, written out as the command line it produces: the node name, the parameter file and the
    # planning_plugin parameter. P2_GRASP_FALLBACKS and MTC_EXECUTE are inherited from the environment.
    mtc_process = ExecuteProcess(
        cmd=["/usr/bin/python3", P3_MTC_SCRIPT, "--ros-args", "-r", "__node:=nero_mtc_pick_and_place",
             "--params-file", params_yaml_path, "-p", "planning_plugin:=ompl_interface/OMPLPlanner"],
        output="screen",
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    return [mtc_process]


def generate_launch_description():
    agx_builder = _load_agx_moveit_config_builder()

    return LaunchDescription(
        agx_builder.declare_common_args() + [OpaqueFunction(function=_launch_setup)]
    )
