#!/usr/bin/env python3
"""Read-only launch composition regression; source ROS and workspace first."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'simlaunch', root / 'src/planner/plan_manage/launch/sentry_sim.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
for action in module.generate_launch_description().entities:
    if isinstance(action, DeclareLaunchArgument):
        context.launch_configurations[action.name] = perform_substitutions(
            context, action.default_value)
context.launch_configurations.update(
    navi_mode='2', execution_mode='waypoint_z_preview',
    init_x='1.9', init_y='-6', init_z='.17', start_rviz='false',
    keypoints_file=str(root / 'docs/testing/maps/field/south_tunnel_waypoint_z_mode2.yaml'))


def executables():
    return [node.node_executable if isinstance(node.node_executable, str)
            else perform_substitutions(context, node.node_executable)
            for node in module._setup(context) if isinstance(node, Node)]


preview = executables()
assert 'open_loop_controller' in preview
assert 'closed_loop_controller' not in preview
assert not any('kinematic' in name for name in preview)
for parameter in ('ground_grid_file', 'ground_file'):
    context.launch_configurations[parameter] = 'forbidden'
    try:
        module._setup(context)
    except RuntimeError as error:
        assert 'forbids' in str(error)
    else:
        raise AssertionError('support surface allowed')
    context.launch_configurations[parameter] = ''
context.launch_configurations['execution_mode'] = 'closed_loop'
default = executables()
assert 'open_loop_controller' not in default
assert 'closed_loop_controller' in default
assert any('kinematic' in name for name in default)
print('PASS: preview executor isolation, both support inputs rejected, default preserved')
