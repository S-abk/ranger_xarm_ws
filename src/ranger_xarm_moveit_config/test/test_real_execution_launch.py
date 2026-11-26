"""real_execution.launch.py: the semantic description and the description command, performed.

The launch description is built, its arguments are declared in a launch context, and two
parameter values of move_group are performed there: robot_description_semantic (the
FileContent / IfElseSubstitution / EqualsSubstitution chain) and the robot_description
command's text. No node is started and no process may start (checked by a fixture).
"""
import asyncio
import importlib.util
import os
import subprocess
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parents[1]
SHARE = Path(get_package_share_directory('ranger_xarm_moveit_config'))
SHIPPED = SHARE / 'config' / 'ranger_xarm.srdf'
MOBILE = SHARE / 'config' / 'ranger_xarm_mobile_base.srdf'


@pytest.fixture(autouse=True)
def no_process_may_start(monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError(f'a process was started: {args[:1]}')

    monkeypatch.setattr(subprocess, 'Popen', refuse)
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', refuse)
    monkeypatch.setattr(asyncio, 'create_subprocess_shell', refuse)
    monkeypatch.setattr(os, 'system', refuse)


def move_group_params(args, executable='move_group'):
    spec = importlib.util.spec_from_file_location(
        'real_execution', PKG / 'launch' / 'real_execution.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(args)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)          # defaults and choices, as a launch would
    move_group = [e for e in description.entities
                  if isinstance(e, Node) and e.node_executable == executable]
    assert len(move_group) == 1
    params = {}   # launch_ros normalises keys to tuples of substitutions; perform them
    for p in move_group[0]._Node__parameters:
        if isinstance(p, dict):
            params.update({perform_substitutions(context, list(k)): v for k, v in p.items()})
    return params, context


def performed(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value.value))


BASE = {'robot_ip': '192.0.2.1'}   # documentation address; nothing connects to it here


@pytest.mark.parametrize('args, expected', [
    ({'fix_base_to_world': 'true'}, SHIPPED),
    ({'fix_base_to_world': 'true', 'planning_frame': 'odom'}, SHIPPED),
    ({'fix_base_to_world': 'true', 'planning_frame': 'base_footprint'}, SHIPPED),
    ({}, SHIPPED),                                                           # all defaults
    ({'fix_base_to_world': 'false', 'planning_frame': 'odom'}, MOBILE),
    ({'fix_base_to_world': 'false'}, MOBILE),                               # planning_frame default
    ({'fix_base_to_world': 'false', 'planning_frame': 'base_footprint'}, SHIPPED),
])
def test_semantic_description_is_the_expected_file_byte_for_byte(args, expected):
    params, context = move_group_params({**BASE, **args})
    text = performed(context, params['robot_description_semantic'])
    assert text.encode('utf-8') == expected.read_bytes()


def test_installed_shipped_file_is_the_source_file():
    assert SHIPPED.read_bytes() == (PKG / 'config' / 'ranger_xarm.srdf').read_bytes()


def command_text(params, context):
    value = params['robot_description'].value
    commands = [s for s in normalize_to_list_of_substitutions(value) if isinstance(s, Command)]
    assert len(commands) == 1
    return perform_substitutions(context, commands[0].command)


@pytest.mark.parametrize('args', [{'fix_base_to_world': 'true'}, {}])   # given, and the default
def test_description_command_ends_with_the_weld(args):
    # robot_state_publisher's description: the command ends with the weld argument
    params, context = move_group_params({**BASE, **args}, 'robot_state_publisher')
    text = command_text(params, context)
    assert text.split()[-1] == 'fix_base_to_world:=true', text
    # move_group's planning description: the same command, then only the planning margin
    params, context = move_group_params({**BASE, **args})
    text = command_text(params, context)
    assert text.split()[-2:] == ['fix_base_to_world:=true', 'planning_margin:=0.005'], text
