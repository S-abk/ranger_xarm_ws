"""Launch-description tests for the two RealSense cameras (CONTRACT INV-7).

No process is started and no hardware is touched. The launch descriptions are
walked with the launch API instead of run: every entity is visited the way a
LaunchService would visit it (conditions evaluated, OpaqueFunctions executed,
GroupActions and includes expanded), except that Node / ExecuteProcess actions
are only recorded, never executed.

robot.launch.py also includes MoveIt, the base driver, the lidars and so on.
Those includes are recorded but not descended into: they are not what is under
test, and descending would run their own launch code for no benefit. The
camera includes (d400_camera and the realsense2_camera rs_launch.py under it)
are descended into, so the Node that would be started is the real one.
"""
import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import (ExecuteLocal, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import evaluate_parameters
import pytest

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / 'launch'

# Includes the walker follows; everything else is recorded and skipped.
DESCEND = ('d400_camera.launch.py', 'rs_launch.py')

GOOD_GANTRY = '012345678901'
GOOD_WRIST = '987654321098'


def load(filename):
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), LAUNCH / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Walk:
    def __init__(self):
        self.processes = []   # Node/ExecuteLocal actions, never executed
        self.facts = {}       # realsense node -> (namespace, name, params)
        self.skipped = []     # includes recorded but not descended into
        self.disabled = []    # actions whose condition was false

    def realsense_nodes(self):
        return [n for n in self.processes
                if isinstance(n, Node) and n.node_package == 'realsense2_camera']

    def node_facts(self, node):
        return self.facts[node]


def walk(entities, context, result=None, descend=DESCEND):
    result = result if result is not None else Walk()
    for entity in entities:
        condition = getattr(entity, 'condition', None)
        if condition is not None and not condition.evaluate(context):
            result.disabled.append(entity)
            continue
        if isinstance(entity, (Node, ExecuteLocal)):
            result.processes.append(entity)
            if isinstance(entity, Node) and entity.node_package == 'realsense2_camera':
                # Evaluated now: the camera's launch configurations only exist
                # inside its scope, which is popped once the walk moves on.
                result.facts[entity] = node_facts(entity, context)
        elif isinstance(entity, TimerAction):
            walk(entity.actions, context, result, descend)
        elif (isinstance(entity, IncludeLaunchDescription)
              and not any(name in entity.launch_description_source.location
                          for name in descend)):
            result.skipped.append(entity)
        else:
            sub = entity.visit(context)
            if sub:
                walk(sub, context, result, descend)
    return result


def run(module, args, descend=DESCEND):
    context = LaunchContext()
    context.launch_configurations.update(args)
    description = module.generate_launch_description()
    return walk(description.entities, context, descend=descend), context


def node_facts(node, context):
    """Namespace, name and parameters the node would start with."""
    def expand(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    params = {}
    for p in evaluate_parameters(context, node._Node__parameters):
        params.update(p if isinstance(p, dict) else {})
    return (expand(node._Node__node_namespace), expand(node._Node__node_name),
            params)


# Every non-camera device off, MoveIt's required argument supplied.
BASE_ARGS = {
    'robot_ip': '192.0.2.1',
    'ouster_sensor_hostname': 'os-test.invalid',
}
OTHERS_OFF = {
    'enable_base': 'false', 'enable_deadman': 'false',
    'enable_rplidar': 'false', 'enable_ouster': 'false',
    'enable_ouster_viz_cloud': 'false',
}


@pytest.fixture(scope='module')
def d400():
    return load('d400_camera.launch.py')


@pytest.fixture(scope='module')
def robot():
    return load('robot.launch.py')


def camera_args(**overrides):
    args = {'camera_name': 'gantry_d455', 'device_type': 'd455',
            'serial_no': GOOD_GANTRY}
    args.update(overrides)
    return args


# ---- (a) d400_camera.launch.py --------------------------------------------

def test_d400_without_serial_raises(d400):
    args = camera_args()
    del args['serial_no']
    with pytest.raises(RuntimeError, match='serial_no'):
        run(d400, args)


@pytest.mark.parametrize('bad', ['', '   ', 'abc', '12ab', '123 456', '__123',
                                 '-123', '1.5', '٣٤'])
def test_d400_bad_serial_raises_and_starts_nothing(d400, bad):
    with pytest.raises(RuntimeError, match='serial'):
        run(d400, camera_args(serial_no=bad))


def test_d400_missing_device_type_or_name_raises(d400):
    args = camera_args()
    del args['device_type']
    with pytest.raises(RuntimeError, match='device_type'):
        run(d400, args)
    with pytest.raises(RuntimeError, match='device_type'):
        run(d400, camera_args(device_type=''))
    with pytest.raises(RuntimeError, match='camera_name'):
        run(d400, camera_args(camera_name='bad name!'))


@pytest.mark.parametrize('given', [GOOD_GANTRY, '_' + GOOD_GANTRY])
def test_d400_node_is_bound_to_the_serial(d400, capsys, given):
    result, context = run(d400, camera_args(serial_no=given))
    (node,) = result.realsense_nodes()
    namespace, name, params = result.node_facts(node)
    assert (namespace, name) == ('camera', 'gantry_d455')
    # A str with the leading zero kept: the underscore is what stops ROS 2
    # reading the digits as an integer. The driver strips it.
    assert params['serial_no'] == '_' + GOOD_GANTRY
    assert isinstance(params['serial_no'], str)
    assert params['base_frame_id'] == 'link'
    assert params['device_type'] == 'd455'
    assert params['rgb_camera.color_profile'] == '640x480x15'
    assert params['depth_module.depth_profile'] == '640x480x15'
    assert params['initial_reset'] is False
    for stream in ('enable_infra', 'enable_infra1', 'enable_infra2',
                   'enable_gyro', 'enable_accel', 'enable_motion',
                   'pointcloud.enable', 'align_depth.enable'):
        assert params[stream] is False, stream
    # The scoped, non-forwarding design: no bare launch argument of ours may
    # reach the driver, which would print "Parameter ... is not supported".
    assert 'is not supported' not in capsys.readouterr().out


def test_d400_profiles_and_reset_are_applied(d400):
    result, context = run(d400, camera_args(
        color_profile='1280x720x30', depth_profile='848x480x30',
        initial_reset='true'))
    (node,) = result.realsense_nodes()
    params = result.node_facts(node)[2]
    assert params['rgb_camera.color_profile'] == '1280x720x30'
    assert params['depth_module.depth_profile'] == '848x480x30'
    assert params['initial_reset'] is True


# ---- (b) robot.launch.py: a missing serial fails closed -------------------

@pytest.mark.parametrize('camera', ['gantry_d455', 'wrist_d435'])
def test_robot_enabled_camera_without_serial_raises(robot, camera):
    other = 'wrist_d435' if camera == 'gantry_d455' else 'gantry_d455'
    args = {**BASE_ARGS, **OTHERS_OFF, f'enable_{other}': 'false'}
    with pytest.raises(RuntimeError, match=f'{camera}_serial_no'):
        run(robot, args)
    # Default is enabled: the same must hold with no enable_* given at all.
    args = {**BASE_ARGS, **OTHERS_OFF, f'enable_{other}': 'false',
            f'{other}_serial_no': GOOD_WRIST}
    with pytest.raises(RuntimeError, match=f'{camera}_serial_no'):
        run(robot, args)


@pytest.mark.parametrize('bad', ['', 'abc', '12 34'])
def test_robot_malformed_serial_raises_naming_the_argument(robot, bad):
    args = {**BASE_ARGS, **OTHERS_OFF, 'enable_gantry_d455': 'false',
            'wrist_d435_serial_no': bad}
    with pytest.raises(RuntimeError, match='wrist_d435_serial_no'):
        run(robot, args)


def test_robot_same_serial_for_both_cameras_raises(robot):
    args = {**BASE_ARGS, **OTHERS_OFF,
            'gantry_d455_serial_no': GOOD_GANTRY,
            'wrist_d435_serial_no': '_' + GOOD_GANTRY}
    with pytest.raises(RuntimeError, match='same'):
        run(robot, args)


def test_robot_serials_have_no_default_and_are_not_declared(robot):
    from launch.actions import DeclareLaunchArgument
    declared = {a.name: a for a in robot.generate_launch_description()
                .get_launch_arguments()}
    assert not [n for n in declared if n.endswith('_serial_no')]
    assert declared['enable_gantry_d455'].default_value is not None
    assert declared['enable_wrist_d435'].default_value is not None
    assert isinstance(declared['robot_ip'], DeclareLaunchArgument)


def test_robot_camera_check_runs_before_anything_that_starts(robot):
    """A bad serial must abort before MoveIt, the arm or the base start."""
    entities = robot.generate_launch_description().entities
    camera = next(i for i, e in enumerate(entities)
                  if isinstance(e, OpaqueFunction))
    starts = [i for i, e in enumerate(entities)
              if isinstance(e, (Node, ExecuteLocal, IncludeLaunchDescription,
                                TimerAction))]
    assert camera < min(starts)


# ---- (c) cameras off: everything else still launches ----------------------

def test_robot_with_both_cameras_off_needs_no_serial_and_starts_no_camera(robot):
    args = {**BASE_ARGS, **OTHERS_OFF,
            'enable_gantry_d455': 'false', 'enable_wrist_d435': 'false'}
    result, context = run(robot, args)
    assert result.realsense_nodes() == []
    assert not any('d400_camera' in i.launch_description_source.location
                   for i in result.skipped)
    # MoveIt (and its robot_state_publisher) is unconditional and still there.
    moveit = [i for i in result.skipped
              if 'real_execution.launch.py' in i.launch_description_source.location]
    assert len(moveit) == 1 and moveit[0].condition is None
    # The unconditional static transform is the only process left.
    assert [(n.node_package, n.node_executable) for n in result.processes] == [
        ('tf2_ros', 'static_transform_publisher')]


def test_robot_moveit_still_launches_with_cameras_off_and_on(robot):
    """Walk MoveIt's own launch too: the description must expand without a camera.

    Nothing is executed -- the xacro is expanded and the Node actions are only
    recorded -- so this needs neither hardware nor a running ROS graph.
    """
    everything = DESCEND + ('.launch.py', '.launch.xml')
    off = {**BASE_ARGS, **OTHERS_OFF,
           'enable_gantry_d455': 'false', 'enable_wrist_d435': 'false'}
    on = {**BASE_ARGS, **OTHERS_OFF,
          'gantry_d455_serial_no': GOOD_GANTRY, 'wrist_d435_serial_no': GOOD_WRIST}
    for args, cameras in ((off, 0), (on, 2)):
        result, _ = run(robot, args, descend=everything)
        executables = [(n.node_package, n.node_executable) for n in result.processes]
        assert ('moveit_ros_move_group', 'move_group') in executables
        assert executables.count(('robot_state_publisher', 'robot_state_publisher')) == 1
        assert len(result.realsense_nodes()) == cameras


def test_robot_with_one_camera_off_does_not_need_its_serial(robot):
    args = {**BASE_ARGS, **OTHERS_OFF, 'enable_wrist_d435': 'false',
            'gantry_d455_serial_no': GOOD_GANTRY}
    result, context = run(robot, args)
    (node,) = result.realsense_nodes()
    assert result.node_facts(node)[1] == 'gantry_d455'


# ---- (d) both cameras: right model, right serial, independent -------------

def test_robot_both_cameras_get_their_own_model_and_serial(robot, capsys):
    args = {**BASE_ARGS, **OTHERS_OFF,
            'gantry_d455_serial_no': GOOD_GANTRY,
            'wrist_d435_serial_no': '_' + GOOD_WRIST}
    result, context = run(robot, args)
    nodes = {result.node_facts(n)[1]: result.node_facts(n)
             for n in result.realsense_nodes()}
    assert sorted(nodes) == ['gantry_d455', 'wrist_d435']

    for name, device_type, serial in (('gantry_d455', 'd455', GOOD_GANTRY),
                                      ('wrist_d435', 'd435(?!i)', GOOD_WRIST)):
        namespace, _, params = nodes[name]
        assert namespace == 'camera'
        assert params['device_type'] == device_type
        assert params['serial_no'] == '_' + serial
        assert params['base_frame_id'] == 'link'
    assert 'is not supported' not in capsys.readouterr().out


def test_robot_camera_initial_reset_is_per_camera(robot):
    args = {**BASE_ARGS, **OTHERS_OFF,
            'gantry_d455_serial_no': GOOD_GANTRY,
            'wrist_d435_serial_no': GOOD_WRIST,
            'wrist_d435_initial_reset': 'true'}
    result, context = run(robot, args)
    reset = {result.node_facts(n)[1]: result.node_facts(n)[2]['initial_reset']
             for n in result.realsense_nodes()}
    assert reset == {'gantry_d455': False, 'wrist_d435': True}


def test_robot_includes_this_packages_camera_launch(robot):
    """The include must resolve to the file next to this test, not a stale copy."""
    args = {**BASE_ARGS, **OTHERS_OFF,
            'gantry_d455_serial_no': GOOD_GANTRY,
            'enable_wrist_d435': 'false'}
    result, context = run(robot, args)
    assert len(result.realsense_nodes()) == 1
    resolved = context.perform_substitution(PathJoinSubstitution([
        FindPackageShare('ranger_xarm_sensors'), 'launch', 'd400_camera.launch.py']))
    assert os.path.realpath(resolved) == str((LAUNCH / 'd400_camera.launch.py').resolve())


# ---- the two serial validators must agree ---------------------------------

@pytest.mark.parametrize('raw', [
    GOOD_GANTRY, '_' + GOOD_GANTRY, ' ' + GOOD_GANTRY + ' ', '0', '_0',
    '', ' ', '_', '__1', 'abc', '12ab', '1 2', '-1', '1e5', '٣'])
def test_robot_and_d400_validators_agree(robot, d400, raw):
    def d400_result():
        try:
            return d400.normalize_serial(raw)
        except RuntimeError:
            return None

    def robot_result():
        context = LaunchContext()
        context.launch_configurations['x_serial_no'] = raw
        try:
            return robot._camera_serial(context, 'x')
        except RuntimeError:
            return None

    assert d400_result() == robot_result()
