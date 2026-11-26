#!/usr/bin/env python3
"""Where this workspace's packages come from, and which are missing.

  deps.py report                 every package a manifest, launch file or
                                 xacro under src/ names, and where it comes
                                 from; exit 1 if one comes from nowhere
  deps.py check TARGET           what TARGET starts that the sourced
                                 environment lacks; exit 1 if anything
  deps.py check PKG FILE         the same for one launch file

TARGET is sim, isaac or hardware (see TARGETS below). Both modes only read
files: launch files are parsed, never run, and nothing joins a ROS graph or
opens a device. A reference the parser cannot read is printed as "not
checked" rather than dropped.
"""
import argparse
import ast
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TARGETS = {
    'sim': [('ranger_xarm_gazebo', 'sim.launch.py'),
            ('ranger_xarm_description', 'display.launch.py'),
            ('ranger_xarm_moveit_config', 'planning_demo.launch.py'),
            ('ranger_xarm_moveit_config', 'fake_execution.launch.py')],
    'isaac': [('ranger_xarm_isaac', 'control.launch.py'),
              ('ranger_xarm_isaac', 'moveit.launch.py')],
    'hardware': [('ranger_xarm_sensors', 'robot.launch.py'),
                 ('ranger_xarm_bringup', 'base_bringup.launch.py'),
                 ('ranger_xarm_bringup', 'ekf_odom_imu.launch.py'),
                 ('ranger_xarm_moveit_config', 'real_hardware_check.launch.py')],
}
# Keys that upstream manifests declare but no build here needs, and that no
# rosdep rule resolves. The README's hardware rosdep line skips the same keys.
NOT_NEEDED = {
    'catkin': "ugv_sdk's ROS 1 build tool; colcon builds it as plain CMake",
    'ament_python': "ranger_mini_v3_sim_messenger lists its build type as a "
                    "dependency; it is not a package",
}
OWN = 'ranger_xarm_'
DEP_TAGS = ('depend', 'build_depend', 'build_export_depend', 'buildtool_depend',
            'exec_depend', 'test_depend')
RUN_TAGS = ('depend', 'exec_depend')
SHARE_FUNCS = ('FindPackageShare', 'get_package_share_directory')
FIND_RE = re.compile(r'\$\(find(?:-pkg-share)? ([A-Za-z0-9_]+)\)|package://([A-Za-z0-9_]+)')
# A pluginlib class: in ros2_control and MoveIt YAML, and xacro arguments.
PLUGIN_RE = re.compile(r'(?:\btype|plugin|sensor_plugin|kinematics_solver)\s*[:=]\s*'
                       r'([A-Za-z0-9_]+/[A-Za-z0-9_]+)')


def manifest_deps(xml, tags=DEP_TAGS):
    root = ET.parse(xml).getroot()
    return [(e.tag, e.text.strip()) for t in tags for e in root.iter(t)]


def _str(node):
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _name(call):
    f = call.func
    return f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', '')


def launch_refs(path):
    """(kind, package, detail, condition) for each package a Python launch
    file names. kind: node (detail = executable), file (detail = path in the
    share directory), share, or unread (detail = the source text)."""
    tree = ast.parse(path.read_text(), str(path))
    cond = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            c = next((k.value for k in n.keywords if k.arg == 'condition'), None)
            if c is not None:
                for sub in ast.walk(n):
                    cond.setdefault(id(sub), ast.unparse(c))
    refs = []
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call):
            continue
        c = cond.get(id(n))
        kw = {k.arg: k.value for k in n.keywords if k.arg}
        if 'package' in kw:
            pkg = _str(kw['package'])
            exe = _str(kw['executable']) if 'executable' in kw else None
            if pkg is None or ('executable' in kw and exe is None):
                refs.append(('unread', None, ast.unparse(n)[:100], c))
            else:
                refs.append(('node', pkg, exe, c))
        name = _name(n)
        if name in SHARE_FUNCS and n.args:
            pkg = _str(n.args[0])
            refs.append(('share', pkg, None, c) if pkg else
                        ('unread', None, ast.unparse(n)[:100], c))
        # PathJoinSubstitution([FindPackageShare('p'), 'a', 'b']) and
        # os.path.join(get_package_share_directory('p'), 'a', 'b')
        items = (n.args[0].elts if name == 'PathJoinSubstitution' and n.args
                 and isinstance(n.args[0], ast.List) else
                 n.args if name == 'join' else None)
        if items and isinstance(items[0], ast.Call) and _name(items[0]) in SHARE_FUNCS \
                and items[0].args and _str(items[0].args[0]):
            parts = [_str(x) for x in items[1:]]
            if parts and all(parts):
                refs.append(('file', _str(items[0].args[0]), '/'.join(parts), c))
            elif parts:
                refs.append(('unread', None, ast.unparse(n)[:100], c))
        if name == 'ExecuteProcess':
            refs.append(('unread', None, ast.unparse(n)[:100], c))
    return refs


# ---------------------------------------------------------------- report

def parse_repos(path):
    """{dir: {url, version, provisional}} from a vcstool .repos file."""
    out, cur = {}, None
    for line in path.read_text().splitlines():
        m = re.match(r'^  ([A-Za-z0-9_.-]+):\s*$', line)
        if m:
            cur = out.setdefault(m.group(1), {'url': '', 'version': '', 'provisional': False})
            continue
        m = re.match(r'^    (url|version):\s*(\S+)(.*)$', line)
        if m and cur is not None:
            cur[m.group(1)] = m.group(2)
            cur['provisional'] |= 'PROVISIONAL' in m.group(3)
    return out


def rosdep_resolve(key):
    r = subprocess.run(['rosdep', 'resolve', key, '--rosdistro',
                        os.environ.get('ROS_DISTRO', 'jazzy')], capture_output=True, text=True)
    lines = [ln for ln in r.stdout.splitlines() if ln and not ln.startswith('#')]
    return ' '.join(lines) if r.returncode == 0 and lines else None


def report(ws):
    tracked = set(subprocess.run(['git', '-C', str(ws), 'ls-files', 'src'],
                                 capture_output=True, text=True).stdout.split())
    mans = {}
    for p in sorted((ws / 'src').rglob('package.xml')):
        if not (p.parent / 'COLCON_IGNORE').exists():
            mans[ET.parse(p).getroot().findtext('name').strip()] = p
    own = {n: p for n, p in mans.items() if str(p.relative_to(ws)) in tracked}
    need = {}

    def add(pkg, why):
        need.setdefault(pkg, set()).add(why)

    for name, xml in mans.items():
        for tag, dep in manifest_deps(xml):
            add(dep, (name if name in own else f'{name} (pulled)', 'manifest'))
    notes = []
    for name, xml in own.items():
        for f in sorted(xml.parent.rglob('*.launch.py')):
            for kind, pkg, detail, _ in launch_refs(f):
                if kind == 'unread':
                    notes.append(f'not checked: {f.relative_to(ws)}: {detail}')
                else:
                    add(pkg, (name, 'launch'))
        for f in sorted(xml.parent.rglob('*.xacro')):
            for a, b in FIND_RE.findall(f.read_text()):
                add(a or b, (name, 'xacro'))
    for f in sorted((ws / 'tools').rglob('*.launch.py')):
        for kind, pkg, detail, _ in launch_refs(f):
            if kind != 'unread':
                add(pkg, ('tools', 'launch'))

    pinned, unfetched = {}, []
    for rf in sorted(ws.glob('*.repos')):
        for d, info in parse_repos(rf).items():
            if not (ws / 'src' / d).exists():
                unfetched.append(f'{rf.name}: {d} is not in src/, so its packages are not '
                                 f'listed (vcs import --recursive src < {rf.name})')
            for name, xml in mans.items():
                if (ws / 'src' / d) in xml.parents:
                    pinned[name] = (rf.name, d, info)

    bad = 0
    print('| package | comes from | named by |\n| --- | --- | --- |')
    for pkg in sorted(set(need) - set(own)):
        if pkg in pinned:
            rf, d, info = pinned[pkg]
            src = f'{rf}: {d} @ {info["version"][:12]}' + \
                  (' **PROVISIONAL**' if info['provisional'] else '')
        elif pkg in mans:
            src = 'in src/, but in no .repos file of this repository'
        elif pkg in NOT_NEEDED:
            src = f'not needed: {NOT_NEEDED[pkg]}'
        else:
            apt = rosdep_resolve(pkg)
            if apt:
                src = f'apt via rosdep: {apt}'
                if not any(how == 'manifest' for _, how in need[pkg]):
                    src += ' (but in no package.xml, so rosdep will not install it)'
            else:
                src, bad = '**NOT ACCOUNTED FOR**', bad + 1
        users = sorted({who for who, _ in need[pkg]})
        print(f'| `{pkg}` | {src} | {", ".join(users[:3])}'
              f'{f" +{len(users) - 3}" if len(users) > 3 else ""} |')
    for line in unfetched + notes:
        print(f'\n{line}')
    print(f'\n{len(own)} packages in this repository, {len(set(need) - set(own))} named '
          f'from outside it, {bad} accounted for nowhere.')
    return 1 if bad else 0


# ----------------------------------------------------------------- check

def check(entries):
    from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                             get_package_share_directory)

    def prefix(p):
        try:
            return get_package_prefix(p)
        except (PackageNotFoundError, ValueError):
            return None

    missing, notes, used = {}, [], set()
    plugins = {}

    def miss(what, by):
        missing.setdefault(what, set()).add(by)

    def need(p, by):
        if prefix(p) is None:
            miss(f'package {p}', by)
            return False
        used.add(p)
        return True

    seen, visited_own = set(), set()
    todo = list(entries)
    while todo:
        pkg, rel = todo.pop()
        if (pkg, rel) in seen or not need(pkg, 'the target'):
            continue
        seen.add((pkg, rel))
        share = Path(get_package_share_directory(pkg))
        path = next((p for p in (share / 'launch' / rel, share / rel) if p.is_file()), None)
        if path is None:
            miss(f'file {pkg}/launch/{rel}', 'the target')
            continue
        by = f'{pkg}/{path.name}'
        if pkg.startswith(OWN) and pkg not in visited_own:
            # The run-time dependencies of our own packages whose launch
            # files are started; not followed further, so a package needed
            # only for a file it installs does not drag in its own drivers.
            visited_own.add(pkg)
            for _, dep in manifest_deps(share / 'package.xml', RUN_TAGS):
                if not dep.startswith('python3-'):
                    need(dep, f'{pkg}/package.xml')
            for y in sorted(share.rglob('*.yaml')):
                for cls in PLUGIN_RE.findall(y.read_text(errors='replace')):
                    plugins.setdefault(cls, set()).add(f'{pkg}/{y.name}')
        text = path.read_text(errors='replace')
        for cls in PLUGIN_RE.findall(text):
            plugins.setdefault(cls, set()).add(by)
        for kind, p, detail, cond in launch_refs(path):
            when = f'{by}, when {cond}' if cond else by
            if kind == 'unread':
                notes.append(f'not checked: {by}: {detail}')
            elif need(p, when) and kind in ('node', 'file') and detail:
                if kind == 'node':
                    if not (Path(prefix(p)) / 'lib' / p / detail).exists():
                        miss(f'executable {p} {detail}', when)
                    continue
                f = Path(get_package_share_directory(p)) / detail
                if not f.exists():
                    miss(f'file {p}/{detail}', when)
                elif p.startswith(OWN) and detail.startswith('launch/') \
                        and f.name.endswith('.launch.py'):
                    todo.append((p, detail[len('launch/'):]))

    # Plugin classes against pluginlib's index: every *__pluginlib__plugin
    # resource names a plugin description XML listing its classes.
    declared = set()
    for pre in filter(None, os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)):
        idx = Path(pre) / 'share' / 'ament_index' / 'resource_index'
        for res in idx.glob('*__pluginlib__plugin/*'):
            for rel in res.read_text().split():
                xml = Path(pre) / rel
                if xml.is_file():
                    declared.update(re.findall(r'(?:type|name)\s*=\s*"([^"]+)"',
                                               xml.read_text(errors='replace')))
    for cls, by in plugins.items():
        if cls not in declared:
            for b in by:
                miss(f'plugin {cls}', b)

    # Where each package was found, outside the ROS install itself, and any
    # found in more than one workspace of the chain.
    def workspace(pre):
        # colcon's default install gives each package its own prefix under
        # <ws>/install; name the install directory instead.
        p = Path(pre)
        return str(p.parent) if (p.parent / 'setup.bash').exists() else pre

    chain = [p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    ros = f'/opt/ros/{os.environ.get("ROS_DISTRO", "")}'
    where, dupes = {}, []
    for p in sorted(used):
        hits = list(dict.fromkeys(
            workspace(pre) for pre in chain
            if (Path(pre) / 'share/ament_index/resource_index/packages' / p).exists()))
        if hits and hits[0] != ros:
            where.setdefault(hits[0], []).append(p)
        if len(hits) > 1:
            dupes.append(f'  {p}: {" , ".join(hits)}  (the first one is used)')
    print(f'{len(used)} packages found. Outside {ros or "the ROS install"}:')
    for pre in where:
        print(f'  {pre}: {" ".join(where[pre])}')
    if dupes:
        print('WARNING: in more than one workspace of the chain:')
        print('\n'.join(dupes))
    for n in sorted(set(notes)):
        print(n)
    if missing:
        print(f'MISSING, {len(missing)}:')
        for what in sorted(missing):
            print(f'  {what}    (needed by {"; ".join(sorted(missing[what]))})')
        return 1
    print('nothing missing')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split('\n', 2)[2])
    ap.add_argument('mode', choices=['report', 'check'])
    ap.add_argument('target', nargs='*', help='sim | isaac | hardware | PKG FILE')
    ap.add_argument('--ws', type=Path, default=Path(__file__).resolve().parents[2],
                    help='workspace root for report (default: this checkout)')
    a = ap.parse_args()
    if a.mode == 'report':
        return report(a.ws.resolve())
    if len(a.target) == 1 and a.target[0] in TARGETS:
        return check(TARGETS[a.target[0]])
    if len(a.target) == 2:
        return check([tuple(a.target)])
    ap.error(f'check needs one of {", ".join(TARGETS)}, or a package and a launch file')


if __name__ == '__main__':
    sys.exit(main())
