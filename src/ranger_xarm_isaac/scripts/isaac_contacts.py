"""Collision ground truth: what the robot touches, from PhysX contact reports.

Anything built on the robot's sensors sees a collision only as well as the
sensors see the obstacle (the Ouster, on the gantry, cannot see low things
close to the base). This reports what physics says actually touched:
every robot rigid body gets PhysxContactReportAPI, and after each physics
step the contact report is read and any pair of (robot body, something
that is neither the robot nor the ground) is sent on.

Ground is a list of prim roots (by default the ground plane isaac_bringup.py
adds): the wheels touch it all the time. Anything else counts, wheels
included (a wheel against a wall is a collision too).

Isaac runs its own Python 3.11, and the system's Jazzy rclpy is built for
3.12; the bridge's bundled rclpy only works if its libraries replace the
system ones for the whole process, bridge included. So this does not
publish itself: it sends one JSON datagram per step, while anything is
touching, to a UDP port on localhost, and isaac_contact_relay.py (system
ROS, started by control.launch.py) publishes it as /ground_truth/contacts.
A datagram with an empty list marks the end of a contact.

    {"t": <sim time s>, "contacts": [
        {"link": "front_left_wheel_link", "other": "/Environment/...",
         "force": <N, summed over the pair's contact points>,
         "depth": <m, deepest penetration, >= 0>,
         "p": [x, y, z]}, ...]}
"""
import json
import socket

from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import ContactEventType
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

DEFAULT_PORT = 47811
GROUND = ('/groundPlane',)


class ContactReporter:
    def __init__(self, stage, robot_root, ground=GROUND, port=DEFAULT_PORT):
        self.robot_root = robot_root.rstrip('/') + '/'
        self.ground = tuple(g.rstrip('/') for g in ground)
        self.bodies = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                api.CreateThresholdAttr().Set(0.0)    # report every contact
                self.bodies.append(str(prim.GetPath()))
        self.sim = get_physx_simulation_interface()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = ('127.0.0.1', port)
        self.paths = {}          # PhysX int path -> str
        self.touching = False
        print(f'[contacts] reporting contacts of {len(self.bodies)} robot bodies '
              f'(ignoring {", ".join(self.ground)}) to udp://127.0.0.1:{port}', flush=True)

    def _path(self, i):
        p = self.paths.get(i)
        if p is None:
            p = self.paths[i] = str(PhysicsSchemaTools.intToSdfPath(i))
        return p

    def _is_robot(self, p):
        return p.startswith(self.robot_root)

    def _is_ground(self, p):
        return any(p == g or p.startswith(g + '/') for g in self.ground)

    def step(self, sim_time, dt):
        headers, data = self.sim.get_contact_report()
        pairs = {}
        for h in headers:
            if h.type == ContactEventType.CONTACT_LOST:
                continue
            a0, a1 = self._path(h.actor0), self._path(h.actor1)
            r0, r1 = self._is_robot(a0), self._is_robot(a1)
            if r0 == r1:                        # the robot with itself, or neither
                continue
            link, other = (a0, self._path(h.collider1)) if r0 else (a1, self._path(h.collider0))
            if self._is_ground(other):
                continue
            key = (link, other)
            e = pairs.setdefault(key, {'link': link.rsplit('/', 1)[-1], 'other': other,
                                       'force': 0.0, 'depth': 0.0, 'p': None})
            for k in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                d = data[k]
                imp = d.impulse
                e['force'] += (imp[0] ** 2 + imp[1] ** 2 + imp[2] ** 2) ** 0.5 / dt
                if -d.separation >= e['depth']:
                    e['depth'] = max(0.0, -d.separation)
                    e['p'] = [round(float(c), 3) for c in d.position]
        if not pairs and not self.touching:
            return
        for e in pairs.values():
            e['force'] = round(e['force'], 2)
            e['depth'] = round(e['depth'], 4)
            if not self.touching or (e['link'], e['other']) not in self.last:
                print(f'[contacts] t={sim_time:.2f} {e["link"]} touches {e["other"]} '
                      f'at {e["p"]} ({e["force"]:.0f} N)', flush=True)
        self.last = set((e['link'], e['other']) for e in pairs.values())
        self.touching = bool(pairs)
        msg = json.dumps({'t': round(sim_time, 4), 'contacts': list(pairs.values())})
        try:
            self.sock.sendto(msg.encode(), self.addr)
        except OSError:
            pass
