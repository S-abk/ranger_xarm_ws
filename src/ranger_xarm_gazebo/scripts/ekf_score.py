#!/usr/bin/env python3
"""Score raw wheel odometry and the EKF against ground truth on one drive."""
import math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class S(Node):
    def __init__(self, ekf_topic, extra=()):
        super().__init__('ekf_score'); self.o=self.f=self.g=None
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'o',m),20)
        self.create_subscription(Odometry,ekf_topic,lambda m:setattr(self,'f',m),20)
        self.create_subscription(Odometry,'/ground_truth/odom',lambda m:setattr(self,'g',m),20)
        # Any further odometry topics are scored alongside, e.g. /kiss/odometry.
        self.x = {t: None for t in extra}
        for t in extra:
            self.create_subscription(Odometry, t, lambda m, t=t: self.x.__setitem__(t, m), 20)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
    def p(self,m):
        q=m.pose.pose.position; return (q.x,q.y,yaw(m.pose.pose.orientation))
    def t(self):
        s=self.g.header.stamp; return s.sec+s.nanosec*1e-9
    def pump(self,sec,tw=None):
        t0=self.t()
        while self.t()-t0<sec and rclpy.ok():
            if tw is not None: self.pub.publish(tw)
            rclpy.spin_once(self,timeout_sec=0.02)

def main():
    # ekf_score.py [ekf_topic] [extra_odometry_topic ...]
    ekf_topic = sys.argv[1] if len(sys.argv)>1 else '/odometry/filtered'
    extra = sys.argv[2:]
    rclpy.init(); n=S(ekf_topic, extra); t0=time.time()
    ready = lambda: (n.o is not None and n.f is not None and n.g is not None
                     and all(v is not None for v in n.x.values()))
    while not ready() and time.time()-t0<40:
        rclpy.spin_once(n,timeout_sec=0.2)
    if not ready():
        print(f"missing streams: odom={n.o is not None} ekf({ekf_topic})={n.f is not None} truth={n.g is not None} "
              + ' '.join(f'{t}={v is not None}' for t, v in n.x.items()))
        return
    o0,f0,g0=n.p(n.o),n.p(n.f),n.p(n.g); gp=g0; dist=0.0
    x0 = {t: n.p(m) for t, m in n.x.items()}
    for vx,vy,wz,dur in ((0.35,0,0,6.0),(0.30,0,0.4,6.0),(0.0,0.30,0,4.0),(0.35,0,0,6.0)):
        tw=Twist(); tw.linear.x=float(vx); tw.linear.y=float(vy); tw.angular.z=float(wz)
        n.pump(dur,tw); n.pub.publish(Twist()); n.pump(1.5)
        g=n.p(n.g); dist+=math.hypot(g[0]-gp[0],g[1]-gp[1]); gp=g
    o,f,g=n.p(n.o),n.p(n.f),n.p(n.g)
    # Each estimator lives in its own world frame, and after a teleport that
    # frame is rotated relative to ground truth by whatever heading the
    # estimator had accumulated. Comparing raw displacement vectors would
    # score that frame offset as estimator error, so rotate the estimator's
    # displacement by the initial heading difference before differencing.
    def pe(a,a0):
        d=g0[2]-a0[2]; c,sn=math.cos(d),math.sin(d)
        dx,dy=a[0]-a0[0],a[1]-a0[1]
        return math.hypot((c*dx-sn*dy)-(g[0]-g0[0]),(sn*dx+c*dy)-(g[1]-g0[1]))
    def ye(a,a0):
        d=(a[2]-a0[2])-(g[2]-g0[2]); return math.degrees(math.atan2(math.sin(d),math.cos(d)))
    print(f"ground-truth path {dist:.3f} m, net heading {math.degrees(g[2]-g0[2]):+.1f} deg")
    print(f"{'estimator':22s} {'pos err':>9s} {'% path':>8s} {'yaw err':>9s}")
    print(f"{'wheel odometry':22s} {pe(o,o0):9.3f} {100*pe(o,o0)/dist:7.2f}% {ye(o,o0):+8.2f}")
    print(f"{'EKF (odom+gyro)':22s} {pe(f,f0):9.3f} {100*pe(f,f0)/dist:7.2f}% {ye(f,f0):+8.2f}")
    for t, m in n.x.items():
        a = n.p(m)
        print(f"{t:22s} {pe(a,x0[t]):9.3f} {100*pe(a,x0[t])/dist:7.2f}% {ye(a,x0[t]):+8.2f}")
    rclpy.shutdown()
main()
