"""Validated ROS ENU odometry to PX4 FRD visual odometry bridge."""
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Bool
from .geometry import enu_ned
from .px4 import px4_topic


class VioBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        self.enabled=self.declare_parameter('enabled',False).value
        self.min_quality=int(self.declare_parameter('min_quality',50).value)
        self.timeout=float(self.declare_parameter('timeout',.2).value)
        self.last_stamp=-1; self.last_rx=-math.inf
        self.pub=self.create_publisher(VehicleOdometry,
            px4_topic('/fmu/in/vehicle_visual_odometry',VehicleOdometry),qos_profile_sensor_data)
        self.ready=self.create_publisher(Bool,'/delivery/vio_ready',1)
        self.create_subscription(Odometry,'/vio/odometry',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.1,self.status)

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        p=msg.pose.pose.position; q=msg.pose.pose.orientation; v=msg.twist.twist.linear
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z)
        if not self.enabled or stamp<=self.last_stamp or not all(math.isfinite(x) for x in values): return
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if abs(norm-1.)>.05: return
        out=VehicleOdometry(); now_us=self.get_clock().now().nanoseconds//1000
        out.timestamp=now_us; out.timestamp_sample=stamp//1000
        out.pose_frame=VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame=VehicleOdometry.VELOCITY_FRAME_FRD
        out.position=list(enu_ned((p.x,p.y,p.z)))
        # ROS FLU/ENU -> PX4 FRD/NED quaternion: fixed basis conversion.
        out.q=[q.w,q.y,q.x,-q.z]
        out.velocity=list(enu_ned((v.x,v.y,v.z)))
        out.angular_velocity=[math.nan]*3
        cov=msg.pose.covariance; tcov=msg.twist.covariance
        out.position_variance=[max(1e-6,float(cov[i])) for i in (0,7,14)]
        out.orientation_variance=[max(1e-6,float(cov[i])) for i in (21,28,35)]
        out.velocity_variance=[max(1e-6,float(tcov[i])) for i in (0,7,14)]
        out.quality=100; self.pub.publish(out)
        self.last_stamp=stamp; self.last_rx=time.monotonic()

    def status(self):
        self.ready.publish(Bool(data=self.enabled and time.monotonic()-self.last_rx<self.timeout))


def main(args=None):
    rclpy.init(args=args); node=VioBridge()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
