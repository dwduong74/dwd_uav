"""The only /fmu/in writer. No perception or action execution in this process."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleCommandAck, VehicleLocalPosition, VehicleStatus, VehicleLandDetected,
    VehicleGlobalPosition, VehicleOdometry, SensorGps, BatteryStatus, EstimatorStatusFlags)
from delivery_interfaces.msg import FlightState, FlightIntent
from .geometry import enu_ned, px4_attitude_to_ros, rotate
from .command import CommandTracker


def px4_topic(base, message_type):
    """PX4 appends _vN for nonzero MESSAGE_VERSION (since PX4 1.16)."""
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    return base + (f'_v{version}' if version else '')


class Px4Adapter(Node):
    def __init__(self):
        super().__init__('px4_adapter')
        self.enabled = self.declare_parameter('enable_control', False).value
        self.timeout = self.declare_parameter('telemetry_timeout', 2.).value
        self.system = self.declare_parameter('target_system', 1).value
        self.allow_vio_without_gps = self.declare_parameter('allow_vio_without_gps',False).value
        self.component = 191
        self.range_source = self.declare_parameter('range_source','px4').value
        if self.range_source not in ('px4','ros_scan'):
            raise ValueError('range_source must be px4 or ros_scan')
        self.scan = None
        self.scan_time = -math.inf
        if self.range_source == 'ros_scan':
            self.create_subscription(LaserScan,'/camera/scan',self.on_scan,qos_profile_sensor_data)
        self.data, self.received = {}, {}
        self.intent = None
        self.intent_time = -math.inf
        self.tracker = CommandTracker()
        self.pub = self.create_publisher(FlightState, '/delivery/flight_state', 10)
        self.mode_pub = self.create_publisher(OffboardControlMode, px4_topic('/fmu/in/offboard_control_mode', OffboardControlMode), 10)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, px4_topic('/fmu/in/trajectory_setpoint', TrajectorySetpoint), 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, px4_topic('/fmu/in/vehicle_command', VehicleCommand), 10)
        self.tf = TransformBroadcaster(self)
        for key, cls in [('vehicle_local_position',VehicleLocalPosition),
                         ('vehicle_status',VehicleStatus), ('vehicle_land_detected',VehicleLandDetected),
                         ('vehicle_global_position',VehicleGlobalPosition), ('vehicle_odometry',VehicleOdometry),
                         ('vehicle_gps_position',SensorGps), ('battery_status',BatteryStatus),
                         ('estimator_status_flags',EstimatorStatusFlags)]:
            self.create_subscription(cls, px4_topic('/fmu/out/'+key, cls),
                                     lambda m,k=key: self.sample(k,m), qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, px4_topic('/fmu/out/vehicle_command_ack', VehicleCommandAck), self.ack, qos_profile_sensor_data)
        self.create_subscription(FlightIntent, '/delivery/flight_intent', self.on_intent, 1)
        self.create_timer(.05, self.tick)

    def sample(self, key, msg):
        old = self.data.get(key)
        if old is not None and msg.timestamp <= old.timestamp:
            return
        self.data[key], self.received[key] = msg, time.monotonic()

    def ack(self, msg):
        if msg.target_system == self.system and msg.target_component == self.component:
            self.tracker.ack(msg.command, msg.result, msg.timestamp)

    def on_scan(self,msg):
        age=self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if 0 <= age < .5:
            self.scan,self.scan_time=msg,time.monotonic()

    def on_intent(self, msg):
        age = self.get_clock().now().nanoseconds/1e9 - msg.header.stamp.sec - msg.header.stamp.nanosec/1e9
        if not 0 <= age < .25:
            return
        if not all(math.isfinite(v) for v in (msg.position.x,msg.position.y,msg.position.z)):
            return
        self.intent, self.intent_time = msg, time.monotonic()

    def state(self):
        out = FlightState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.command_id, out.command_state, out.detail = self.tracker.token, self.tracker.state, self.tracker.detail
        if len(self.data) != 8:
            out.detail = 'Waiting for PX4 DDS topics (including vehicle_land_detected)'
            return out
        p, s = self.data['vehicle_local_position'], self.data['vehicle_status']
        g, gps = self.data['vehicle_global_position'], self.data['vehicle_gps_position']
        b, land = self.data['battery_status'], self.data['vehicle_land_detected']
        out.position.x, out.position.y, out.position.z = enu_ned((p.x,p.y,p.z))
        out.latitude, out.longitude = g.lat, g.lon
        out.ref_latitude, out.ref_longitude, out.ref_altitude = p.ref_lat,p.ref_lon,p.ref_alt
        out.reference_timestamp = p.ref_timestamp
        out.xy_reset_counter, out.z_reset_counter = p.xy_reset_counter,p.z_reset_counter
        out.heading_reset_counter = p.heading_reset_counter
        out.armed = s.arming_state == VehicleStatus.ARMING_STATE_ARMED
        out.offboard = s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        out.auto_land = s.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        out.failsafe, out.landed = s.failsafe, land.landed
        out.range_valid = bool(p.dist_bottom_valid and p.dist_bottom_sensor_bitfield & 1
                               and math.isfinite(p.dist_bottom) and p.dist_bottom >= 0)
        out.agl, out.battery_remaining = float(p.dist_bottom),float(b.remaining)
        if self.range_source == 'ros_scan':
            scan=self.scan
            out.range_valid=False
            if scan and time.monotonic()-self.scan_time<.5 and len(scan.ranges)==1:
                r=scan.ranges[0]
                odom=self.data['vehicle_odometry']
                try:
                    w,x,y,z=odom.q
                    cosine=rotate((x,y,z,w),(0.,0.,1.))[2]
                    out.range_valid=bool(math.isfinite(r) and scan.range_min<=r<=scan.range_max and cosine>.9)
                    out.agl=float(r*cosine)
                except ValueError:
                    pass
        gps_ok = gps.fix_type >= 3 or self.allow_vio_without_gps
        out.healthy = bool(all(time.monotonic()-v < self.timeout for v in self.received.values())
            and p.xy_valid and p.z_valid and p.xy_global and p.z_global and not p.dead_reckoning
            # In 1.17 heading_good_for_control requires final in-flight mag alignment.
            # We send no yaw setpoint; require initial EKF yaw alignment before takeoff.
            and self.data['estimator_status_flags'].cs_yaw_align and gps_ok and b.connected
            and all(math.isfinite(v) for v in (p.x,p.y,p.z,g.lat,g.lon,p.ref_lat,p.ref_lon,p.ref_alt,b.remaining))
            and 0 <= b.remaining <= 1)
        return out

    def tick(self):
        now = time.monotonic()
        state = self.state()
        confirmed = {1:state.offboard, 2:state.armed and state.offboard,
                     3:state.auto_land or (state.landed and not state.armed)}.get(self.tracker.kind,False)
        self.tracker.tick(now, state.healthy and confirmed)
        state.command_state, state.detail = self.tracker.state,self.tracker.detail
        self.pub.publish(state)
        p = self.data.get('vehicle_local_position')
        odom = self.data.get('vehicle_odometry')
        if state.healthy and p and odom and odom.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            # Odometry bundles position and orientation at the same measurement time.
            stamp_us = odom.timestamp_sample
            tf = TransformStamped()
            tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(int(stamp_us)*1000, 1000000000)
            tf.header.frame_id, tf.child_frame_id = 'map','base_link'
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = enu_ned(odom.position)
            try:
                q = px4_attitude_to_ros(odom.q)
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = q
                self.tf.sendTransform(tf)
            except ValueError:
                return
        intent = self.intent
        if not self.enabled or not state.healthy or intent is None or now-self.intent_time >= .25:
            return
        us = self.get_clock().now().nanoseconds//1000
        if intent.stream:
            sp = TrajectorySetpoint()
            sp.timestamp = us
            sp.position = list(enu_ned((intent.position.x,intent.position.y,intent.position.z)))
            sp.velocity = sp.acceleration = sp.jerk = [math.nan]*3
            sp.yaw = float(-intent.yaw+math.pi/2) if intent.yaw_valid and math.isfinite(intent.yaw) else math.nan
            sp.yawspeed = math.nan
            mode = OffboardControlMode()
            mode.timestamp, mode.position = us,True
            self.mode_pub.publish(mode)
            self.sp_pub.publish(sp)
        if intent.command:
            command = {1:VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       2:VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                       3:VehicleCommand.VEHICLE_CMD_NAV_LAND}.get(intent.command)
            # ACK timestamps use PX4's synchronized clock, which may lag wall time.
            # Reject ACKs older than the latest PX4 telemetry at transaction start.
            ack_floor = max((int(msg.timestamp) for msg in self.data.values()), default=us)
            if command and self.tracker.begin(intent.command_id,intent.command,command,now,ack_floor):
                msg = VehicleCommand()
                msg.timestamp, msg.command = us, command
                msg.target_system, msg.target_component = self.system,1
                msg.source_system, msg.source_component = self.system,self.component
                msg.from_external = True
                if intent.command == 1:
                    msg.param1,msg.param2 = 1.,6.
                elif intent.command == 2:
                    msg.param1 = 1.
                else:
                    msg.param4 = msg.param5 = msg.param6 = msg.param7 = math.nan
                self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Px4Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
