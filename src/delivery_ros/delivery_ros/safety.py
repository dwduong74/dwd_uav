"""Single intent gate for operator hold, forward clearance and geofence."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from delivery_interfaces.msg import FlightIntent,FlightState,ObstacleState,SafetyState
from .course import point_in_polygon


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.enabled=self.declare_parameter('enable_obstacle_guard',False).value
        self.required=self.declare_parameter('require_obstacle_data',False).value
        self.clearance=float(self.declare_parameter('obstacle_clearance',.6).value)
        self.timeout=float(self.declare_parameter('obstacle_timeout',.5).value)
        flat=list(self.declare_parameter('geofence_xy',Parameter.Type.DOUBLE_ARRAY).value or [])
        if len(flat)%2: raise ValueError('geofence_xy must contain x,y pairs')
        self.geofence=tuple((float(flat[i]),float(flat[i+1])) for i in range(0,len(flat),2))
        self.intent=None; self.flight=None; self.scan=None
        self.intent_time=self.flight_time=self.scan_time=-math.inf
        self.operator_hold=False
        self.pub=self.create_publisher(FlightIntent,'/delivery/flight_intent',1)
        self.obstacle_pub=self.create_publisher(ObstacleState,'/delivery/obstacles',10)
        self.state_pub=self.create_publisher(SafetyState,'/delivery/safety_state',10)
        self.create_subscription(FlightIntent,'/delivery/flight_intent_raw',self.on_intent,1)
        self.create_subscription(FlightState,'/delivery/flight_state',self.on_flight,10)
        self.create_subscription(LaserScan,'/depth/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Bool,'/delivery/safety_hold',self.on_hold,1)
        self.create_timer(.05,self.tick)

    def on_intent(self,msg): self.intent,self.intent_time=msg,time.monotonic()
    def on_flight(self,msg): self.flight,self.flight_time=msg,time.monotonic()
    def on_hold(self,msg): self.operator_hold=msg.data
    def on_scan(self,msg):
        values=[]
        for i,r in enumerate(msg.ranges):
            angle=msg.angle_min+i*msg.angle_increment
            if abs(angle)>math.pi/3 or math.isnan(r):
                continue
            if math.isinf(r) and r>0:
                r=msg.range_max
            if math.isfinite(r) and msg.range_min<=r<=msg.range_max:
                values.append((angle,float(r)))
        if not values: return
        def minimum(a,b):
            candidates=[r for angle,r in values if a<=angle<b]
            return min(candidates) if candidates else math.inf
        self.scan=(minimum(-math.pi/3,-math.pi/9),minimum(-math.pi/9,math.pi/9),
                   minimum(math.pi/9,math.pi/3))
        self.scan_time=time.monotonic()

    def tick(self):
        now=time.monotonic(); intent=self.intent; flight=self.flight
        if intent is None or now-self.intent_time>.25: return
        scan_valid=self.scan is not None and now-self.scan_time<self.timeout
        left,center,right=self.scan if scan_valid else (math.nan,math.nan,math.nan)
        blocked=bool(scan_valid and center<self.clearance)
        outside=bool(flight and self.geofence and not point_in_polygon(
            (flight.position.x,flight.position.y),self.geofence))
        stale=self.required and not scan_valid
        hold=self.operator_hold or (self.enabled and (blocked or outside or stale))
        out=ObstacleState(); out.header.stamp=self.get_clock().now().to_msg()
        out.valid=scan_valid; out.blocked=blocked
        out.left_clearance,out.center_clearance,out.right_clearance=left,center,right
        out.corridor_offset=(right-left)/(right+left) if scan_valid and math.isfinite(left+right) and left+right>0 else 0.
        out.detail='blocked' if blocked else ('stale' if stale else 'clear')
        self.obstacle_pub.publish(out)
        state=SafetyState(); state.header.stamp=out.header.stamp
        state.state=SafetyState.HOLD if hold else SafetyState.OK
        state.intent_allowed=not hold; state.detail=('operator hold' if self.operator_hold else out.detail)
        self.state_pub.publish(state)
        if not hold:
            self.pub.publish(intent); return
        guarded=FlightIntent(); guarded.header.stamp=out.header.stamp; guarded.header.frame_id='map'
        guarded.command_id=intent.command_id
        # LAND commands must pass through a guard. Other flight commands wait in hold.
        guarded.command=intent.command if intent.command==FlightIntent.LAND else FlightIntent.NONE
        if flight and now-self.flight_time<.3:
            guarded.stream=True
            guarded.position=flight.position
        self.pub.publish(guarded)


def main(args=None):
    rclpy.init(args=args); node=SafetyMonitor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
