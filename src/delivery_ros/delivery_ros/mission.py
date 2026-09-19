"""Nonblocking ROS action facade over the deterministic engine."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.srv import CancelGoal
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
from delivery_interfaces.msg import MissionStatus, FlightState, FlightIntent, LandingTarget, PayloadState
from delivery_interfaces.action import ExecuteDelivery, ReleasePayload
from .engine import Engine, Config, Goal, Telemetry, Execution, Phase


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.enabled = self.declare_parameter('enable_control',False).value
        config = Config(**{k:self.declare_parameter(k,v).value for k,v in vars(Config()).items()})
        self.engine = Engine(config)
        self.default_goal = {k:self.declare_parameter(k,v).value for k,v in dict(
            latitude=0.,longitude=0.,relative_altitude=5.,delivery_marker_id=0,home_marker_id=1).items()}
        self.flight = self.payload = None
        self.flight_time = self.payload_time = -math.inf
        self.camera_ready,self.camera_time = False,-math.inf
        self.goal_handle = None
        self.reserved = False
        self.done = None
        self.mission_id = UUID()
        self.sequence = 0
        self.offset = 0
        self.last_publish = -math.inf
        self.last_revision = -1
        self.release_sent = False
        self.release_handle = None
        self.release_done = self.release_success = self.released = False
        self.release_detail = ''
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MissionStatus,'/delivery/mission_status',qos)
        self.legacy = self.create_publisher(String,'/delivery/state',10)
        self.intent_pub = self.create_publisher(FlightIntent,'/delivery/flight_intent_raw',1)
        self.create_subscription(FlightState,'/delivery/flight_state',self.on_flight,10)
        self.create_subscription(PayloadState,'/delivery/payload_state',self.on_payload,10)
        self.create_subscription(Bool,'/delivery/vision_ready',self.on_camera,1)
        self.create_subscription(LandingTarget,'/delivery/landing_target',self.on_marker,1)
        self.server = ActionServer(self,ExecuteDelivery,'/delivery/execute',
            execute_callback=self.execute,goal_callback=self.accept,cancel_callback=self.cancel,
            handle_accepted_callback=self.accepted)
        self.client = ActionClient(self,ExecuteDelivery,'/delivery/execute')
        self.release_client = ActionClient(self,ReleasePayload,'/delivery/release_payload')
        self.cancel_client = self.create_client(CancelGoal,'/delivery/execute/_action/cancel_goal')
        self.create_service(Trigger,'/delivery/start',self.start_service)
        self.create_service(Trigger,'/delivery/stop',self.stop_service)
        self.create_timer(.05,self.tick)
        self.publish_status()

    def on_flight(self,msg):
        self.flight,self.flight_time = msg,time.monotonic()

    def on_payload(self,msg):
        self.payload,self.payload_time = msg,time.monotonic()

    def on_camera(self,msg):
        self.camera_ready,self.camera_time = msg.data,time.monotonic()

    def telemetry(self):
        t = Telemetry()
        t.camera_ready = self.camera_ready and time.monotonic()-self.camera_time < .5
        f,p = self.flight,self.payload
        if f:
            t.healthy = f.healthy and time.monotonic()-self.flight_time < .3
            t.position = (f.position.x,f.position.y,f.position.z)
            t.latitude,t.longitude = f.latitude,f.longitude
            t.ref_latitude,t.ref_longitude = f.ref_latitude,f.ref_longitude
            t.reset = (f.reference_timestamp,f.xy_reset_counter,f.z_reset_counter,f.heading_reset_counter)
            t.armed,t.landed,t.offboard,t.auto_land,t.failsafe = f.armed,f.landed,f.offboard,f.auto_land,f.failsafe
            t.range_valid,t.agl,t.battery = f.range_valid,f.agl,f.battery_remaining
            t.command_id,t.command_state = f.command_id-self.offset,f.command_state
            t.command_detail = f.detail
        if p:
            t.payload_healthy = p.healthy and not p.fault and time.monotonic()-self.payload_time < .5
            t.payload_closed,t.payload_present = p.closed,p.present
            t.payload_ready = p.release_ready
        t.release_done,t.release_success,t.released = self.release_done,self.release_success,self.released
        if p and t.payload_healthy and self.release_sent and not p.present:
            t.released = True
        t.release_detail = self.release_detail
        return t

    def on_marker(self,msg):
        if not self.engine.active or msg.header.frame_id != 'map':
            return
        age = self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if not 0 <= age < .5:
            return
        p = msg.pose.position
        # Preserve acquisition stamp identity rather than adding receipt jitter to it.
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not hasattr(self,'marker_stamps'):
            self.marker_stamps = {}
        if stamp <= self.marker_stamps.get(msg.marker_id,-math.inf):
            return
        self.marker_stamps[msg.marker_id] = stamp
        self.engine.markers.feed(msg.marker_id,time.monotonic()-age,(p.x,p.y,p.z),msg.reprojection_error)

    def accept(self,request):
        route=tuple((p.x,p.y,p.z) for p in request.outbound_route)
        goal = Goal(request.latitude,request.longitude,request.relative_altitude,
                    request.delivery_marker_id,request.home_marker_id,route)
        if self.reserved or not self.enabled or not goal.valid() or not self.telemetry().healthy:
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self,handle):
        self.goal_handle,self.mission_id,self.sequence = handle,handle.goal_id,0
        self.offset = int.from_bytes(bytes(handle.goal_id.uuid)[:6],'big') << 16
        self.done = Future()
        self.release_sent = self.release_done = self.release_success = self.released = False
        self.release_detail = ''
        self.release_handle = None
        self.release_cancel_sent = False
        self.marker_stamps = {}
        r = handle.request
        route=tuple((p.x,p.y,p.z) for p in r.outbound_route)
        self.engine.start(Goal(r.latitude,r.longitude,r.relative_altitude,r.delivery_marker_id,r.home_marker_id,route),
                          time.monotonic(),self.telemetry())
        handle.execute()
        self.publish_status()

    async def execute(self,handle):
        result = await self.done
        if self.engine.execution == Execution.SUCCEEDED:
            handle.succeed()
        elif self.engine.execution == Execution.CANCELED and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        self.goal_handle = None
        self.reserved = False
        return result

    def cancel(self,handle):
        if handle == self.goal_handle and self.engine.active:
            self.engine.cancel()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def start_service(self,request,response):
        if self.reserved or not self.enabled or not self.client.server_is_ready():
            response.success,response.message = False,'Disabled, busy or action server unavailable'
            return response
        self.client.send_goal_async(ExecuteDelivery.Goal(**self.default_goal)).add_done_callback(self.started_service)
        response.success,response.message = True,'Goal submitted; monitor /delivery/mission_status for acceptance/result'
        return response

    def started_service(self,future):
        if not future.result().accepted:
            self.get_logger().warning('Configured mission goal rejected')

    def stop_service(self,request,response):
        if self.goal_handle is None or not self.engine.active:
            response.success,response.message = False,'No active goal'
        else:
            req = CancelGoal.Request()
            req.goal_info.goal_id = self.mission_id
            self.cancel_client.call_async(req)
            response.success,response.message = True,'Cancellation requested; monitor terminal action status'
        return response

    def publish_status(self):
        e = self.engine
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mission_id,msg.sequence = self.mission_id,self.sequence
        self.sequence += 1
        msg.execution_state,msg.phase = int(e.execution),int(e.phase)
        msg.progress,msg.payload_released,msg.error_code,msg.detail = float(e.progress),e.payload_released,int(e.error),e.detail
        elapsed = max(0.,(e.ended if e.ended is not None else time.monotonic())-e.started) if e.execution != Execution.IDLE else 0.
        msg.elapsed.sec,msg.elapsed.nanosec = divmod(int(elapsed*1e9),1000000000)
        self.pub.publish(msg)
        self.legacy.publish(String(data=Execution(msg.execution_state).name+':'+Phase(msg.phase).name))
        if self.goal_handle:
            self.goal_handle.publish_feedback(ExecuteDelivery.Feedback(status=msg))
        self.last_status = msg
        self.last_revision,self.last_publish = e.revision,time.monotonic()

    def request_release(self):
        self.release_sent = True
        if not self.release_client.server_is_ready():
            self.release_done = True
            self.release_detail = 'Release action server unavailable'
            return
        mission_id = bytes(self.mission_id.uuid)
        future = self.release_client.send_goal_async(ReleasePayload.Goal(mission_id=self.mission_id))
        future.add_done_callback(lambda f:self.release_accepted(f,mission_id))

    def release_accepted(self,future,mission_id):
        handle = future.result()
        if mission_id != bytes(self.mission_id.uuid):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.release_handle = handle
        if not handle.accepted:
            self.release_done = True
            self.release_detail = 'Release action rejected by payload interlock'
            return
        if not self.engine.active or self.engine.execution == Execution.CANCELING:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f:self.release_result(f,mission_id))

    def release_result(self,future,mission_id):
        if mission_id != bytes(self.mission_id.uuid):
            return
        result = future.result().result
        self.release_done,self.release_success,self.released = True,result.success,result.released
        self.release_detail = result.detail

    def tick(self):
        now = time.monotonic()
        self.engine.tick(now,self.telemetry())
        if self.engine.execution == Execution.RUNNING and self.engine.release_requested and not self.release_sent:
            self.request_release()
        if self.engine.execution == Execution.CANCELING and self.release_handle and not self.release_done:
            if not getattr(self,'release_cancel_sent',False):
                self.release_handle.cancel_goal_async()
                self.release_cancel_sent = True
        if self.reserved:
            intent = FlightIntent()
            intent.header.stamp = self.get_clock().now().to_msg()
            intent.header.frame_id = 'map'
            i = self.engine.intent
            intent.stream = i.stream
            intent.position.x,intent.position.y,intent.position.z = i.position
            intent.yaw_valid = False
            intent.max_speed = float(self.engine.cfg.approach_speed)
            intent.command_id,intent.command = self.offset+i.command_id,i.command
            self.intent_pub.publish(intent)
        if self.engine.revision != self.last_revision or (self.engine.active and now-self.last_publish >= .5):
            self.publish_status()
        if self.goal_handle and not self.engine.active and not self.done.done():
            if self.release_handle and self.release_handle.accepted and not self.release_done:
                self.release_handle.cancel_goal_async()
            self.done.set_result(ExecuteDelivery.Result(final_status=self.last_status))


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
