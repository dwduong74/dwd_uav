"""ROS facade coordinating three autonomous Table C deliveries."""
import math
import time
import uuid
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
from delivery_interfaces.action import RunCompetition,OperatePayload,ExecuteDelivery
from delivery_interfaces.msg import CompetitionStatus,FiducialObservation,MissionStatus
from delivery_interfaces.srv import CompetitionControl
from .competition_core import CompetitionEngine,CompetitionPhase
from .course import Course


class Competition(Node):
    def __init__(self):
        super().__init__('competition')
        self.enabled=self.declare_parameter('enable_control',False).value
        self.course_file=self.declare_parameter('course_file','').value
        self.default_course=self.declare_parameter('course_id','table_c_demo').value
        self.scan_timeout=float(self.declare_parameter('scan_timeout',30.).value)
        self.course=Course.load(self.course_file,self.default_course)
        self.engine=CompetitionEngine(self.course)
        self.reserved=False; self.goal_handle=None; self.done=None
        self.sequence=0; self.started=0.; self.last_status=CompetitionStatus()
        self.leg_status=MissionStatus(); self.leg_handle=None; self.payload_handle=None
        self.paused=False; self.scan_started=0.
        self.abort_detail=''
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(CompetitionStatus,'/competition/status',qos)
        self.hold_pub=self.create_publisher(Bool,'/delivery/safety_hold',1)
        self.create_subscription(FiducialObservation,'/delivery/fiducials',self.on_marker,10)
        self.create_subscription(MissionStatus,'/delivery/mission_status',self.on_leg_status,qos)
        self.payload_client=ActionClient(self,OperatePayload,'/delivery/operate_payload')
        self.leg_client=ActionClient(self,ExecuteDelivery,'/delivery/execute')
        self.server=ActionServer(self,RunCompetition,'/competition/run',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=self.cancel,handle_accepted_callback=self.accepted)
        self.client=ActionClient(self,RunCompetition,'/competition/run')
        self.create_service(Trigger,'/competition/start',self.start_service)
        self.create_service(CompetitionControl,'/competition/control',self.control)
        self.create_timer(.1,self.tick)
        self.publish()

    def accept(self,request):
        count=int(request.package_count)
        if self.reserved or not self.enabled or request.course_id!=self.course.course_id or count!=3:
            return GoalResponse.REJECT
        self.reserved=True
        return GoalResponse.ACCEPT

    def accepted(self,handle):
        self.goal_handle=handle; self.done=Future(); self.sequence=0
        self.started=self.scan_started=time.monotonic(); self.paused=False
        self.abort_detail=''
        self.engine.start(self.started,int(handle.request.package_count))
        handle.execute(); self.publish()

    async def execute(self,handle):
        result=await self.done
        if self.engine.phase==CompetitionPhase.FINISHED: handle.succeed()
        elif handle.is_cancel_requested: handle.canceled()
        else: handle.abort()
        self.goal_handle=None; self.reserved=False
        return result

    def cancel(self,handle):
        if handle!=self.goal_handle or not self.engine.active:
            return CancelResponse.REJECT
        self.abort('Competition action canceled')
        return CancelResponse.ACCEPT

    def on_marker(self,msg):
        if self.paused or msg.role!=FiducialObservation.PACKAGE:
            return
        if self.engine.marker(msg.marker_id,time.monotonic()):
            self.request_grab()
            self.publish()

    def on_leg_status(self,msg):
        self.leg_status=msg

    @staticmethod
    def operation_id():
        out=UUID(); out.uuid=list(uuid.uuid4().bytes); return out

    def request_grab(self):
        if not self.payload_client.server_is_ready():
            self.fail(21,'Payload action unavailable'); return
        goal=OperatePayload.Goal(operation_id=self.operation_id(),operation=OperatePayload.Goal.GRAB)
        self.payload_client.send_goal_async(goal).add_done_callback(self.grab_accepted)

    def grab_accepted(self,future):
        self.payload_handle=future.result()
        if not self.payload_handle.accepted:
            self.fail(21,'Payload grab rejected by interlock'); return
        self.payload_handle.get_result_async().add_done_callback(self.grab_result)

    def grab_result(self,future):
        wrapped=future.result(); result=wrapped.result
        self.payload_handle=None
        if self.abort_detail:
            self.fail(23,self.abort_detail); return
        ok=wrapped.status==GoalStatus.STATUS_SUCCEEDED and result.success and result.present and result.stowed
        if not self.engine.grab_result(ok,result.detail):
            self.finish(); return
        self.request_leg(); self.publish()

    def request_leg(self):
        if not self.leg_client.server_is_ready():
            self.fail(22,'Delivery action unavailable'); return
        task=self.course.packages[self.engine.index]
        destination=self.course.destinations[task.destination]
        route=[Point(x=self.course.nodes[n][0],y=self.course.nodes[n][1],z=self.course.nodes[n][2])
               for n in destination.route]
        goal=ExecuteDelivery.Goal(latitude=destination.latitude,longitude=destination.longitude,
            relative_altitude=destination.altitude,delivery_marker_id=destination.marker_id,
            home_marker_id=self.course.home_marker_id,outbound_route=route)
        self.leg_client.send_goal_async(goal,feedback_callback=self.leg_feedback).add_done_callback(self.leg_accepted)

    def leg_feedback(self,msg):
        self.leg_status=msg.feedback.status; self.publish()

    def leg_accepted(self,future):
        self.leg_handle=future.result()
        if not self.leg_handle.accepted:
            self.fail(22,'Delivery action rejected'); return
        self.leg_handle.get_result_async().add_done_callback(self.leg_result)

    def leg_result(self,future):
        wrapped=future.result(); detail=wrapped.result.final_status.detail
        if self.abort_detail:
            self.engine.fail(23,self.abort_detail,time.monotonic())
            self.leg_handle=None; self.finish(); return
        ok=wrapped.status==GoalStatus.STATUS_SUCCEEDED
        self.engine.leg_result(ok,time.monotonic(),detail)
        self.leg_handle=None
        if self.engine.phase==CompetitionPhase.SCAN_PACKAGE:
            self.scan_started=time.monotonic()
        if not self.engine.active: self.finish()
        else: self.publish()

    def abort(self,detail):
        self.hold_pub.publish(Bool(data=False))
        self.abort_detail=detail
        if self.leg_handle:
            self.leg_handle.cancel_goal_async(); return
        if self.payload_handle:
            self.payload_handle.cancel_goal_async(); return
        self.fail(23,detail)

    def fail(self,code,detail):
        self.engine.fail(code,detail,time.monotonic()); self.finish()

    def finish(self):
        self.publish()
        if self.done is not None and not self.done.done():
            self.done.set_result(RunCompetition.Result(final_status=self.last_status))

    def control(self,request,response):
        if not self.engine.active:
            response.accepted,response.message=False,'No active competition run'; return response
        if request.command==CompetitionControl.Request.PAUSE:
            self.paused=True; self.hold_pub.publish(Bool(data=True)); response.message='Position hold requested'
        elif request.command==CompetitionControl.Request.RESUME:
            self.paused=False; self.hold_pub.publish(Bool(data=False)); response.message='Autonomy resumed'
        elif request.command in (CompetitionControl.Request.RTL,CompetitionControl.Request.LAND,
                                 CompetitionControl.Request.ABORT_SAFE):
            self.abort('Operator requested safe abort'); response.message='Safe cancellation requested'
        else:
            response.accepted,response.message=False,'Unknown control command'; return response
        response.accepted=True; self.publish(); return response

    def start_service(self,request,response):
        if self.reserved or not self.enabled or not self.client.server_is_ready():
            response.success,response.message=False,'Disabled, busy or action unavailable'; return response
        goal=RunCompetition.Goal(course_id=self.course.course_id,package_count=3)
        self.client.send_goal_async(goal)
        response.success,response.message=True,'Three-package run submitted; monitor /competition/status'
        return response

    def tick(self):
        if (self.engine.phase==CompetitionPhase.SCAN_PACKAGE and not self.paused
                and time.monotonic()-self.scan_started>self.scan_timeout):
            self.fail(20,'Package marker scan timeout')
        elif self.engine.active:
            self.publish()

    def publish(self):
        e=self.engine; msg=CompetitionStatus()
        msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='map'
        if self.goal_handle: msg.run_id=self.goal_handle.goal_id
        msg.sequence=self.sequence; self.sequence+=1
        msg.state=int(CompetitionPhase.HOLD if self.paused and e.active else e.phase)
        msg.package_index=e.index; msg.package_count=len(self.course.packages)
        msg.package_marker_id=e.package_marker; msg.destination_marker_id=e.destination_marker
        msg.completed_deliveries=e.completed; msg.progress=float(e.progress)
        elapsed=max(0.,time.monotonic()-self.started) if self.started else 0.
        msg.elapsed.sec,msg.elapsed.nanosec=divmod(int(elapsed*1e9),1000000000)
        msg.error_code=e.error; msg.detail=e.detail; msg.leg_status=self.leg_status
        self.last_status=msg; self.pub.publish(msg)
        if self.goal_handle: self.goal_handle.publish_feedback(RunCompetition.Feedback(status=msg))


def main(args=None):
    rclpy.init(args=args); node=Competition()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
