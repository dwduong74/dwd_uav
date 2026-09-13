import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from delivery_interfaces.msg import FlightState,PayloadState
from delivery_interfaces.action import ReleasePayload
from .payload_protocol import SimBackend,SerialBackend


class Payload(Node):
    def __init__(self):
        super().__init__('payload')
        kind=self.declare_parameter('backend','serial').value
        port=self.declare_parameter('port','/dev/serial/by-id/CONFIGURE_PAYLOAD').value
        if kind not in ('serial','sim'):
            raise ValueError('backend must be serial or sim')
        self.backend=SimBackend() if kind=='sim' else SerialBackend(port)
        self.goal_handle=None
        self.reserved=False
        self.done=None
        self.flight=None
        self.flight_time=-1e9
        self.ground_since=None
        self.healthy=False
        self.cache={}
        self.boot=''
        self.started=0.
        self.cancelled=False
        self.error=''
        self.pub=self.create_publisher(PayloadState,'/delivery/payload_state',10)
        self.create_subscription(FlightState,'/delivery/flight_state',self.on_flight,10)
        self.server=ActionServer(self,ReleasePayload,'/delivery/release_payload',
            execute_callback=self.execute,goal_callback=self.accept,cancel_callback=self.cancel,
            handle_accepted_callback=self.accepted)
        self.create_timer(.1,self.tick)

    def on_flight(self,msg):
        self.flight,self.flight_time=msg,time.monotonic()

    def accept(self,request):
        token=bytes(request.mission_id.uuid).hex()
        if (self.reserved or not any(request.mission_id.uuid) or not self.healthy
                or self.ground_since is None or time.monotonic()-self.ground_since<2.):
            return GoalResponse.REJECT
        self.reserved=True
        return GoalResponse.ACCEPT

    def accepted(self,handle):
        self.goal_handle,self.done,self.cancelled=handle,Future(),False
        self.token=bytes(handle.request.mission_id.uuid).hex()
        self.started=time.monotonic()
        self.boot=self.backend.reading.boot
        self.error=''
        handle.execute()
        if self.token in self.cache:
            self.done.set_result(self.cache[self.token])
            return
        r=self.backend.reading
        if r.last_id==self.token:
            # Never replay on host restart; report existing sensor state only.
            self.complete(not r.present and r.closed and not r.busy,'Previously attempted token')
            return
        try:
            self.backend.release(self.token,self.started)
        except Exception as exc:
            self.complete(False,str(exc))

    async def execute(self,handle):
        result=await self.done
        if result.success:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        self.goal_handle=None
        self.reserved=False
        return result

    def cancel(self,handle):
        if handle==self.goal_handle:
            self.cancelled=True
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def complete(self,success,detail):
        if self.done.done():
            return
        r=self.backend.reading
        result=ReleasePayload.Result(success=success,released=not r.present,
                                     stowed=r.closed,detail=detail)
        self.cache[self.token]=result
        self.done.set_result(result)

    def tick(self):
        now=time.monotonic()
        f=self.flight
        ground=bool(f and f.healthy and now-self.flight_time<.3 and f.landed and not f.armed)
        self.ground_since=(self.ground_since if self.ground_since is not None else now) if ground else None
        try:
            r=self.backend.update(now)
            self.healthy=not r.fault
            self.error=''
        except Exception as exc:
            r=self.backend.reading
            self.healthy=False
            self.error=str(exc)
        out=PayloadState()
        out.header.stamp=self.get_clock().now().to_msg()
        out.healthy,out.closed,out.present,out.busy,out.fault=self.healthy,r.closed,r.present,r.busy,r.fault
        out.detail=self.error
        out.release_ready=bool(self.healthy and self.ground_since is not None and now-self.ground_since>=2.
                               and r.closed and r.present and not r.busy)
        self.pub.publish(out)
        if self.goal_handle and not self.done.done():
            if self.cancelled or not ground or not self.healthy or r.boot!=self.boot or now-self.started>10.:
                try:
                    self.backend.stop()
                except Exception:
                    pass
                self.complete(False,'Canceled/interlock/connection/timeout')
            elif r.last_id==self.token and not r.busy and not r.present and r.closed:
                self.complete(True,'Release and stow confirmed by sensors')
            else:
                self.goal_handle.publish_feedback(ReleasePayload.Feedback(state='Waiting for release and stow sensors'))


def main(args=None):
    rclpy.init(args=args)
    node=Payload()
    try:
        rclpy.spin(node)
    finally:
        if node.goal_handle:
            node.backend.stop()
        node.destroy_node()
        rclpy.shutdown()
