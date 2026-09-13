"""Real ROS transport/actions with a kinematic flight double (not PX4 SITL)."""
import time
import unittest
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from std_msgs.msg import Bool
    from delivery_interfaces.action import ExecuteDelivery
    from delivery_interfaces.msg import FlightIntent,FlightState,LandingTarget,MissionStatus
    from delivery_ros.mission import Mission
    from delivery_ros.payload import Payload
    from delivery_ros.engine import Phase
    ROS=True
except ImportError as exc:
    if exc.name != 'rclpy':
        raise
    ROS=False


@unittest.skipUnless(ROS,'Requires built ROS 2 Humble interfaces')
class ActionTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','enable_control:=true','-p','backend:=sim'])
        self.mission=Mission()
        self.payload=Payload()
        self.node=Node('action_test_client')
        self.executor=SingleThreadedExecutor()
        for node in (self.node,self.mission,self.payload): self.executor.add_node(node)
        self.pub=self.node.create_publisher(FlightState,'/delivery/flight_state',10)
        self.marker_pub=self.node.create_publisher(LandingTarget,'/delivery/landing_target',1)
        self.vision_pub=self.node.create_publisher(Bool,'/delivery/vision_ready',1)
        self.intent=None
        self.node.create_subscription(FlightIntent,'/delivery/flight_intent',lambda m:setattr(self,'intent',m),1)
        self.state=FlightState(healthy=True,landed=True,battery_remaining=.9,range_valid=True,
                               latitude=10.,longitude=106.,ref_latitude=10.,ref_longitude=106.,reference_timestamp=1)
        self.timer=self.node.create_timer(.05,self.plant)
        self.client=ActionClient(self.node,ExecuteDelivery,'/delivery/execute')
        self.feedback=[]
        self.statuses=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(MissionStatus,'/delivery/mission_status',self.statuses.append,qos)
        self.spin_for(2.3)

    def tearDown(self):
        self.executor.shutdown()
        for node in (self.node,self.mission,self.payload): node.destroy_node()
        rclpy.shutdown()

    def spin_for(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end: self.executor.spin_once(timeout_sec=.01)

    def wait(self,future,timeout=10):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end: self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(future.done(),'Future timeout')
        return future.result()

    def plant(self):
        s,i=self.state,self.intent
        if i:
            if i.command and i.command_id!=s.command_id:
                s.command_id,s.command_state=i.command_id,2
                if i.command==1: s.offboard,s.auto_land=True,False
                if i.command==2: s.armed=True
                if i.command==3: s.auto_land,s.offboard=True,False
            if i.stream and s.offboard and s.armed:
                s.position=i.position
                s.agl=max(0.,s.position.z)
                s.landed=s.agl<=.01
            if s.auto_land:
                s.position.z=max(0.,s.position.z-.1)
                s.agl=s.position.z
                if s.agl==0: s.landed,s.armed=True,False
        s.header.stamp=self.node.get_clock().now().to_msg()
        self.pub.publish(s)
        self.vision_pub.publish(Bool(data=True))
        e=self.mission.engine
        if e.active and e.phase in (Phase.SEARCH,Phase.APPROACH,Phase.DESCEND):
            m=LandingTarget()
            m.header.stamp=s.header.stamp; m.header.frame_id='map'
            m.marker_id=1 if e.returning else 0
            target=e.home if e.returning else e.destination
            m.pose.position.x,m.pose.position.y=target[:2]
            m.pose.orientation.w=1.
            self.marker_pub.publish(m)

    def send(self):
        self.assertTrue(self.client.wait_for_server(timeout_sec=1))
        return self.wait(self.client.send_goal_async(ExecuteDelivery.Goal(latitude=10.,longitude=106.,
            relative_altitude=1.,delivery_marker_id=0,home_marker_id=1),
            feedback_callback=lambda m:self.feedback.append(m.feedback.status)))

    def test_feedback_cancel_busy_and_late_subscriber(self):
        handle=self.send()
        self.assertTrue(handle.accepted)
        other=self.send()
        self.assertFalse(other.accepted)
        self.spin_for(.7)
        self.assertGreaterEqual(len(self.feedback),2)
        self.wait(handle.cancel_goal_async())
        result=self.wait(handle.get_result_async()).result.final_status
        self.assertEqual(result.execution_state,MissionStatus.CANCELED)
        self.assertFalse(result.payload_released)
        self.spin_for(.1)
        snapshots={m.sequence:m for m in self.statuses if m.mission_id==handle.goal_id}
        for feedback in self.feedback:
            self.assertEqual(feedback.mission_id,handle.goal_id)
            if feedback.sequence in snapshots:
                self.assertEqual(feedback,snapshots[feedback.sequence])
        self.assertEqual(result,self.statuses[-1])
        late=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        sub=self.node.create_subscription(MissionStatus,'/delivery/mission_status',late.append,qos)
        self.spin_for(.3)
        self.assertTrue(late)
        self.assertEqual(late[-1],result)
        self.node.destroy_subscription(sub)

    def test_round_trip_with_payload_action(self):
        self.mission.engine.cfg.descend_speed=.5
        handle=self.send()
        self.assertTrue(handle.accepted)
        result=self.wait(handle.get_result_async(),timeout=60).result.final_status
        self.assertEqual(result.execution_state,MissionStatus.SUCCEEDED,result.detail)
        self.assertTrue(result.payload_released)
        self.assertEqual(result.progress,1.)
        self.assertEqual(self.payload.backend.count,1)
        self.assertEqual(result.mission_id,handle.goal_id)
        seq=[s.sequence for s in self.feedback]
        self.assertEqual(seq,sorted(set(seq)))
