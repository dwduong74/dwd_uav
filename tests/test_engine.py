import math
import unittest
from dataclasses import replace
from delivery_ros.engine import Engine, Goal, Telemetry, Config, Phase, Execution, Error, MarkerGate
from delivery_ros.geometry import enu_ned, flu_frd, project_enu, rotate, px4_attitude_to_ros
from delivery_ros.command import CommandTracker


def ready():
    return Telemetry(healthy=True, landed=True, battery=.9, latitude=10.,longitude=106.,
        ref_latitude=10.,ref_longitude=106., reset=(1,0,0,0), payload_healthy=True,
        payload_present=True,payload_closed=True,range_valid=True,camera_ready=True,payload_ready=True)


class Simulation:
    """Deterministic kinematic double, explicitly NOT PX4 SITL."""
    def __init__(self):
        self.e,self.t,self.now = Engine(Config(radius=1.,spacing=1.)),ready(),0.
        self.e.start(Goal(10.00002,106.,2.,0,1),0.,self.t)
        self.release_count = 0
        self.phases = set()

    def step(self, markers=True):
        self.now += .05
        e,t = self.e,self.t
        i = e.intent
        if i.command and i.command_id != t.command_id:
            t.command_id,t.command_state = i.command_id,2
            if i.command == 1:
                t.offboard,t.auto_land = True,False
            elif i.command == 2:
                t.armed = True
            elif i.command == 3:
                t.auto_land,t.offboard = True,False
        if i.stream and t.offboard and t.armed:
            t.position = i.position
            t.agl = max(0.,t.position[2])
            t.landed = t.agl <= .01
        if t.auto_land:
            t.position = (*t.position[:2],max(0.,t.position[2]-.05))
            t.agl = t.position[2]
            if t.agl == 0:
                t.landed,t.armed = True,False
        if markers and e.phase in (Phase.SEARCH,Phase.APPROACH,Phase.DESCEND):
            position = (*e.home[:2],0.) if e.returning else (*e.destination[:2],0.)
            e.markers.feed(1 if e.returning else 0,self.now,position)
        if e.release_requested and not t.release_done:
            self.release_count += 1
            t.release_done,t.release_success,t.released = True,True,True
            t.payload_present = False
        self.phases.add(e.phase)
        e.tick(self.now,t)

    def until(self, phase):
        for _ in range(10000):
            if self.e.phase == phase or not self.e.active:
                break
            self.step()
        if self.e.phase != phase:
            raise AssertionError((self.e.phase,self.e.detail,self.now))


class EngineTests(unittest.TestCase):
    def test_twenty_round_trips(self):
        for _ in range(20):
            s = Simulation()
            for _ in range(10000):
                if not s.e.active:
                    break
                s.step()
            self.assertEqual(s.e.execution,Execution.SUCCEEDED,(s.e.phase,s.e.detail,s.now))
            self.assertTrue(s.e.payload_released)
            self.assertEqual(s.release_count,1)
            self.assertEqual(s.e.progress,1.)
            self.assertIn(Phase.RETURN_PREFLIGHT,s.phases)

    def test_invalid_goal(self):
        for goal in (Goal(math.nan,0,5,0,1),Goal(10,106,0,0,1),Goal(10,106,5,1,1)):
            self.assertFalse(Engine().start(goal,0,ready()))

    def test_reject_simultaneous_start(self):
        s = Simulation()
        self.assertFalse(s.e.start(Goal(10,106,5,0,1),1,ready()))

    def test_cancel_on_ground_never_arms_or_releases(self):
        s = Simulation()
        s.e.cancel()
        s.step()
        self.assertEqual(s.e.execution,Execution.CANCELED)
        self.assertEqual(s.e.intent.command,0)
        self.assertEqual(s.release_count,0)

    def test_cancel_airborne_returns(self):
        s = Simulation()
        s.until(Phase.TRANSIT)
        s.e.cancel()
        s.step()
        self.assertEqual(s.e.execution,Execution.CANCELING)
        for _ in range(5000):
            if not s.e.active: break
            s.step()
        self.assertEqual(s.e.execution,Execution.CANCELED,s.e.detail)
        self.assertEqual(s.release_count,0)

    def test_no_release_without_disarm(self):
        s = Simulation()
        s.until(Phase.LAND)
        s.t.armed,s.t.landed = True,True
        for n in range(100):
            s.e.tick(s.now+n*.05,s.t)
        self.assertFalse(s.e.release_requested)

    def test_payload_failure_stays_grounded(self):
        s = Simulation()
        s.until(Phase.RELEASE)
        s.t.release_done,s.t.release_success = True,False
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.error,Error.PAYLOAD_FAILED)
        self.assertFalse(s.e.intent.stream)

    def test_stale_marker_stops_descent(self):
        s = Simulation()
        s.until(Phase.DESCEND)
        for _ in range(15): s.step(markers=False)
        height = s.e.intent.position[2]
        for _ in range(10): s.step(markers=False)
        self.assertEqual(s.e.intent.position[2],height)
        for _ in range(50): s.step(markers=False)
        self.assertEqual(s.e.error,Error.MARKER_LOST)

    def test_telemetry_reset_takeover(self):
        for mutate,error in ((lambda t:setattr(t,'healthy',False),Error.TELEMETRY_LOST),
                             (lambda t:setattr(t,'reset',(2,0,0,0)),Error.ESTIMATOR_RESET),
                             (lambda t:setattr(t,'offboard',False),Error.OPERATOR_TAKEOVER)):
            s=Simulation(); s.until(Phase.TRANSIT); mutate(s.t)
            s.e.tick(s.now+.05,s.t)
            self.assertEqual(s.e.error,error)
            self.assertFalse(s.e.intent.stream)

    def test_first_takeoff_accepts_only_expected_yaw_alignment_reset(self):
        s=Simulation(); s.until(Phase.TAKEOFF)
        s.t.reset=(1,0,0,1)
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.execution,Execution.RUNNING)
        self.assertEqual(s.e.origin_reset,(1,0,0,1))
        s.t.reset=(1,0,0,2)
        s.e.tick(s.now+.075,s.t)
        self.assertEqual(s.e.error,Error.ESTIMATOR_RESET)

    def test_transit_accepts_late_first_yaw_alignment_reset(self):
        s=Simulation(); s.until(Phase.TRANSIT)
        s.t.reset=(1,0,0,1)
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.execution,Execution.RUNNING)
        self.assertTrue(s.e.yaw_alignment_reset_accepted)
        s.t.reset=(1,0,1,1)
        s.e.tick(s.now+.1,s.t)
        self.assertEqual(s.e.error,Error.ESTIMATOR_RESET)

    def test_release_result_survives_return_failure(self):
        s=Simulation(); s.until(Phase.RETURN_PREFLIGHT)
        s.t.healthy=False
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.execution,Execution.FAILED)
        self.assertTrue(s.e.payload_released)

    def test_home_not_changed_after_second_arm(self):
        s=Simulation(); s.until(Phase.RETURN_PREFLIGHT)
        home=s.e.home
        s.t.latitude,s.t.longitude=11.,107.
        s.step()
        self.assertEqual(s.e.home,home)
        self.assertEqual(s.e.home_gps,(10.,106.))

    def test_camera_and_range_are_preflight_requirements(self):
        for field in ('camera_ready','range_valid'):
            t=ready(); setattr(t,field,False)
            e=Engine(); e.start(Goal(10,106,2,0,1),0,t)
            self.assertEqual(e.error,Error.PREFLIGHT_FAILED)
            self.assertIn('camera/TF' if field=='camera_ready' else 'range invalid',e.detail)
            self.assertEqual(e.progress,0.)

    def test_preflight_countdown_reports_lost_readiness(self):
        for field, reason in [('camera_ready', 'camera/TF'), ('range_valid', 'range invalid'),
                              ('payload_healthy', 'payload unhealthy'), ('payload_closed', 'gripper')]:
            t = ready()
            e = Engine()
            e.start(Goal(10,106,2,0,1),0,t)
            setattr(t, field, False)
            e.tick(.1, t)
            self.assertEqual(e.error, Error.PREFLIGHT_FAILED)
            self.assertIn(reason, e.detail)
            self.assertEqual(e.intent.command, 0)

    def test_cancel_pending_arm_does_not_complete_early(self):
        s=Simulation()
        s.t.offboard=True
        s.e.intent.command=2; s.e.intent.command_id=2; s.e.substage=2
        s.e.cancel(); s.e.tick(.1,s.t)
        self.assertEqual(s.e.execution,Execution.CANCELING)
        self.assertEqual(s.e.phase,Phase.LAND)
        self.assertEqual(s.e.intent.command,3)

    def test_cancel_during_release_waits_for_result(self):
        s=Simulation(); s.until(Phase.RELEASE)
        s.e.release_requested=True
        s.e.cancel(); s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.execution,Execution.CANCELING)
        s.t.release_done,s.t.released=True,True
        s.e.tick(s.now+.1,s.t)
        self.assertEqual(s.e.execution,Execution.CANCELED)
        self.assertTrue(s.e.payload_released)

    def test_low_battery_during_land_still_sends_land(self):
        s=Simulation(); s.until(Phase.LAND)
        s.t.battery=.1
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.intent.command,3)
        self.assertEqual(s.e.error,Error.BATTERY_LOW)

    def test_range_loss_returns_without_descent(self):
        s=Simulation(); s.until(Phase.DESCEND)
        s.t.range_valid=False
        s.e.tick(s.now+.05,s.t)
        self.assertEqual(s.e.error,Error.RANGE_LOST)
        self.assertEqual(s.e.phase,Phase.RETURN_TAKEOFF)


class MarkerTests(unittest.TestCase):
    def test_duplicate_and_out_of_order_not_confirmation(self):
        gate=MarkerGate()
        for _ in range(10): gate.feed(1,1.,(0,0,0))
        gate.feed(1,.9,(0,0,0))
        self.assertIsNone(gate.get(1,1.1))
        for i in range(1,5): gate.feed(1,1+i*.05,(0,0,0))
        self.assertIsNotNone(gate.get(1,1.3))
        self.assertIsNone(gate.get(1,2.))
        self.assertIsNone(gate.get(0,1.3))


class GeometryTests(unittest.TestCase):
    def test_axes_and_yaws(self):
        self.assertEqual(enu_ned(enu_ned((1,2,3))),(1,2,3))
        self.assertEqual(flu_frd((1,2,3)),(1,-2,-3))
        for yaw in (0,math.pi/2,math.pi):
            q=px4_attitude_to_ros((math.cos(yaw/2),0,0,math.sin(yaw/2)))
            v=rotate(q,(1,0,0))
            self.assertAlmostEqual(v[0],math.sin(yaw))
            self.assertAlmostEqual(v[1],math.cos(yaw))
        self.assertEqual(project_enu(10,106,10,106),(0.,0.))
        e,n=project_enu(10.001,106.001,10,106)
        self.assertGreater(e,100); self.assertGreater(n,100)


class CommandTests(unittest.TestCase):
    def test_ack_needs_state_and_stale_ack_ignored(self):
        c=CommandTracker()
        c.begin(1,1,176,0,1000)
        c.ack(176,0,999); c.tick(1,True)
        self.assertEqual(c.state,1)
        c.ack(176,0,1001); c.tick(2,False)
        self.assertEqual(c.state,1)
        c.tick(3,True)
        self.assertEqual(c.state,2)

    def test_timeout_rejection_and_dedup(self):
        c=CommandTracker()
        self.assertTrue(c.begin(1,2,400,0,1000))
        self.assertFalse(c.begin(1,2,400,1,1001))
        c.tick(5,False)
        self.assertEqual(c.state,3)
        c.begin(2,2,400,6,2000); c.ack(400,2,2001)
        self.assertEqual(c.state,3)
