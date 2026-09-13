"""Deterministic mission engine. No ROS, threads, IO or wall clock calls."""
from dataclasses import dataclass
from enum import IntEnum
import math
from .geometry import project_enu, step_toward
from .core import search_path, distance


class Execution(IntEnum):
    IDLE=0; RUNNING=1; CANCELING=2; SUCCEEDED=3; CANCELED=4; FAILED=5


class Phase(IntEnum):
    IDLE=0; PREFLIGHT=1; TAKEOFF=2; TRANSIT=3; SEARCH=4; APPROACH=5
    DESCEND=6; LAND=7; RELEASE=8; RETURN_PREFLIGHT=9; RETURN_TAKEOFF=10
    RETURN_TRANSIT=11; RETURN_LAND=12; FINISHED=13


class Error(IntEnum):
    NONE=0; INVALID_GOAL=1; TELEMETRY_LOST=2; ESTIMATOR_RESET=3; COMMAND_FAILED=4
    MARKER_LOST=5; SEARCH_EXHAUSTED=6; PAYLOAD_FAILED=7; OPERATOR_TAKEOVER=8
    TIMEOUT=9; BATTERY_LOW=10; PREFLIGHT_FAILED=11; RANGE_LOST=12


@dataclass
class Goal:
    latitude: float
    longitude: float
    relative_altitude: float
    delivery_marker_id: int
    home_marker_id: int

    def valid(self):
        return (all(math.isfinite(v) for v in (self.latitude, self.longitude, self.relative_altitude))
                and abs(self.latitude) < 85 and abs(self.longitude) <= 180
                and 1 <= self.relative_altitude <= 30
                and 0 <= self.delivery_marker_id < 250 and 0 <= self.home_marker_id < 250
                and self.delivery_marker_id != self.home_marker_id)


@dataclass
class Telemetry:
    healthy: bool = False
    position: tuple = (0.,0.,0.)
    latitude: float = 0.
    longitude: float = 0.
    ref_latitude: float = 0.
    ref_longitude: float = 0.
    reset: tuple = ()
    armed: bool = False
    landed: bool = False
    offboard: bool = False
    auto_land: bool = False
    failsafe: bool = False
    range_valid: bool = False
    agl: float = 0.
    battery: float = 0.
    command_id: int = 0
    command_state: int = 0
    payload_healthy: bool = False
    payload_present: bool = False
    payload_closed: bool = False
    release_done: bool = False
    release_success: bool = False
    released: bool = False
    camera_ready: bool = False
    payload_ready: bool = False
    release_detail: str = ''


@dataclass
class Intent:
    stream: bool = False
    position: tuple = (0.,0.,0.)
    command_id: int = 0
    command: int = 0  # NONE, OFFBOARD, ARM, LAND


@dataclass
class Config:
    radius: float = 5.
    spacing: float = 2.
    waypoint_tolerance: float = .5
    approach_speed: float = .3
    descend_speed: float = .15
    center_tolerance: float = .15
    handoff_agl: float = .3
    mission_timeout: float = 600.
    phase_timeout: float = 120.
    recovery_timeout: float = 180.
    min_battery: float = .3
    max_distance: float = 200.

    def validate(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Configuration values must be finite and positive')
        if self.min_battery >= 1 or self.handoff_agl > 1:
            raise ValueError('Invalid battery or handoff threshold')
        search_path(0,0,0,self.radius,self.spacing)


class MarkerGate:
    def __init__(self):
        self.samples = {}

    def feed(self, marker_id, stamp, position, quality=0.):
        if not all(math.isfinite(v) for v in (*position, stamp, quality)) or quality > 3 or quality < 0:
            return
        old = self.samples.get(marker_id)
        if old and stamp <= old[0]:
            return
        count = old[2]+1 if old and stamp-old[0] <= .5 and distance(position,old[1]) <= .3 else 1
        self.samples[marker_id] = (stamp, tuple(position), count)

    def get(self, marker_id, now):
        sample = self.samples.get(marker_id)
        return sample if sample and 0 <= now-sample[0] <= .5 and sample[2] >= 5 else None


class Engine:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.cfg.validate()
        self.execution = Execution.IDLE
        self.phase = Phase.IDLE
        self.error = Error.NONE
        self.detail = 'Ready'
        self.progress = 0.
        self.payload_released = False
        self.intent = Intent()
        self.revision = 0
        self.started = self.phase_started = self.last_tick = 0.
        self.ended = None
        self.markers = MarkerGate()
        self.cancel_requested = False
        self.returning = False
        self.recovery_started = None
        self.ground_since = None
        self.center_since = None
        self.last_marker = None
        self.substage = 0
        self.path = []
        self.path_index = 0
        self.release_requested = False

    @property
    def active(self):
        return self.execution in (Execution.RUNNING, Execution.CANCELING)

    def change(self, phase, now, detail=''):
        self.phase, self.phase_started = phase, now
        self.substage = 0
        self.center_since = None
        self.detail = detail or phase.name
        self.progress = max(self.progress, min(.95, (int(phase)-1)/13.))
        self.revision += 1

    def finish(self, execution, now, error=None, detail=None):
        if error is not None:
            self.error = error
        self.execution = execution
        self.ended = now
        self.intent.stream = False
        self.intent.command = 0
        progress = self.progress
        self.change(Phase.FINISHED, now, detail or self.error.name)
        self.progress = progress
        if execution == Execution.SUCCEEDED:
            self.progress = 1.

    def start(self, goal, now, t):
        if self.active or not goal.valid():
            return False
        self.__init__(self.cfg)
        self.goal = goal
        self.started = self.last_tick = now
        self.execution = Execution.RUNNING
        self.origin_reset = t.reset
        self.home = t.position
        self.home_gps = (t.latitude, t.longitude)
        self.intent.position = t.position
        self.cruise_z = t.position[2]+goal.relative_altitude
        self.destination = (*project_enu(goal.latitude, goal.longitude, t.ref_latitude, t.ref_longitude), self.cruise_z)
        self.change(Phase.PREFLIGHT, now)
        if (not t.healthy or not t.camera_ready or not t.range_valid or t.armed or not t.landed or not t.payload_healthy
                or not t.payload_present or not t.payload_closed or t.battery < self.cfg.min_battery
                or distance(self.destination[:2], self.home[:2]) > self.cfg.max_distance):
            self.finish(Execution.FAILED, now, Error.PREFLIGHT_FAILED)
        return True

    def cancel(self):
        if self.active:
            self.cancel_requested = True

    def command(self, kind):
        self.intent.command_id += 1
        self.intent.command = kind

    def acked(self, t):
        return t.command_id == self.intent.command_id and t.command_state == 2

    def recover(self, error, now, t):
        if self.recovery_started is not None:
            self.finish(Execution.FAILED, now, error, 'Recovery failed: '+error.name)
            return
        self.error = error
        self.recovery_started = now
        self.returning = True
        self.intent.position = t.position
        self.change(Phase.RETURN_TAKEOFF, now, 'Climb for return: '+error.name)

    def tick(self, now, t):
        if not self.active:
            return
        dt = min(.1, max(0., now-self.last_tick))
        self.last_tick = now
        self.payload_released |= t.released
        if not t.healthy:
            self.finish(Execution.FAILED, now, Error.TELEMETRY_LOST)
            return
        if t.reset != self.origin_reset:
            self.finish(Execution.FAILED, now, Error.ESTIMATOR_RESET)
            return
        if t.failsafe:
            self.finish(Execution.FAILED, now, Error.OPERATOR_TAKEOVER, 'PX4 failsafe active')
            return
        if t.command_id == self.intent.command_id and t.command_state == 3 and self.intent.command:
            self.finish(Execution.FAILED, now, Error.COMMAND_FAILED)
            return
        ground = t.landed and not t.armed
        self.ground_since = (self.ground_since if self.ground_since is not None else now) if ground else None
        grounded = self.ground_since is not None and now-self.ground_since >= 2.
        landing = self.phase in (Phase.LAND, Phase.RETURN_LAND)
        preflight = self.phase in (Phase.PREFLIGHT, Phase.RETURN_PREFLIGHT)
        if not preflight and self.phase != Phase.RELEASE and not landing and not t.offboard:
            self.finish(Execution.FAILED, now, Error.OPERATOR_TAKEOVER)
            return
        if not preflight and not landing and self.phase != Phase.RELEASE and not t.armed:
            self.finish(Execution.FAILED, now, Error.OPERATOR_TAKEOVER, 'Unexpected disarm')
            return
        if landing and not (t.offboard or t.auto_land or ground):
            self.finish(Execution.FAILED, now, Error.OPERATOR_TAKEOVER)
            return
        if self.cancel_requested and self.execution != Execution.CANCELING:
            self.execution = Execution.CANCELING
            self.revision += 1
            if ground and self.phase == Phase.RELEASE and self.release_requested and not t.release_done:
                # Wait for payload cancellation/result so final released flag reflects sensors.
                self.detail = 'Canceling payload operation'
            elif ground and preflight and self.intent.command == 2:
                # An ARM may already be in flight. LAND and observe disarm before concluding.
                self.intent.position = t.position
                self.change(Phase.LAND, now, 'Cancel pending arm with landing')
                landing = True
            elif ground:
                self.finish(Execution.CANCELED, now, detail='Canceled on ground')
                return
            elif landing:
                self.recovery_started = now
            elif not t.offboard:
                # No Offboard authority yet: abort rather than taking control from the pilot.
                self.finish(Execution.FAILED, now, Error.OPERATOR_TAKEOVER)
                return
            else:
                self.recover(Error.NONE, now, t)
        if self.recovery_started is not None and now-self.recovery_started > self.cfg.recovery_timeout:
            self.finish(Execution.FAILED, now, Error.TIMEOUT, 'Recovery timeout')
            return
        landing = self.phase in (Phase.LAND, Phase.RETURN_LAND)
        preflight = self.phase in (Phase.PREFLIGHT, Phase.RETURN_PREFLIGHT)
        if now-self.phase_started > self.cfg.phase_timeout:
            if landing or preflight or ground:
                self.finish(Execution.FAILED, now, Error.TIMEOUT)
            else:
                self.recover(Error.TIMEOUT, now, t)
            return
        if (now-self.started > self.cfg.mission_timeout or t.battery < self.cfg.min_battery) and self.recovery_started is None:
            error = Error.BATTERY_LOW if t.battery < self.cfg.min_battery else Error.TIMEOUT
            if ground:
                self.finish(Execution.FAILED, now, error)
                return
            elif landing:
                if self.error != error:
                    self.error,self.detail = error,'Complete landing: '+error.name
                    self.revision += 1
            elif not preflight or t.offboard:
                self.recover(error, now, t)
                return
            else:
                self.finish(Execution.FAILED, now, error)
                return
        if preflight:
            self._preflight(now, t)
        elif self.phase in (Phase.TAKEOFF, Phase.RETURN_TAKEOFF):
            self.intent.stream = True
            target = (self.intent.position[0], self.intent.position[1], self.cruise_z)
            self.intent.position = step_toward(self.intent.position, target, dt)
            if distance(t.position, target) < self.cfg.waypoint_tolerance:
                self.change(Phase.RETURN_TRANSIT if self.returning else Phase.TRANSIT, now)
        elif self.phase in (Phase.TRANSIT, Phase.RETURN_TRANSIT):
            target = (*self.home[:2], self.cruise_z) if self.returning else self.destination
            self.intent.position = step_toward(self.intent.position, target, dt)
            if distance(t.position, target) < self.cfg.waypoint_tolerance:
                if self.returning and self.recovery_started is not None:
                    self.change(Phase.RETURN_LAND, now, 'Recovery landing at launch')
                else:
                    self.path = search_path(*target, self.cfg.radius, self.cfg.spacing)
                    self.path_index = 0
                    self.change(Phase.SEARCH, now)
        elif self.phase == Phase.SEARCH:
            marker = self.markers.get(self.goal.home_marker_id if self.returning else self.goal.delivery_marker_id, now)
            if marker:
                self.last_marker = marker[0]
                self.change(Phase.APPROACH, now)
            elif self.path_index >= len(self.path):
                self.recover(Error.SEARCH_EXHAUSTED, now, t)
            else:
                target = self.path[self.path_index]
                self.intent.position = step_toward(self.intent.position, target, dt)
                if distance(t.position, target) < self.cfg.waypoint_tolerance:
                    self.path_index += 1
        elif self.phase in (Phase.APPROACH, Phase.DESCEND):
            self._precision(now, dt, t)
        elif landing:
            if self.substage == 0:
                self.command(3)
                self.substage = 1
            if self.acked(t) and (t.auto_land or ground):
                self.intent.stream = False
                self.substage = 2
            if grounded and self.substage == 2:
                if self.execution == Execution.CANCELING:
                    self.finish(Execution.CANCELED, now, detail='Cancellation landing complete')
                elif self.returning:
                    self.finish(Execution.FAILED if self.error else Execution.SUCCEEDED, now,
                                detail='Returned to launch')
                elif self.error:
                    self.finish(Execution.FAILED, now, detail='Landed after fault; payload not released')
                else:
                    self.change(Phase.RELEASE, now)
        elif self.phase == Phase.RELEASE:
            if self.execution == Execution.CANCELING:
                if t.release_done:
                    self.finish(Execution.CANCELED, now, detail='Payload cancellation settled')
                elif not t.payload_healthy:
                    self.finish(Execution.FAILED, now, Error.PAYLOAD_FAILED, 'Lost payload during cancellation')
            elif not grounded or not t.payload_healthy:
                self.finish(Execution.FAILED, now, Error.PAYLOAD_FAILED)
            elif t.release_done:
                if not (t.release_success and t.released and t.payload_closed and not t.payload_present):
                    self.finish(Execution.FAILED, now, Error.PAYLOAD_FAILED, t.release_detail)
                else:
                    self.returning = True
                    self.intent.position = t.position
                    self.change(Phase.RETURN_PREFLIGHT, now, 'Return preflight and 5 second countdown')
            elif t.payload_ready:
                self.release_requested = True

    def _preflight(self, now, t):
        if not t.camera_ready or not t.range_valid or not t.payload_healthy or not t.payload_closed or (self.returning and t.payload_present):
            self.finish(Execution.FAILED, now, Error.PREFLIGHT_FAILED)
            return
        self.intent.stream = True
        delay = 5. if self.returning else 2.
        if self.substage == 0 and now-self.phase_started >= delay:
            if t.armed or not t.landed:
                self.finish(Execution.FAILED, now, Error.PREFLIGHT_FAILED)
                return
            self.command(1)
            self.substage = 1
        elif self.substage == 1 and self.acked(t) and t.offboard:
            self.command(2)
            self.substage = 2
        elif self.substage == 2 and self.acked(t) and t.offboard and t.armed:
            self.change(Phase.RETURN_TAKEOFF if self.returning else Phase.TAKEOFF, now)

    def _precision(self, now, dt, t):
        marker = self.markers.get(self.goal.home_marker_id if self.returning else self.goal.delivery_marker_id, now)
        if not marker:
            self.center_since = None
            self.intent.position = t.position
            if self.last_marker is None or now-self.last_marker >= 3.:
                self.recover(Error.MARKER_LOST, now, t)
            return
        self.last_marker = marker[0]
        target = (*marker[1][:2], self.intent.position[2])
        self.intent.position = step_toward(self.intent.position, target, self.cfg.approach_speed*dt)
        centered = distance(t.position[:2], marker[1][:2]) <= self.cfg.center_tolerance
        self.center_since = (self.center_since if self.center_since is not None else now) if centered else None
        if self.phase == Phase.APPROACH and self.center_since is not None and now-self.center_since >= 1.:
            self.change(Phase.DESCEND, now)
        elif self.phase == Phase.DESCEND:
            if not t.range_valid:
                self.recover(Error.RANGE_LOST, now, t)
            elif centered:
                if t.agl <= self.cfg.handoff_agl:
                    self.change(Phase.RETURN_LAND if self.returning else Phase.LAND, now)
                else:
                    p = self.intent.position
                    self.intent.position = (p[0],p[1],p[2]-self.cfg.descend_speed*dt)
