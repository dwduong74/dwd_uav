"""Pure state machine for one three-package Table C run."""
from enum import IntEnum


class CompetitionPhase(IntEnum):
    IDLE=0; SCAN_PACKAGE=1; GRAB_PACKAGE=2; DELIVER=3; RETURNED=4
    HOLD=5; FINISHED=6; FAILED=7


class CompetitionEngine:
    def __init__(self, course, marker_confirmations=5, marker_window=.5):
        self.course = course
        self.confirmations = marker_confirmations
        self.window = marker_window
        self.reset()

    def reset(self):
        self.phase = CompetitionPhase.IDLE
        self.index = self.completed = 0
        self.detail = 'Ready'
        self.error = 0
        self.samples = []
        self.package_marker = self.destination_marker = -1
        self.started = self.ended = None

    @property
    def active(self):
        return self.phase not in (CompetitionPhase.IDLE, CompetitionPhase.FINISHED, CompetitionPhase.FAILED)

    @property
    def progress(self):
        return 1. if self.phase == CompetitionPhase.FINISHED else self.completed/len(self.course.packages)

    def start(self, now, count=3):
        if self.active or count != 3:
            return False
        self.reset(); self.started = now
        self.phase = CompetitionPhase.SCAN_PACKAGE
        self.detail = 'Scan package 1/3'
        return True

    def marker(self, marker_id, stamp):
        if self.phase != CompetitionPhase.SCAN_PACKAGE:
            return False
        expected = self.course.packages[self.index].marker_id
        if marker_id != expected:
            return False
        self.samples = [s for s in self.samples if 0 <= stamp-s <= self.window]
        if self.samples and stamp <= self.samples[-1]:
            return False
        self.samples.append(stamp)
        if len(self.samples) < self.confirmations:
            return False
        task = self.course.packages[self.index]
        self.package_marker = marker_id
        self.destination_marker = self.course.destinations[task.destination].marker_id
        self.phase = CompetitionPhase.GRAB_PACKAGE
        self.detail = f'Grab package marker {marker_id}'
        return True

    def grab_result(self, success, detail=''):
        if self.phase != CompetitionPhase.GRAB_PACKAGE:
            return False
        if not success:
            self.fail(21, detail or 'Payload grab failed')
            return False
        self.phase = CompetitionPhase.DELIVER
        self.detail = f'Deliver to marker {self.destination_marker}'
        return True

    def leg_result(self, success, now, detail=''):
        if self.phase != CompetitionPhase.DELIVER:
            return False
        if not success:
            self.fail(22, detail or 'Delivery leg failed', now)
            return False
        self.completed += 1
        self.phase = CompetitionPhase.RETURNED
        self.detail = f'Returned with {self.completed}/3 deliveries complete'
        if self.completed == len(self.course.packages):
            self.phase = CompetitionPhase.FINISHED; self.ended = now
            self.detail = 'All three packages delivered and vehicle returned'
        else:
            self.index += 1; self.samples = []
            self.package_marker = self.destination_marker = -1
            self.phase = CompetitionPhase.SCAN_PACKAGE
            self.detail = f'Scan package {self.index+1}/3'
        return True

    def fail(self, code, detail, now=None):
        self.error, self.detail, self.phase = code, detail, CompetitionPhase.FAILED
        self.ended = now
