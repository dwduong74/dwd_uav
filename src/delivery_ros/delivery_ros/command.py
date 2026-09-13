"""One outstanding PX4 transaction; ACK alone is not execution success."""
class CommandTracker:
    def __init__(self, timeout=5.):
        self.timeout = timeout
        self.token = 0
        self.kind = 0
        self.command = 0
        self.state = 0
        self.accepted = False
        self.sent = 0.
        self.stamp_us = 0
        self.detail = ''

    def begin(self, token, kind, command, now, stamp_us):
        if token == self.token:
            return False
        if self.state == 1:
            return False
        self.token, self.kind, self.command = token, kind, command
        self.sent, self.stamp_us = now, stamp_us
        self.state, self.accepted, self.detail = 1, False, 'Awaiting ACK and state'
        return True

    def ack(self, command, result, stamp_us):
        if self.state != 1 or command != self.command or stamp_us < self.stamp_us:
            return
        if result == 0:
            self.accepted = True
        elif result != 5:
            self.state, self.detail = 3, 'PX4 rejected command: '+str(result)

    def tick(self, now, confirmed):
        if self.state == 1:
            if self.accepted and confirmed:
                self.state, self.detail = 2, 'ACK and state confirmed'
            elif now-self.sent >= self.timeout:
                self.state, self.detail = 3, 'ACK/state timeout'
