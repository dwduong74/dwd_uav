"""Bounded payload protocol. GRAB/RELEASE operations are UUID-idempotent."""
from dataclasses import dataclass


def crc16(data):
    crc=0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1)^0x1021 if crc & 0x8000 else crc << 1)&0xffff
    return crc


def frame(text):
    raw=text.encode('ascii')
    return raw+('*%04X\n'%crc16(raw)).encode('ascii')


def unframe(raw):
    if len(raw)>160:
        raise ValueError('Oversized serial frame')
    text,checksum=raw.strip().rsplit(b'*',1)
    if len(checksum)!=4 or crc16(text)!=int(checksum,16):
        raise ValueError('CRC mismatch')
    return text.decode('ascii')


@dataclass
class Reading:
    boot: str = ''
    closed: bool = False
    present: bool = False
    busy: bool = False
    fault: bool = True
    last_id: str = '-'


def parse_state(raw):
    fields=unframe(raw).split()
    if len(fields)!=7 or fields[0]!='STATE' or any(x not in ('0','1') for x in fields[2:6]):
        raise ValueError('Invalid STATE frame')
    if not fields[1].isdigit() or len(fields[1])>10:
        raise ValueError('Invalid boot counter')
    if fields[6]!='-' and (len(fields[6])!=32 or any(c not in '0123456789abcdef' for c in fields[6])):
        raise ValueError('Invalid mission token')
    return Reading(fields[1],*(bool(int(x)) for x in fields[2:6]),fields[6])


class SimBackend:
    def __init__(self, present=True):
        self.reading=Reading('1',True,present,False,False)
        self.started=None
        self.operation=''
        self.count=0

    def update(self,now):
        if self.started is not None:
            if now-self.started>.5:
                self.reading.present=self.operation=='GRAB'
            if now-self.started>1.:
                self.reading.closed,self.reading.busy=True,False
                self.started=None
        return self.reading

    def release(self,token,now):
        r=self.reading
        if r.last_id==token:
            return
        if r.busy or r.fault or not r.closed or not r.present:
            raise ValueError('Payload not ready')
        r.last_id,r.busy,r.closed=token,True,False
        self.started,self.operation=now,'RELEASE'
        self.count+=1

    def grab(self,token,now):
        r=self.reading
        if r.last_id==token:
            return
        if r.busy or r.fault or not r.closed or r.present:
            raise ValueError('Payload not ready for grab')
        r.last_id,r.busy,r.closed=token,True,False
        self.started,self.operation=now,'GRAB'
        self.count+=1

    def stop(self):
        self.reading.busy=False
        self.reading.fault=True
        self.started=None


class SerialBackend:
    def __init__(self,port):
        import serial
        self.serial=serial.Serial(port,115200,timeout=0,write_timeout=.05)
        self.reading=Reading()
        self.last=-1e9
        self.buffer=b''

    def update(self,now):
        self.serial.write(frame('PING'))
        self.buffer+=self.serial.read(min(512,self.serial.in_waiting))
        if len(self.buffer)>1024:
            self.buffer=b''
        while b'\n' in self.buffer:
            raw,self.buffer=self.buffer.split(b'\n',1)
            try:
                self.reading=parse_state(raw)
                self.last=now
            except (ValueError,UnicodeError):
                continue
        if now-self.last>.5:
            raise TimeoutError('Payload telemetry stale')
        return self.reading

    def release(self,token,now):
        self.serial.write(frame('RELEASE '+self.reading.boot+' '+token))

    def grab(self,token,now):
        self.serial.write(frame('GRAB '+self.reading.boot+' '+token))

    def stop(self):
        self.serial.write(frame('STOP'))
