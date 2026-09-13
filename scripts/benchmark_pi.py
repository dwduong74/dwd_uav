#!/usr/bin/env python3
"""Observe an independently operated test; never starts or arms a mission."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
from delivery_interfaces.msg import FlightState
from std_msgs.msg import Float32


def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=1800)
    p.add_argument('--output',type=Path,default=Path('benchmark.json')); args=p.parse_args()
    rclpy.init(); node=Node('delivery_benchmark')
    samples=[]; gaps=[]; thermal=[]; last=[None]; active=[False]
    def state(msg):
        if msg.offboard!=active[0]: last[0]=None
        active[0]=msg.offboard
    def mode(msg):
        now=time.monotonic()
        if active[0] and last[0] is not None: gaps.append(now-last[0])
        last[0]=now
    def temp():
        try:
            value=subprocess.check_output(['vcgencmd','get_throttled'],text=True,timeout=1).strip()
            thermal.append(int(value.split('=')[1],16))
        except (OSError,ValueError,subprocess.SubprocessError): pass
    node.create_subscription(FlightState,'/delivery/flight_state',state,10)
    node.create_subscription(OffboardControlMode,'/fmu/in/offboard_control_mode',mode,10)
    node.create_subscription(Float32,'/delivery/vision_latency',lambda m:samples.append(m.data),10)
    node.create_timer(5.,temp)
    started=time.monotonic()
    while rclpy.ok() and time.monotonic()-started<args.seconds:
        rclpy.spin_once(node,timeout_sec=.1)
        if active[0] and last[0] is not None and time.monotonic()-last[0]>.5:
            gaps.append(time.monotonic()-last[0])
    samples.sort()
    p95=samples[max(0,math.ceil(len(samples)*.95)-1)] if samples else None
    result=dict(duration=time.monotonic()-started,frame_count=len(samples),heartbeat_count=len(gaps),
                latency_p95=p95,max_heartbeat_gap=max(gaps) if gaps else None,
                throttling_observed=any(v & 0x50005 for v in thermal) if thermal else None)
    result['passed']=bool(result['duration']>=1800 and len(samples)>=100 and len(gaps)>=100
        and p95<=.2 and max(gaps)<.5 and result['throttling_observed'] is False)
    args.output.write_text(json.dumps(result,indent=2))
    node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
