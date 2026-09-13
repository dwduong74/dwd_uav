#!/usr/bin/env python3
"""Add the required land detector publication to a PX4 1.15 checkout."""
import argparse
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('px4_checkout',type=Path)
args=p.parse_args()
path=args.px4_checkout/'src/modules/uxrce_dds_client/dds_topics.yaml'
text=path.read_text()
entry='  - topic: /fmu/out/vehicle_land_detected\n    type: px4_msgs::msg::VehicleLandDetected\n\n'
if '/fmu/out/vehicle_land_detected' not in text:
    if 'publications:\n' not in text:
        raise SystemExit('Unexpected DDS schema; no changes made')
    path.write_text(text.replace('publications:\n','publications:\n\n'+entry,1))
print('vehicle_land_detected configured; rebuild PX4 firmware/SITL')
