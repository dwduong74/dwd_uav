#!/usr/bin/env python3
"""Generate a downward-camera x500 and two real ArUco-textured landing pads."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml


def prepare(px4,output):
    world_source=px4/'Tools/simulation/gz/worlds/default.sdf'
    if not world_source.exists():
        raise ValueError('Initialize PX4 1.15 Gazebo submodule before preparing assets')
    models=output/'models'; worlds=output/'worlds'
    worlds.mkdir(parents=True,exist_ok=True)
    camera=models/'x500_delivery'; camera.mkdir(parents=True,exist_ok=True)
    (camera/'model.config').write_text('<model><name>x500_delivery</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
    (camera/'model.sdf').write_text('''<sdf version="1.9"><model name="x500_delivery">
      <include merge="true"><uri>model://x500</uri></include>
      <link name="delivery_camera">
        <pose>0 0 -0.05 0 1.5707963267948966 0</pose>
        <inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
        <sensor name="delivery_image" type="camera">
          <always_on>true</always_on><update_rate>10</update_rate>
          <topic>/camera/image_raw</topic><gz_frame_id>camera_optical_frame</gz_frame_id>
          <camera><horizontal_fov>1.0471975512</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>100</far></clip>
            <camera_info_topic>/camera/camera_info</camera_info_topic>
          </camera>
        </sensor>
        <sensor name="delivery_range" type="gpu_lidar">
          <always_on>true</always_on><update_rate>20</update_rate><topic>/camera/scan</topic>
          <lidar><scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan>
            <range><min>0.02</min><max>30</max><resolution>0.01</resolution></range>
          </lidar>
        </sensor>
      </link>
      <joint name="delivery_camera_fixed" type="fixed"><parent>base_link</parent><child>delivery_camera</child></joint>
    </model></sdf>''')
    tree=ET.parse(world_source); world=tree.getroot().find('world'); world.set('name','delivery')
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    for marker_id,east in ((1,0.),(0,3.)):
        name='delivery_pad_'+str(marker_id)
        model=models/name; model.mkdir(parents=True,exist_ok=True)
        raw=(cv2.aruco.drawMarker(dictionary,marker_id,400) if hasattr(cv2.aruco,'drawMarker')
             else cv2.aruco.generateImageMarker(dictionary,marker_id,400))
        canvas=np.full((600,600),255,np.uint8); canvas[100:500,100:500]=raw
        cv2.imwrite(str(model/'marker.png'),canvas)
        (model/'model.config').write_text(f'<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
        (model/'model.sdf').write_text(f'''<sdf version="1.9"><model name="{name}"><static>true</static>
          <link name="pad"><visual name="marker"><geometry><plane><normal>0 0 1</normal><size>0.6 0.6</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{name}/marker.png</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
          </material></visual></link></model></sdf>''')
        include=ET.SubElement(world,'include')
        ET.SubElement(include,'uri').text='model://'+name
        ET.SubElement(include,'pose').text=f'{east} 0 0.005 0 0 0'
    tree.write(worlds/'delivery.sdf',encoding='unicode')
    spherical=world.find('spherical_coordinates')
    if spherical is None:
        spherical=ET.SubElement(world,'spherical_coordinates')
        for key,value in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),
                          ('latitude_deg','47.397971'),('longitude_deg','8.546163'),('elevation','488'),('heading_deg','0')]:
            ET.SubElement(spherical,key).text=value
        tree.write(worlds/'delivery.sdf',encoding='unicode')
    lat=float(spherical.findtext('latitude_deg')); lon=float(spherical.findtext('longitude_deg'))
    config={
        'mission':{'ros__parameters':dict(enable_control=True,latitude=lat,
           longitude=lon+math.degrees(3/(6371000*math.cos(math.radians(lat)))) ,relative_altitude=2.,
           delivery_marker_id=0,home_marker_id=1,radius=1.,spacing=1.,min_battery=.15)},
        'px4_adapter':{'ros__parameters':dict(enable_control=True,range_source='ros_scan')},
        'vision':{'ros__parameters':dict(calibrated=True,marker_size=.4)},
        'payload':{'ros__parameters':dict(backend='sim')}}
    (output/'sitl.yaml').write_text(yaml.safe_dump(config))
    print('Generated',output,'; simulation only, no mission starts automatically')


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('px4_checkout',type=Path)
    p.add_argument('--output',type=Path,default=Path('build/sitl_assets'))
    args=p.parse_args(); prepare(args.px4_checkout.resolve(),args.output.resolve())
