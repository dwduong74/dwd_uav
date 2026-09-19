#!/usr/bin/env python3
"""Generate a downward-camera x500 and two real ArUco-textured landing pads."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml


def prepare(px4,output,table_c=False,course_file=None):
    world_source=px4/'Tools/simulation/gz/worlds/default.sdf'
    if not world_source.exists():
        raise ValueError('Initialize PX4 1.17 Gazebo submodule before preparing assets')
    models=output/'models'; worlds=output/'worlds'
    worlds.mkdir(parents=True,exist_ok=True)
    camera=models/'x500_delivery'; camera.mkdir(parents=True,exist_ok=True)
    (camera/'model.config').write_text('<model><name>x500_delivery</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
    sensor_xml = '''
        <sensor name="delivery_image" type="camera">
          <pose>0 0 -0.30 0 1.5707963267948966 0</pose>
          <always_on>true</always_on><update_rate>10</update_rate>
          <topic>/camera/image_raw</topic><gz_frame_id>camera_optical_frame</gz_frame_id>
          <camera><horizontal_fov>1.0471975512</horizontal_fov>
            <image><width>320</width><height>240</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>100</far></clip>
            <camera_info_topic>/camera/camera_info</camera_info_topic>
          </camera>
        </sensor>
        <sensor name="delivery_range" type="gpu_lidar">
          <pose>0 0 -0.15 0 1.5707963267948966 0</pose>
          <always_on>true</always_on><update_rate>15</update_rate><topic>/camera/scan</topic>
          <lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.001</min_angle><max_angle>0.001</max_angle></horizontal></scan>
            <range><min>0.02</min><max>30</max><resolution>0.01</resolution></range>
          </lidar>
        </sensor>
        <sensor name="forward_depth" type="gpu_lidar">
          <pose>0.20 0 0 0 0 0</pose>
          <always_on>true</always_on><update_rate>15</update_rate><topic>/depth/scan</topic>
          <lidar><scan><horizontal><samples>121</samples><resolution>1</resolution><min_angle>-1.0472</min_angle><max_angle>1.0472</max_angle></horizontal></scan>
            <range><min>0.10</min><max>15</max><resolution>0.02</resolution></range>
          </lidar>
        </sensor>'''
    # Flatten stock x500 and put both sensors on base_link. A separate tiny link
    # made the fixed-joint physics unstable enough to saturate EKF accel bias.
    stock = px4/'Tools/simulation/gz/models/x500/model.sdf'
    if stock.exists():
        env = os.environ.copy()
        env['SDF_PATH'] = str(stock.parents[1])
        expanded = subprocess.run(['gz', 'sdf', '-p', str(stock)], env=env,
                                  check=True, text=True, capture_output=True).stdout
        vehicle = ET.fromstring(expanded)
        model = vehicle.find('model')
        model.set('name', 'x500_delivery')
        base = model.find("link[@name='base_link']")
        if base is None:
            raise ValueError('Expanded x500 has no base_link')
        wrapper = ET.fromstring('<link>'+sensor_xml+'</link>')
        for sensor in list(wrapper):
            base.append(sensor)
        ET.ElementTree(vehicle).write(camera/'model.sdf', encoding='unicode')
    else:  # Minimal test fixture without the PX4 model tree.
        (camera/'model.sdf').write_text('''<sdf version="1.9"><model name="x500_delivery">
          <include merge="true"><uri>model://x500</uri></include>
          <link name="delivery_camera"><pose relative_to="base_link">0 0 -0.15 0 0 0</pose>'''
          + sensor_xml + '</link></model></sdf>')
    tree=ET.parse(world_source); world=tree.getroot().find('world'); world.set('name','delivery')
    physics = world.find('physics')
    if physics is not None:
        physics.find('max_step_size').text = '0.008'
        physics.find('real_time_update_rate').text = '125'
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    course_data=None
    if table_c:
        source=course_file or Path(__file__).resolve().parents[1]/'config/table_c.yaml'
        all_courses=yaml.safe_load(Path(source).read_text())['courses']
        course_data=all_courses['table_c_demo']
        home=course_data['home']
        pads=[(int(home['marker_id']),float(home['position'][0]),float(home['position'][1]))]
        pads += [(int(p['marker_id']),float(p['position'][0]),float(p['position'][1]))
                 for p in course_data['delivery_points'].values()]
    else:
        pads=((1,0.,0.),(0,3.,0.))
    for marker_id,east,north in pads:
        name='delivery_pad_'+str(marker_id)
        model=models/name; model.mkdir(parents=True,exist_ok=True)
        raw=(cv2.aruco.drawMarker(dictionary,marker_id,400) if hasattr(cv2.aruco,'drawMarker')
             else cv2.aruco.generateImageMarker(dictionary,marker_id,400))
        canvas=np.full((600,600),255,np.uint8); canvas[100:500,100:500]=raw
        texture=f'marker_{marker_id}.png'
        cv2.imwrite(str(model/texture),canvas)
        (model/'model.config').write_text(f'<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
        (model/'model.sdf').write_text(f'''<sdf version="1.9"><model name="{name}"><static>true</static>
          <link name="pad"><visual name="marker"><geometry><plane><normal>0 0 1</normal><size>0.6 0.6</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{name}/{texture}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
          </material></visual></link></model></sdf>''')
        include=ET.SubElement(world,'include')
        ET.SubElement(include,'uri').text='model://'+name
        # The Gazebo bridge maps world X to PX4 east / ROS ENU X.
        north_text='0' if north==0 else str(north)
        ET.SubElement(include,'pose').text=f'{east} {north_text} 0.005 0 0 0'
    if table_c:
        def box(name,pose,size):
            model=ET.SubElement(world,'model',name=name); ET.SubElement(model,'static').text='true'
            ET.SubElement(model,'pose').text=' '.join(map(str,pose))
            link=ET.SubElement(model,'link',name='body')
            for kind in ('collision','visual'):
                item=ET.SubElement(link,kind,name=kind); geom=ET.SubElement(item,'geometry')
                shape=ET.SubElement(geom,'box'); ET.SubElement(shape,'size').text=' '.join(map(str,size))
        for obstacle in course_data['obstacles']:
            name,kind=obstacle['name'],obstacle['type']
            x,y,z=map(float,obstacle['center'])
            if kind=='box':
                box(name,(x,y,z,0,0,0),tuple(map(float,obstacle['size'])))
            elif kind=='gate':
                width=float(obstacle['opening_width']); bottom=float(obstacle['opening_bottom'])
                height=float(obstacle['opening_height']); thick=float(obstacle['thickness'])
                post_width=.5; total_height=bottom+height+.5
                box(name+'_left',(x,y-width/2-post_width/2,total_height/2,0,0,0),(thick,post_width,total_height))
                box(name+'_right',(x,y+width/2+post_width/2,total_height/2,0,0,0),(thick,post_width,total_height))
                box(name+'_top',(x,y,bottom+height+.25,0,0,0),(thick,width+2*post_width,.5))
            elif kind=='tunnel':
                length,width,height=map(float,obstacle['size']); thick=float(obstacle['thickness'])
                box(name+'_left',(x,y-width/2,z,0,0,0),(length,thick,height))
                box(name+'_right',(x,y+width/2,z,0,0,0),(length,thick,height))
                box(name+'_top',(x,y,z+height/2,0,0,0),(length,width,thick))
    tree.write(worlds/'delivery.sdf',encoding='unicode')
    spherical=world.find('spherical_coordinates')
    if spherical is None:
        spherical=ET.SubElement(world,'spherical_coordinates')
        for key,value in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),
                          ('latitude_deg','47.397971'),('longitude_deg','8.546163'),('elevation','488'),('heading_deg','0')]:
            ET.SubElement(spherical,key).text=value
    if table_c:
        origin=course_data['origin']
        for key,value in [('latitude_deg',origin['latitude']),('longitude_deg',origin['longitude']),
                          ('elevation',origin.get('altitude',0.))]:
            child=spherical.find(key)
            if child is None: child=ET.SubElement(spherical,key)
            child.text=str(value)
    tree.write(worlds/'delivery.sdf',encoding='unicode')
    lat=float(spherical.findtext('latitude_deg')); lon=float(spherical.findtext('longitude_deg'))
    if table_c:
        deliveries=list(course_data['delivery_points'].values())
        package_ids=[int(p['marker_id']) for p in course_data['pickup_points']]
        delivery_ids=[int(p['marker_id']) for p in deliveries]
        home_id=int(course_data['home']['marker_id'])
        first=deliveries[0]; first_east,first_north=map(float,first['position'][:2])
        mission_lat=lat+math.degrees(first_north/6371000.)
        mission_lon=lon+math.degrees(first_east/(6371000*math.cos(math.radians(lat))))
        geofence_flat=[float(value) for point in course_data['geofence'] for value in point]
    else:
        package_ids=[]; delivery_ids=[0]; home_id=1; geofence_flat=[]
        mission_lat=lat
        mission_lon=lon+math.degrees(3/(6371000*math.cos(math.radians(lat))))
    config={
        'mission':{'ros__parameters':dict(enable_control=True,latitude=mission_lat,
           longitude=mission_lon,relative_altitude=2.,
           delivery_marker_id=delivery_ids[0],home_marker_id=home_id,radius=1.,spacing=1.,min_battery=.15,
           handoff_agl=.8)},
        'px4_adapter':{'ros__parameters':dict(enable_control=True,range_source='ros_scan')},
        'vision':{'ros__parameters':dict(calibrated=True,marker_size=.4,
            package_marker_ids=package_ids,delivery_marker_ids=delivery_ids,home_marker_id=home_id)},
        'payload':{'ros__parameters':dict(backend='sim',initial_present=not table_c)},
        'competition':{'ros__parameters':dict(enable_control=table_c,course_id='table_c_demo',scan_timeout=30.)},
        'safety_monitor':{'ros__parameters':dict(enable_obstacle_guard=table_c,
            require_obstacle_data=table_c,obstacle_clearance=.55,
            geofence_xy=geofence_flat)},
        'vio_bridge':{'ros__parameters':dict(enabled=False)},
        'sim_package_station':{'ros__parameters':dict(enabled=table_c,package_marker_ids=package_ids)}}
    (output/'sitl.yaml').write_text(yaml.safe_dump(config))
    print('Generated',output,'; simulation only, no mission starts automatically')


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('px4_checkout',type=Path)
    p.add_argument('--output',type=Path,default=Path('build/sitl_assets'))
    p.add_argument('--table-c',action='store_true')
    p.add_argument('--course',type=Path)
    args=p.parse_args(); prepare(args.px4_checkout.resolve(),args.output.resolve(),args.table_c,args.course)
