import time
import unittest
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from delivery_ros.px4 import Px4Adapter, px4_topic
    from delivery_ros.vision import Vision
    from delivery_ros.sim_camera import SimCameraClock
    from delivery_interfaces.msg import FlightIntent,FlightState,LandingTarget
    from px4_msgs.msg import (VehicleLocalPosition,VehicleStatus,VehicleLandDetected,
        VehicleGlobalPosition,VehicleOdometry,SensorGps,BatteryStatus,VehicleCommand,VehicleCommandAck,OffboardControlMode,EstimatorStatusFlags)
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo,Image,LaserScan
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    ROS=True
except ImportError as exc:
    if exc.name != 'rclpy':
        raise
    ROS=False


@unittest.skipUnless(ROS,'Requires built ROS 2 Humble interfaces')
class NodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','enable_control:=true','-p','calibrated:=true'])
        self.node=Node('node_test_driver')
        self.ex=SingleThreadedExecutor(); self.ex.add_node(self.node)
        self.nodes=[self.node]

    def tearDown(self):
        self.ex.shutdown()
        for node in self.nodes: node.destroy_node()
        rclpy.shutdown()

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: self.ex.spin_once(timeout_sec=.01)

    def test_sim_sensor_wall_time_and_frozen_frame(self):
        relay=SimCameraClock(); self.nodes.append(relay); self.ex.add_node(relay)
        # A frame acquired 12 simulator seconds maps to a known wall-clock time.
        from px4_msgs.msg import TimesyncStatus
        acquisition = self.node.get_clock().now().nanoseconds
        relay.on_timesync(TimesyncStatus(source_protocol=TimesyncStatus.SOURCE_PROTOCOL_DDS,
            estimated_offset=(12000000000-acquisition)//1000))
        received={}; pubs={}; samples={}
        for name,cls in [('image_raw',Image),('camera_info',CameraInfo),('scan',LaserScan)]:
            received[name]=[]
            self.node.create_subscription(cls,'/camera/'+name,
                lambda msg,name=name:received[name].append(msg),qos_profile_sensor_data)
            pubs[name]=self.node.create_publisher(cls,'/gazebo/camera/'+name,qos_profile_sensor_data)
            samples[name]=cls()
            samples[name].header.frame_id='camera_optical_frame'
            samples[name].header.stamp.sec=12
        samples['scan'].ranges=[2.,.4,2.]
        self.spin(.2)
        for _ in range(3):
            for name,msg in samples.items(): pubs[name].publish(msg)
            self.spin(.1)
        for messages in received.values():
            self.assertEqual(len(messages),1,'A repeated simulator frame must not become fresh')
            msg=messages[0]
            age=self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
            self.assertTrue(0<=age<.7)
            mapped=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            self.assertLess(abs(mapped-acquisition),1000)
            self.assertEqual(msg.header.frame_id,'camera_optical_frame')
        self.assertAlmostEqual(received['scan'][0].ranges[0],.4)
        self.assertEqual(len(received['scan'][0].ranges),1)
        for name,msg in samples.items():
            msg.header.stamp.sec=11; pubs[name].publish(msg)
        self.spin(.1)
        self.assertTrue(all(len(messages)==1 for messages in received.values()))

    def test_vision_keeps_frame_until_delayed_tf_arrives(self):
        vision=Vision(); self.nodes.append(vision)
        vision.on_info(CameraInfo(width=640,height=480,distortion_model='plumb_bob',
            k=[400.,0.,320.,0.,400.,240.,0.,0.,1.],d=[0.]*5))
        first=Image(); first.header.frame_id='camera_optical_frame'
        first.header.stamp=self.node.get_clock().now().to_msg()
        vision.on_image(first); vision.process()
        self.assertTrue(vision.waiting_tf)
        newer=Image(); newer.header.stamp=self.node.get_clock().now().to_msg()
        vision.on_image(newer)
        self.assertIs(vision.latest,first)

    def test_px4_schema_ack_state_and_watchdog(self):
        bridge=Px4Adapter(); self.nodes.append(bridge); self.ex.add_node(bridge)
        samples={
          'vehicle_local_position':VehicleLocalPosition(xy_valid=True,z_valid=True,xy_global=True,z_global=True,
              heading_good_for_control=False,ref_lat=10.,ref_lon=106.,ref_alt=0.,ref_timestamp=1,
              dist_bottom_valid=True,dist_bottom_sensor_bitfield=1,dist_bottom=.2),
          'vehicle_status':VehicleStatus(), 'vehicle_land_detected':VehicleLandDetected(landed=True),
          'vehicle_global_position':VehicleGlobalPosition(lat=10.,lon=106.),
          'vehicle_odometry':VehicleOdometry(pose_frame=1,q=[1.,0.,0.,0.],position=[0.,0.,0.]),
          'vehicle_gps_position':SensorGps(fix_type=3),
          'battery_status':BatteryStatus(connected=True,remaining=.9),
          'estimator_status_flags':EstimatorStatusFlags(cs_yaw_align=True)}
        self.assertEqual(px4_topic('/fmu/out/vehicle_status',VehicleStatus), '/fmu/out/vehicle_status_v1')
        self.assertEqual(px4_topic('/fmu/out/battery_status',BatteryStatus), '/fmu/out/battery_status_v1')
        self.assertEqual(px4_topic('/fmu/in/vehicle_command',VehicleCommand), '/fmu/in/vehicle_command')
        pubs={k:self.node.create_publisher(type(v),px4_topic('/fmu/out/'+k,type(v)),qos_profile_sensor_data) for k,v in samples.items()}
        def publish():
            stamp=self.node.get_clock().now().nanoseconds//1000-250000
            for k,v in samples.items():
                v.timestamp=stamp
                if k=='vehicle_odometry': v.timestamp_sample=stamp
                pubs[k].publish(v)
        self.node.create_timer(.05,publish)
        commands=[]; states=[]; heartbeats=[]
        self.node.create_subscription(VehicleCommand,'/fmu/in/vehicle_command',commands.append,10)
        self.node.create_subscription(FlightState,'/delivery/flight_state',states.append,10)
        self.node.create_subscription(OffboardControlMode,'/fmu/in/offboard_control_mode',heartbeats.append,10)
        ip=self.node.create_publisher(FlightIntent,'/delivery/flight_intent',1)
        ap=self.node.create_publisher(VehicleCommandAck,'/fmu/out/vehicle_command_ack',qos_profile_sensor_data)
        self.spin(.5)
        self.assertTrue(states[-1].healthy)
        samples['estimator_status_flags'].cs_yaw_align=False
        self.spin(.15)
        self.assertFalse(states[-1].healthy,'Unaligned yaw must block control')
        samples['estimator_status_flags'].cs_yaw_align=True
        self.spin(.15)
        intent=FlightIntent(stream=True,command_id=7,command=1)
        def send():
            intent.header.stamp=self.node.get_clock().now().to_msg()
            ip.publish(intent)
        timer=self.node.create_timer(.05,send)
        self.spin(.3)
        self.assertEqual(len(commands),1)
        self.assertGreater(len(heartbeats),2)
        ack=VehicleCommandAck(command=commands[0].command,result=0,target_system=1,target_component=191,
            timestamp=bridge.tracker.stamp_us-1)
        ap.publish(ack); self.spin(.05)
        self.assertFalse(bridge.tracker.accepted, 'Old PX4 ACK must be ignored')
        ack.timestamp=bridge.tracker.stamp_us+1
        self.assertLess(ack.timestamp,commands[0].timestamp, 'PX4 clock lags send wall time')
        ap.publish(ack); self.spin(.15)
        self.assertEqual(states[-1].command_state,1)
        samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.spin(.2)
        self.assertEqual(states[-1].command_state,2)
        self.assertEqual(len(commands),1)
        self.node.destroy_timer(timer)
        self.spin(.35); count=len(heartbeats); self.spin(.2)
        self.assertEqual(len(heartbeats),count)

    def test_calibrated_vision_uses_pose_transform(self):
        vision=Vision(); self.nodes.append(vision); self.ex.add_node(vision)
        poses=[]
        self.node.create_subscription(LandingTarget,'/delivery/landing_target',poses.append,10)
        tf=StaticTransformBroadcaster(self.node)
        transform=TransformStamped()
        transform.header.frame_id='map'; transform.child_frame_id='camera_optical_frame'
        transform.transform.translation.z=1.
        transform.transform.rotation.x=1.
        transform.transform.rotation.w=0.
        tf.sendTransform(transform)
        ip=self.node.create_publisher(Image,'/camera/image_raw',qos_profile_sensor_data)
        cp=self.node.create_publisher(CameraInfo,'/camera/camera_info',qos_profile_sensor_data)
        info=CameraInfo(width=640,height=480,distortion_model='plumb_bob',
                        k=[400.,0.,320.,0.,400.,240.,0.,0.,1.],d=[0.]*5)
        info.header.frame_id='camera_optical_frame'
        image=np.full((480,640),255,np.uint8)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        marker=cv2.aruco.drawMarker(dictionary,0,200)
        image[140:340,220:420]=marker
        msg=CvBridge().cv2_to_imgmsg(image,encoding='mono8')
        msg.header.frame_id='camera_optical_frame'
        def publish():
            stamp=self.node.get_clock().now().to_msg()
            info.header.stamp=stamp; msg.header.stamp=stamp
            cp.publish(info); ip.publish(msg)
        timer=self.node.create_timer(.1,publish)
        self.spin(1.)
        self.assertTrue(poses,'No transformed ArUco pose')
        self.assertEqual(poses[-1].header.frame_id,'map')
        self.assertAlmostEqual(poses[-1].pose.position.z,.2,delta=.03)
        self.assertAlmostEqual(poses[-1].pose.position.x,0.,delta=.02)
        self.node.destroy_timer(timer)
        self.spin(.2)
        count=len(poses)
        ip.publish(msg)  # identical timestamp must not create another observation
        self.spin(.1)
        self.assertEqual(len(poses),count)
