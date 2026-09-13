import time
import unittest
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from delivery_ros.px4 import Px4Adapter
    from delivery_ros.vision import Vision
    from delivery_interfaces.msg import FlightIntent,FlightState,LandingTarget
    from px4_msgs.msg import (VehicleLocalPosition,VehicleStatus,VehicleLandDetected,
        VehicleGlobalPosition,VehicleOdometry,SensorGps,BatteryStatus,VehicleCommand,VehicleCommandAck,OffboardControlMode)
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo,Image
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

    def test_px4_schema_ack_state_and_watchdog(self):
        bridge=Px4Adapter(); self.nodes.append(bridge); self.ex.add_node(bridge)
        samples={
          'vehicle_local_position':VehicleLocalPosition(xy_valid=True,z_valid=True,xy_global=True,z_global=True,
              heading_good_for_control=True,ref_lat=10.,ref_lon=106.,ref_alt=0.,ref_timestamp=1,
              dist_bottom_valid=True,dist_bottom_sensor_bitfield=1,dist_bottom=.2),
          'vehicle_status':VehicleStatus(), 'vehicle_land_detected':VehicleLandDetected(landed=True),
          'vehicle_global_position':VehicleGlobalPosition(lat=10.,lon=106.),
          'vehicle_odometry':VehicleOdometry(pose_frame=1,q=[1.,0.,0.,0.],position=[0.,0.,0.]),
          'vehicle_gps_position':SensorGps(fix_type=3),
          'battery_status':BatteryStatus(connected=True,remaining=.9)}
        pubs={k:self.node.create_publisher(type(v),'/fmu/out/'+k,qos_profile_sensor_data) for k,v in samples.items()}
        def publish():
            stamp=self.node.get_clock().now().nanoseconds//1000
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
        intent=FlightIntent(stream=True,command_id=7,command=1)
        def send():
            intent.header.stamp=self.node.get_clock().now().to_msg()
            ip.publish(intent)
        timer=self.node.create_timer(.05,send)
        self.spin(.3)
        self.assertEqual(len(commands),1)
        self.assertGreater(len(heartbeats),2)
        ack=VehicleCommandAck(command=commands[0].command,result=0,target_system=1,target_component=191,
            timestamp=self.node.get_clock().now().nanoseconds//1000)
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
