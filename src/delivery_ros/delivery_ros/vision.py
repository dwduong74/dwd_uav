"""Calibrated ArUco pose estimates transformed at image acquisition time."""
import math
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from delivery_interfaces.msg import MarkerDetection, LandingTarget, FiducialObservation
from .geometry import rotate, multiply, normalize


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        cv2.setNumThreads(2)
        self.calibrated = self.declare_parameter('calibrated',False).value
        self.size = self.declare_parameter('marker_size',.4).value
        self.package_ids = set(self.declare_parameter('package_marker_ids',Parameter.Type.INTEGER_ARRAY).value or [])
        self.delivery_ids = set(self.declare_parameter('delivery_marker_ids',Parameter.Type.INTEGER_ARRAY).value or [])
        self.home_id = self.declare_parameter('home_marker_id',-1).value
        sized_ids=list(self.declare_parameter('sized_marker_ids',Parameter.Type.INTEGER_ARRAY).value or [])
        sized_values=list(self.declare_parameter('sized_marker_sizes',Parameter.Type.DOUBLE_ARRAY).value or [])
        if len(sized_ids)!=len(sized_values):
            raise ValueError('sized_marker_ids and sized_marker_sizes must have equal length')
        self.sizes={int(k):float(v) for k,v in zip(sized_ids,sized_values)}
        if not math.isfinite(self.size) or self.size <= 0:
            raise ValueError('marker_size must be positive metres')
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco,'DetectorParameters_create')
                           else cv2.aruco.DetectorParameters())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,self.parameters) if hasattr(cv2.aruco,'ArucoDetector') else None
        self.pub = self.create_publisher(MarkerDetection,'/delivery/markers',10)
        self.pose_pub = self.create_publisher(LandingTarget,'/delivery/landing_target',1)
        self.observation_pub = self.create_publisher(FiducialObservation,'/delivery/fiducials',10)
        self.diag = self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.ready_pub = self.create_publisher(Bool,'/delivery/vision_ready',1)
        self.latency_pub = self.create_publisher(Float32,'/delivery/vision_latency',10)
        self.last_tf_ok = -math.inf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.info = self.latest = None
        self.waiting_tf = False
        self.last_stamp = -1
        self.processed = self.dropped = 0
        self.latency = 0.
        self.create_subscription(CameraInfo,'camera/camera_info',self.on_info,qos_profile_sensor_data)
        self.create_subscription(Image,'camera/image_raw',self.on_image,qos_profile_sensor_data)
        self.create_timer(.02,self.process)
        self.create_timer(1.,self.diagnostics)
        self.create_timer(.1,self.readiness)

    def on_info(self,msg):
        if (msg.k[0] > 0 and msg.k[4] > 0 and msg.width > 0 and msg.height > 0
                and msg.distortion_model in ('plumb_bob','rational_polynomial')
                and all(math.isfinite(v) for v in (*msg.k,*msg.d))):
            self.info = msg

    def on_image(self,msg):
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp > self.last_stamp:
            if self.latest is not None:
                self.dropped += 1
                # Do not continually replace a frame before its delayed TF arrives.
                pending=self.latest.header.stamp.sec*1000000000+self.latest.header.stamp.nanosec
                age=(self.get_clock().now().nanoseconds-pending)/1e9
                if self.waiting_tf and 0 <= age < .4:
                    return
            self.latest = msg

    def process(self):
        if self.latest is None:
            return
        msg,self.latest = self.latest,None
        self.waiting_tf = False
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= self.last_stamp or not 0 <= age < .5:
            self.dropped += 1
            return
        if self.calibrated and self.info is not None:
            try:
                self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                self.last_tf_ok = time.monotonic()
            except TransformException:
                if age < .4:
                    self.latest = msg
                    self.waiting_tf = True
                return
        self.last_stamp = stamp
        started = time.monotonic()
        try:
            gray = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')
            corners,ids,_ = (self.detector.detectMarkers(gray) if self.detector else
                            cv2.aruco.detectMarkers(gray,self.dictionary,parameters=self.parameters))
            self.processed += 1
            if ids is None:
                return
            for corner,marker_id in zip(corners,ids.flatten()):
                center = corner[0].mean(axis=0)
                out = MarkerDetection()
                out.header,out.marker_id = msg.header,int(marker_id)
                out.center_x,out.center_y = map(float,center)
                out.image_width,out.image_height = msg.width,msg.height
                self.pub.publish(out)
                self.pose(msg,int(marker_id),corner)
        except (cv2.error,ValueError,TransformException) as exc:
            self.get_logger().warning(str(exc),throttle_duration_sec=2.)
        finally:
            self.latency = age+time.monotonic()-started
            self.latency_pub.publish(Float32(data=float(self.latency)))

    def pose(self,msg,marker_id,corners):
        info = self.info
        if (not self.calibrated or info is None or info.header.frame_id != msg.header.frame_id
                or (info.width,info.height) != (msg.width,msg.height)):
            return
        tf = self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        marker_size=self.sizes.get(marker_id,self.size)
        h = marker_size/2
        obj = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],dtype=np.float64)
        k,d = np.array(info.k).reshape(3,3),np.array(info.d)
        ok,rvec,tvec = cv2.solvePnP(obj,corners.reshape(4,2).astype(np.float64),k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(tvec).all() or tvec[2,0] <= 0:
            return
        projected,_ = cv2.projectPoints(obj,rvec,tvec,k,d)
        error = float(np.sqrt(np.mean((projected.reshape(4,2)-corners.reshape(4,2))**2)))
        if error > 3.:
            return
        rot,pos = tf.transform.rotation,tf.transform.translation
        q = (rot.x,rot.y,rot.z,rot.w)
        offset = rotate(q,tuple(float(x) for x in tvec.flatten()))
        angle = float(np.linalg.norm(rvec))
        rq = tuple(float(x)*math.sin(angle/2)/angle for x in rvec.flatten())+(math.cos(angle/2),) if angle > 1e-9 else (0.,0.,0.,1.)
        orientation = normalize(multiply(q,rq))
        target = LandingTarget()
        target.header.stamp,target.header.frame_id = msg.header.stamp,'map'
        target.marker_id,target.reprojection_error = marker_id,error
        target.pose.position.x,target.pose.position.y,target.pose.position.z = (pos.x+offset[0],pos.y+offset[1],pos.z+offset[2])
        target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w = orientation
        self.pose_pub.publish(target)
        observation=FiducialObservation()
        observation.header=target.header
        observation.marker_id=marker_id
        observation.marker_size=float(marker_size)
        observation.reprojection_error=error
        observation.pose.pose=target.pose
        variance=max(1e-6,(error*.01)**2)
        observation.pose.covariance[0]=variance
        observation.pose.covariance[7]=variance
        observation.pose.covariance[14]=variance*4
        if marker_id in self.package_ids:
            observation.role=FiducialObservation.PACKAGE
        elif marker_id in self.delivery_ids:
            observation.role=FiducialObservation.DELIVERY_PAD
        elif marker_id==self.home_id:
            observation.role=FiducialObservation.HOME_PAD
        else:
            observation.role=FiducialObservation.UNKNOWN
        self.observation_pub.publish(observation)

    def diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='delivery/vision',hardware_id='camera')
        status.level = DiagnosticStatus.OK if self.calibrated and self.info is not None and self.latency < .2 else DiagnosticStatus.WARN
        status.message = 'Image latency and calibration'
        status.values = [KeyValue(key='latency_s',value=str(self.latency)),
                         KeyValue(key='processed',value=str(self.processed)),KeyValue(key='dropped',value=str(self.dropped))]
        array.status = [status]
        self.diag.publish(array)

    def readiness(self):
        age = (self.get_clock().now().nanoseconds-self.last_stamp)/1e9
        self.ready_pub.publish(Bool(data=bool(self.calibrated and self.info is not None
            and 0 <= age < .5 and time.monotonic()-self.last_tf_ok < .5)))


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
