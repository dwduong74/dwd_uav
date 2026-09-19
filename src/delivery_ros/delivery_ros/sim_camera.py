"""Map Gazebo acquisition times into the same DDS clock as PX4 odometry."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from px4_msgs.msg import TimesyncStatus


class SimCameraClock(Node):
    def __init__(self):
        super().__init__('sim_camera_clock')
        self.last = {}
        self.offset_us = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status',
                                 self.on_timesync, qos)
        for name, cls in [('image_raw', Image), ('camera_info', CameraInfo), ('scan', LaserScan)]:
            pub = self.create_publisher(cls, '/camera/' + name, qos)
            self.create_subscription(cls, '/gazebo/camera/' + name,
                lambda msg, name=name, pub=pub: self.forward(name, pub, msg), qos)

    def on_timesync(self, msg):
        if msg.source_protocol == TimesyncStatus.SOURCE_PROTOCOL_DDS:
            self.offset_us = msg.estimated_offset

    def forward(self, name, pub, msg):
        if self.offset_us is None:
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if name in self.last and stamp <= self.last[name]:
            return
        self.last[name] = stamp
        # PX4 DDS serialization adds -estimated_offset to simulator microseconds.
        # Preserve acquisition time; receipt time can be ahead of delayed odometry.
        mapped = stamp - int(self.offset_us) * 1000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(mapped, 1000000000)
        if name == 'scan' and msg.ranges:
            center = len(msg.ranges) // 2
            msg.ranges = [msg.ranges[center]]
            if msg.intensities:
                msg.intensities = [msg.intensities[center]]
            msg.angle_min = msg.angle_max = msg.angle_increment = 0.
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
