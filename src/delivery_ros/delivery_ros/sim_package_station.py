"""SITL-only package-station scanner; replaces a fixed physical pickup camera."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from delivery_interfaces.msg import CompetitionStatus,FiducialObservation,FlightState,PayloadState


class SimPackageStation(Node):
    def __init__(self):
        super().__init__('sim_package_station')
        self.enabled=self.declare_parameter('enabled',False).value
        self.package_ids=list(self.declare_parameter(
            'package_marker_ids',Parameter.Type.INTEGER_ARRAY).value or [20,21,22])
        if len(self.package_ids)!=3:
            raise ValueError('Table C simulation requires three package marker IDs')
        self.status=None; self.flight=None; self.payload=None
        self.pub=self.create_publisher(FiducialObservation,'/delivery/fiducials',10)
        self.create_subscription(CompetitionStatus,'/competition/status',lambda m:setattr(self,'status',m),10)
        self.create_subscription(FlightState,'/delivery/flight_state',lambda m:setattr(self,'flight',m),10)
        self.create_subscription(PayloadState,'/delivery/payload_state',lambda m:setattr(self,'payload',m),10)
        self.create_timer(.1,self.tick)

    def tick(self):
        s,f,p=self.status,self.flight,self.payload
        if not (self.enabled and s and f and p and s.state==CompetitionStatus.SCAN_PACKAGE
                and f.healthy and f.landed and not f.armed and p.healthy and p.grab_ready):
            return
        msg=FiducialObservation(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='pickup_station'; msg.role=FiducialObservation.PACKAGE
        msg.marker_id=int(self.package_ids[min(int(s.package_index),2)])
        msg.marker_size=.08; msg.pose.pose.orientation.w=1.; msg.reprojection_error=.1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=SimPackageStation()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
