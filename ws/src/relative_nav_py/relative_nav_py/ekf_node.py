import rclpy
from rclpy.node import Node
import numpy as np

from space_msgs.msg import RelativeState
from .ekf import RelativeEKF


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        self.subscription = self.create_subscription(
            RelativeState,
            '/sensor/relative_measurement',
            self.measurement_callback,
            10
        )

        self.publisher_ = self.create_publisher(RelativeState, '/nav/estimated_state', 10)
        self.filter = RelativeEKF(dt=0.1)

        self.get_logger().info('EKF node started')

    def measurement_callback(self, msg: RelativeState):
        z = np.array([msg.x, msg.y, msg.z], dtype=float)

        self.filter.predict()
        self.filter.update(z)

        xhat = self.filter.state_vector()

        out = RelativeState()
        out.stamp = self.get_clock().now().to_msg()
        out.x = float(xhat[0])
        out.y = float(xhat[1])
        out.z = float(xhat[2])
        out.vx = float(xhat[3])
        out.vy = float(xhat[4])
        out.vz = float(xhat[5])

        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
