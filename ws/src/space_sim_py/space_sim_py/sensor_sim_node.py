import rclpy
from rclpy.node import Node
import numpy as np

from space_msgs.msg import RelativeState
from .sensor_models import noisy_position_measurement


class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')

        self.subscription = self.create_subscription(
            RelativeState,
            '/truth_state',
            self.truth_callback,
            10
        )

        self.publisher_ = self.create_publisher(RelativeState, '/sensor/relative_measurement', 10)
        self.sigma = 0.05

        self.get_logger().info('Sensor simulator started')

    def truth_callback(self, msg: RelativeState):
        state = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz], dtype=float)
        meas = noisy_position_measurement(state, sigma=self.sigma)

        out = RelativeState()
        out.stamp = self.get_clock().now().to_msg()
        out.x = float(meas[0])
        out.y = float(meas[1])
        out.z = float(meas[2])
        out.vx = 0.0
        out.vy = 0.0
        out.vz = 0.0

        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
