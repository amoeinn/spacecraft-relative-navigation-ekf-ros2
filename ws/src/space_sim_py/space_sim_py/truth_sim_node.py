import rclpy
from rclpy.node import Node
import numpy as np

from space_msgs.msg import RelativeState
from .dynamics import propagate_state


class TruthSimNode(Node):
    def __init__(self):
        super().__init__('truth_sim_node')
        self.publisher_ = self.create_publisher(RelativeState, '/truth_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.dt = 0.1
        self.state = np.array([5.0, -2.0, 1.0, -0.02, 0.01, 0.0], dtype=float)

        self.get_logger().info('Truth simulator started')

    def timer_callback(self):
        self.state = propagate_state(self.state, self.dt)

        msg = RelativeState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.x = float(self.state[0])
        msg.y = float(self.state[1])
        msg.z = float(self.state[2])
        msg.vx = float(self.state[3])
        msg.vy = float(self.state[4])
        msg.vz = float(self.state[5])

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TruthSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
