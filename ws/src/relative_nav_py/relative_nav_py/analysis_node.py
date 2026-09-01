"""Records truth, measurement and estimate, then reports RMSE and plots.

The three topics are published independently by three nodes, so samples
arrive interleaved rather than as matched triples. Comparing whatever
happened to arrive most recently would fold the pipeline's latency into the
error, so samples are paired by nearest timestamp instead.

Only position is compared. The sensor publishes position measurements and
zeroes the velocity fields, so a velocity comparison against the
measurement would be meaningless.

Usage:
    ros2 run relative_nav_py analysis_node
    ros2 run relative_nav_py analysis_node --ros-args -p duration:=30.0
"""

import bisect

import numpy as np
import rclpy
from rclpy.node import Node

from space_msgs.msg import RelativeState


def stamp_seconds(msg) -> float:
    return msg.stamp.sec + msg.stamp.nanosec * 1e-9


def position(msg) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z], dtype=float)


class AnalysisNode(Node):
    """Collects the three streams and reports how much the EKF helped."""

    def __init__(self):
        super().__init__('analysis_node')

        self.declare_parameter('duration', 20.0)
        self.declare_parameter('output', 'docs/results/ekf_result.png')
        self.duration = self.get_parameter('duration').value
        self.output = self.get_parameter('output').value

        self.truth = []
        self.measured = []
        self.estimated = []

        self.create_subscription(RelativeState, '/truth_state',
                                 self._on_truth, 100)
        self.create_subscription(RelativeState, '/sensor/relative_measurement',
                                 self._on_measurement, 100)
        self.create_subscription(RelativeState, '/nav/estimated_state',
                                 self._on_estimate, 100)

        self.timer = self.create_timer(self.duration, self._finish)
        self.get_logger().info(
            f'Analysis node started, recording for {self.duration:.0f} s')

    def _on_truth(self, msg):
        self.truth.append((stamp_seconds(msg), position(msg)))

    def _on_measurement(self, msg):
        self.measured.append((stamp_seconds(msg), position(msg)))

    def _on_estimate(self, msg):
        self.estimated.append((stamp_seconds(msg), position(msg)))

    # ------------------------------------------------------------ pairing

    @staticmethod
    def _nearest(series, when):
        """The sample in series closest in time to when.

        series must be sorted by timestamp, which it is: messages arrive in
        publication order on each topic.
        """
        times = [t for t, _ in series]
        index = bisect.bisect_left(times, when)
        candidates = []
        if index > 0:
            candidates.append(series[index - 1])
        if index < len(series):
            candidates.append(series[index])
        return min(candidates, key=lambda pair: abs(pair[0] - when))

    def _aligned(self):
        """Truth, measurement and estimate sampled at the estimate times."""
        if not (self.truth and self.measured and self.estimated):
            return None

        times, truth, measured, estimated = [], [], [], []
        for when, estimate in self.estimated:
            times.append(when)
            truth.append(self._nearest(self.truth, when)[1])
            measured.append(self._nearest(self.measured, when)[1])
            estimated.append(estimate)

        return (np.asarray(times), np.asarray(truth),
                np.asarray(measured), np.asarray(estimated))

    # ------------------------------------------------------------- report

    def _finish(self):
        self.timer.cancel()

        aligned = self._aligned()
        if aligned is None:
            self.get_logger().error(
                'not enough data on all three topics, is the pipeline running?')
            rclpy.shutdown()
            return

        times, truth, measured, estimated = aligned
        times = times - times[0]

        measurement_error = np.linalg.norm(measured - truth, axis=1)
        estimate_error = np.linalg.norm(estimated - truth, axis=1)

        measurement_rmse = float(np.sqrt(np.mean(measurement_error ** 2)))
        estimate_rmse = float(np.sqrt(np.mean(estimate_error ** 2)))
        improvement = 1.0 - estimate_rmse / measurement_rmse

        self.get_logger().info(f'samples: {len(times)}')
        self.get_logger().info(f'measurement RMSE: {measurement_rmse:.4f} m')
        self.get_logger().info(f'EKF estimate RMSE: {estimate_rmse:.4f} m')
        self.get_logger().info(f'improvement: {improvement:.1%}')

        self._plot(times, truth, measured, estimated,
                   measurement_rmse, estimate_rmse, improvement)
        rclpy.shutdown()

    def _plot(self, times, truth, measured, estimated,
              measurement_rmse, estimate_rmse, improvement):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from pathlib import Path

        fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))

        axes[0].plot(times, measured[:, 0], color='#b0b0b0', linewidth=0.9,
                     label='measurement')
        axes[0].plot(times, truth[:, 0], color='#1a1a1a', linewidth=1.8,
                     label='truth')
        axes[0].plot(times, estimated[:, 0], color='#c4523a', linewidth=1.6,
                     label='EKF estimate')
        axes[0].set_xlabel('time (s)')
        axes[0].set_ylabel('x position (m)')
        axes[0].set_title('Relative position, x axis')
        axes[0].legend(fontsize=9)
        axes[0].grid(alpha=0.25, linewidth=0.5)

        axes[1].plot(times, np.linalg.norm(measured - truth, axis=1),
                     color='#3a76c4', linewidth=1.1, label='measurement')
        axes[1].plot(times, np.linalg.norm(estimated - truth, axis=1),
                     color='#c4523a', linewidth=1.4, label='EKF estimate')
        axes[1].set_xlabel('time (s)')
        axes[1].set_ylabel('position error (m)')
        axes[1].set_title(
            f'Error against truth\n'
            f'RMSE {measurement_rmse:.4f} to {estimate_rmse:.4f} m, '
            f'{improvement:.0%} better')
        axes[1].legend(fontsize=9)
        axes[1].grid(alpha=0.25, linewidth=0.5)

        plt.tight_layout()
        output = Path(self.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output, dpi=150, bbox_inches='tight', facecolor='white')
        self.get_logger().info(f'saved {output.resolve()}')


def main(args=None):
    rclpy.init(args=args)
    node = AnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()