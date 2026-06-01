#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation


class AverageOrientation(Node):

    def __init__(self):
        super().__init__("average_orientation")

        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            100
        )

        self.quaternions = []
        self.reference_quaternion = None

        self.get_logger().info("Waiting for IMU messages...")

    def imu_callback(self, msg: Imu):

        q = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ], dtype=np.float64)

        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return

        q /= norm

        if self.reference_quaternion is None:
            self.reference_quaternion = q.copy()

        # Ensure all quaternions are on the same hemisphere.
        if np.dot(q, self.reference_quaternion) < 0.0:
            q = -q

        self.quaternions.append(q)

    def print_average_orientation(self):

        if len(self.quaternions) == 0:
            self.get_logger().warning("No IMU messages received.")
            return

        q_sum = np.sum(self.quaternions, axis=0)

        q_avg = q_sum / np.linalg.norm(q_sum)

        r = Rotation.from_quat(q_avg)

        roll, pitch, yaw = r.as_euler("xyz", degrees=True)

        print("\n========== RESULTS ==========")
        print(f"Samples: {len(self.quaternions)}")
        print(
            f"Average quaternion [x y z w]: "
            f"{q_avg}"
        )
        print(f"Average roll  (deg): {roll:.6f}")
        print(f"Average pitch (deg): {pitch:.6f}")
        print(f"Average yaw   (deg): {yaw:.6f}")
        print("=============================\n")


def main(args=None):

    rclpy.init(args=args)

    node = AverageOrientation()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.print_average_orientation()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()