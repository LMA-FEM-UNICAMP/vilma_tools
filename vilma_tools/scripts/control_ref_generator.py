#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport

import numpy as np

REFERENCES = [5.0, 10.0, 15.0, 20.0, 25.0, 15.0, 0.0]

# TIME_STEP = 1.6 # 2.5 m/s²
TIME_STEP = 5
UPDATE_REF = 15.0


class ControlRefGenerator(Node):

    def __init__(self):
        super().__init__("control_ref_generator")

        self.control_mode_sub_ = self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.control_mode_callback,
            10,
        )

        self.control_command_pub_ = self.create_publisher(
            Control, "/control/command/control_cmd", 10
        )

        self.dt = 0.1
        self.control_send_ = self.create_timer(self.dt, self.control_timer)

        self.control_mode_ = ControlModeReport()

        self.t = 0

        self.i = 0
        self.inc = 1

        self.reference = 0.0

        # Complementary filter

        self.reference_filtered = 0.0

        self.tau = TIME_STEP / np.log(100)  # 1% settling criterion

        self.alpha = self.dt / (self.tau + self.dt)

    def control_mode_callback(self, msg):
        self.control_mode_ = msg

    def control_timer(self):

        if (self.get_clock().now().nanoseconds - self.t) * 1e-9 >= UPDATE_REF:
            self.t = self.get_clock().now().nanoseconds
            self.reference = REFERENCES[self.i]

            self.i = self.i + self.inc

            if self.i == 0:
                self.inc = 1
            elif self.i == (len(REFERENCES) - 1):
                self.inc = -1

            self.get_logger().info("New reference: " + str(self.reference) + " km/h")

        if self.control_mode_.mode == 4:
            self.i = 0
            self.inc = 1
            self.t = self.get_clock().now().nanoseconds
            self.reference_filtered = 0.0

        # * Calculate the reference

        self.reference_filtered = self.reference_filtered + self.alpha * (
            self.reference - self.reference_filtered
        )

        control_command_msg = Control()

        # * Assembly the code

        control_command_msg.longitudinal.velocity = self.reference_filtered / 3.6

        self.control_command_pub_.publish(control_command_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlRefGenerator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
