#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport

import numpy as np

AMPLITUDE = 0.2 # norm
FREQUENCY = 0.5 # Hz


class SteeringTester(Node):

    def __init__(self):
        super().__init__("steering_tester")

        self.control_mode_sub_ = self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.control_mode_callback,
            10,
        )

        self.control_command_pub_ = self.create_publisher(
            Control, "/control/command/control_cmd", 10
        )

        self.dt = 0.01
        self.control_send_ = self.create_timer(self.dt, self.control_timer)
        
        self.t0 = self.get_clock().now().nanoseconds

        self.control_mode_ = ControlModeReport()
        
        self.reference = 0

    def control_mode_callback(self, msg):
        self.control_mode_ = msg

    def control_timer(self):

        t = (self.get_clock().now().nanoseconds - self.t0)*1e-9

        control_command_msg = Control()

        # * Assembly the code

        control_command_msg.lateral.steering_tire_angle = AMPLITUDE * 0.4410 * np.sin(2*np.pi * FREQUENCY * t)

        self.control_command_pub_.publish(control_command_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SteeringTester())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
