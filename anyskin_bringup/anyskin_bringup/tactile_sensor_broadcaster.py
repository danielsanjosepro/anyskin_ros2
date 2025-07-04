#!/usr/bin/env python3
"""
ROS 2 node that streams AnySkin tactile data.

‣ Topic           : /anyskin_data   (std_msgs/msg/Float32MultiArray)
‣ QoS             : depth 10 (best-effort fits high-rate streaming)
‣ Parameters      :
      port              (/dev/ttyACM0)
      num_mags          (5)
      baudrate          (115200)
      burst_mode        (true)
      temp_filtered     (true)
      use_dummy         (false)    # handy for development w/o hardware
      publish_rate_hz   (200.0)    # 0 ⇒ “publish as fast as data arrives”

If the return signature of `get_sample()` ever changes, the helper
`_unpack_sample()` keeps the rest of the code working.
"""

from __future__ import annotations

import numpy
import rclpy

# import rclpy.qos import qos_
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from anyskin import AnySkinBase


class AnySkinPublisher(Node):
    """ROS 2 node that streams AnySkin tactile data.

    ‣ Topic           : /tactile_data   (std_msgs/msg/Float32MultiArray)
    ‣ QoS             : depth 10 (best-effort fits high-rate streaming)
    ‣ Parameters      :
          port              (/dev/ttyACM0)
          num_mags          (5)
          baudrate          (115200)
          burst_mode        (true)
          temp_filtered     (true)
          use_dummy         (false)    # handy for development w/o hardware
          publish_rate_hz   (200.0)    # 0 ⇒ “publish as fast as data arrives”
    """

    def __init__(self):
        super().__init__("anyskin_publisher")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("num_mags", 5)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("burst_mode", True)
        self.declare_parameter("temp_filtered", True)
        self.declare_parameter("publish_rate_hz", 100.0)

        port = self.get_parameter("port").value
        num_mags = self.get_parameter("num_mags").value
        baudrate = self.get_parameter("baudrate").value
        burst_mode = self.get_parameter("burst_mode").value
        temp_filtered = self.get_parameter("temp_filtered").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self.sensor = AnySkinBase(
            num_mags=num_mags,
            port=port,
            baudrate=baudrate,
            burst_mode=burst_mode,
            temp_filtered=temp_filtered,
        )

        self._data_publisher = self.create_publisher(
            Float32MultiArray, "tactile_data", 10
        )

        if publish_rate_hz:
            self._timer = self.create_timer(
                1.0 / publish_rate_hz, self._callback_publish_data
            )

        self.get_logger().info("AnySkin publisher started.")

    def _callback_publish_data(self):
        """Publish the data of the anyskin sensor."""
        try:
            _, data = self.sensor.get_sample()
            data = data.reshape(-1, 3)
            data_magnitude = numpy.linalg.norm(data, axis=1)

            msg = Float32MultiArray()
            msg.data = data_magnitude.astype(float).tolist()

            self._data_publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Sensor read failed: {exc}")

    def destroy_node(self):
        """Inform the user that the node is being destroyed."""
        self.get_logger().info("AnySkin publisher stopped.")
        super().destroy_node()


def main():
    rclpy.init()
    node = AnySkinPublisher()
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.001)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
