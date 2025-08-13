#!/usr/bin/env python3
"""ROS 2 node that streams AnySkin tactile data.

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
import time

import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from cv_bridge import CvBridge

from anyskin import AnySkinBase

from anyskin_filtering.high_pass_filter import HighPassFilter


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

        default_values = {
            "port": "/dev/ttyACM0",
            "num_mags": 5,
            "baudrate": 115200,
            "burst_mode": True,
            "temp_filtered": True,
            "data_process_rate_hz": 200.0,
            "data_publish_rate_hz": 50.0,
            "image_publish_rate_hz": 30.0,
            "high_pass_filter_cutoff_hz": 0.1,
            "image_size": 256,
        }

        for param, value in default_values.items():
            self.declare_parameter(param, value)

        self.sensor = AnySkinBase(
            num_mags=self.get_parameter("num_mags").get_parameter_value().integer_value,
            port=self.get_parameter("port").get_parameter_value().string_value,
            baudrate=self.get_parameter("baudrate").get_parameter_value().integer_value,
            burst_mode=self.get_parameter("burst_mode")
            .get_parameter_value()
            .bool_value,
            temp_filtered=self.get_parameter("temp_filtered")
            .get_parameter_value()
            .bool_value,
        )
        self.num_magnets = self.sensor.num_mags
        self.num_dimensions = 3

        self.image_size = (
            self.get_parameter("image_size").get_parameter_value().integer_value
        )

        # Pre-compute meshgrid for efficiency
        x = np.linspace(0, self.image_size - 1, self.image_size)
        y = np.linspace(0, self.image_size - 1, self.image_size)
        self.X, self.Y = np.meshgrid(x, y)
        # Define sensor positions: center, left, right, top, bottom
        self.positions = [
            (self.image_size // 2, self.image_size // 2),  # center
            (self.image_size // 4, self.image_size // 2),  # left
            (3 * self.image_size // 4, self.image_size // 2),  # right
            (self.image_size // 2, self.image_size // 4),  # top
            (self.image_size // 2, 3 * self.image_size // 4),  # bottom
        ]

        self._data_publisher = self.create_publisher(
            Float32MultiArray, "tactile_data", 10
        )

        self._cv_bridge = CvBridge()

        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "camera_info", 10
        )
        self._camera_info = CameraInfo()
        self._camera_info.header.frame_id = "anyskin_camera"
        self._camera_info.width = self.image_size
        self._camera_info.height = self.image_size
        self._camera_info.distortion_model = "plumb_bob"
        self._camera_info.d = [0.0] * 5  # Distortion coefficients
        self._camera_info.k = [
            1.0,
            0.0,
            self.image_size / 2.0,  # fx, skew, cx
            0.0,
            1.0,
            self.image_size / 2.0,  # fy, cy
            0.0,
            0.0,
            1.0,  # s, 0, 1
        ]
        self.high_pass_filter = HighPassFilter(
            n_channels=self.sensor.num_mags * self.num_dimensions,
            cutoff_frequency=self.get_parameter("high_pass_filter_cutoff_hz")
            .get_parameter_value()
            .double_value,
            sampling_rate=self.get_parameter("data_process_rate_hz")
            .get_parameter_value()
            .double_value,
        )

        self._image_publisher = self.create_publisher(Image, "image_raw", 10)
        self._data_process_timer = self.create_timer(
            1.0
            / self.get_parameter("data_process_rate_hz")
            .get_parameter_value()
            .double_value,
            self._callback_process_data,
        )

        self._data_timer = self.create_timer(
            1.0
            / self.get_parameter("data_publish_rate_hz")
            .get_parameter_value()
            .double_value,
            self._callback_publish_data,
        )
        self._image_timer = self.create_timer(
            1.0
            / self.get_parameter("image_publish_rate_hz")
            .get_parameter_value()
            .double_value,
            self._callback_publish_image,
        )

        self.data_magnitude = np.zeros(self.sensor.num_mags, dtype=np.float32)
        self.image = np.zeros((self.image_size, self.image_size, 3), dtype=np.float32)

        self._baseline = None

        self.get_logger().info("Calibrating AnySkin sensor to zero...")

        self.calibrate_to_zero(num_samples=10, sample_rate=10.0)

        self.get_logger().info("AnySkin publisher started.")

    def _callback_publish_image(self):
        """Publish the processed image of the anyskin sensor."""
        try:
            img_uint8 = np.clip((self.image * 256), 0, 255).astype(np.uint8)

            # Create random noise to test
            # img_uint8 = np.random.randint(
            #     0, 256, (self.image_size, self.image_size), dtype=np.uint8
            # )

            img_msg = self._cv_bridge.cv2_to_imgmsg(img_uint8, encoding="rgb8")
            stamp = self.get_clock().now().to_msg()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "anyskin_sensor"

            self._camera_info.header.stamp = stamp
            self._camera_info.header.frame_id = "anyskin_sensor"

            self._image_publisher.publish(img_msg)
            self._camera_info_publisher.publish(self._camera_info)
        except Exception as exc:
            self.get_logger().warn(f"Image publish failed: {exc}")

    def _callback_publish_data(self):
        """Publish the data of the anyskin sensor."""
        try:
            msg = Float32MultiArray()
            msg.data = self.data_magnitude.astype(float).tolist()

            self._data_publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Data publish failed: {exc}")

    def _callback_process_data(self):
        """Publish the data of the anyskin sensor."""
        try:
            _, data = self.sensor.get_sample()
            data = data.reshape(-1, 3)

            if self._baseline is not None:
                # Apply filtering to the raw difference data (per axis)
                diff_raw = data - self._baseline
                # Reshape for filter: (n_channels, n_samples) where n_samples=1 for single sample
                diff_reshaped = diff_raw.flatten().reshape(-1, 1)
                filtered_flat = self.high_pass_filter.apply(diff_reshaped).flatten()
                diff = filtered_flat.reshape(-1, self.num_dimensions)

                # Get processed images for each axis (returns positive/negative channels)
                x_channels = self.get_processed_image(diff[:, 0])  # Shape: (H, W, 2)
                y_channels = self.get_processed_image(diff[:, 1])
                z_channels = self.get_processed_image(diff[:, 2])

                # Map to RGB: Red = positive, Blue = negative, Green = combination
                self.image[:, :, 0] = x_channels[:, :, 0]  # X-axis positive (red)
                self.image[:, :, 1] = (
                    y_channels[:, :, 0] + z_channels[:, :, 0]
                ) * 0.5  # Y+Z positive (green)
                self.image[:, :, 2] = (
                    x_channels[:, :, 1] + y_channels[:, :, 1] + z_channels[:, :, 1]
                )  # All negative (blue)

                self.data_magnitude = np.linalg.norm(diff, axis=1)
            else:
                self.data_magnitude = np.zeros(self.sensor.num_mags, dtype=np.float32)

        except Exception as exc:
            self.get_logger().warn(f"Sensor read failed: {exc}")
            raise exc

    def destroy_node(self):
        """Inform the user that the node is being destroyed."""
        self.get_logger().info("AnySkin publisher stopped.")
        super().destroy_node()

    def get_processed_image(self, sensor_values: np.ndarray) -> np.ndarray:
        """Generate a 2D Gaussian intensity image from the sensor values.

        Positive values are visualized in red, negative values in blue.

        Returns:
            np.ndarray: A 2D array representing the Gaussian intensity image (image_size x image_size, 2).
            Returns [positive_channel, negative_channel] for red/blue visualization.
        """
        positive_image = np.zeros((self.image_size, self.image_size))
        negative_image = np.zeros((self.image_size, self.image_size))

        # Create Gaussian for each sensor
        for i, (pos_x, pos_y) in enumerate(self.positions):
            if i < len(sensor_values):
                sensor_val = sensor_values[i]

                # Map absolute sensor value to standard deviation (1 to 50 pixels)
                min_old, max_old, min_new, max_new = 0.0, 400.0, 1.0, 50.0
                std_dev = (max_new - min_new) / (max_old - min_old) * (
                    np.clip(abs(sensor_val), min_old, max_old) - min_old
                ) + min_new

                # Scale magnitude of Gaussian based on absolute value
                min_old, max_old, min_new, max_new = 0.0, 400.0, 0.0, 1.0
                magnitude = (max_new - min_new) / (max_old - min_old) * (
                    np.clip(abs(sensor_val), min_old, max_old) - min_old
                ) + min_new

                # Create individual Gaussian
                gaussian = magnitude * np.exp(
                    -((self.X - pos_x) ** 2 + (self.Y - pos_y) ** 2) / (2 * std_dev**2)
                )

                # Scale by sensor value intensity (0-1 range)
                intensity = np.clip(abs(sensor_val) / 30.0, 0, 1)
                weighted_gaussian = gaussian * intensity

                # Add to appropriate channel based on sign
                if sensor_val >= 0:
                    positive_image += weighted_gaussian
                else:
                    negative_image += weighted_gaussian

        # Stack positive and negative channels
        return np.stack([positive_image, negative_image], axis=-1)

    def calibrate_to_zero(self, num_samples: int = 10, sample_rate: float = 10.0):
        """Calibrate the sensor to zero.

        This function computes the average of a number of samples to compute the baseline.
        The value is then normalized by this average with the formula:

            calibrated_value = value - average(samples)

        Args:
            num_samples (int): Number of samples to take for calibration.
            sample_rate (float): Rate at which to take samples in Hz.
        """
        samples = np.zeros(
            (num_samples, self.num_magnets, self.num_dimensions), dtype=np.float32
        )

        for sample_num in range(num_samples):
            _, data = self.sensor.get_sample()
            data = data.reshape(-1, 3)  # Reshape to (num_magnets, num_dimensions)
            self.get_logger().info(
                f"Calibrating sample {sample_num + 1}/{num_samples}: {data}"
            )

            # sample = np.linalg.norm(data, axis=1)
            # samples[sample_num] = sample
            samples[sample_num] = data
            time.sleep(1.0 / sample_rate)

        self._baseline = np.mean(
            samples, axis=0
        )  # Shape: (num_magnets, num_dimensions)
        self.get_logger().info(f"Calibration complete. Baseline: {self._baseline}")


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
