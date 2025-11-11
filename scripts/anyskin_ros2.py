#!/usr/bin/env python3
"""ROS 2 node that streams AnySkin tactile data with visualization.

‣ Topics:
    /tactile_data        (std_msgs/msg/Float32MultiArray)
                         Default: 15 values [x, y, mag] per magnet (rotated to match viz)
                         With --all_values: 15 values [x, y, z] per magnet (raw sensor data)
    /image_raw           (sensor_msgs/msg/Image)
    /camera_info         (sensor_msgs/msg/CameraInfo)
‣ Services:
    /reset               (std_srvs/srv/Trigger)
‣ QoS             : depth 10 (best-effort fits high-rate streaming)
‣ Command-line Arguments:
      --port                    (/dev/ttyACM0)
      --num_mags                (5)
      --baudrate                (115200)
      --burst_mode              (True)
      --temp_filtered           (True)
      --namespace               (None)
      --data_publish_rate_hz    (50.0)
      --image_publish_rate_hz   (30.0)
      --image_width             (400)
      --image_height            (400)
      --viz_mode                (3axis)
      --scaling                 (7.0)
      --all_values              (False)  # If set, publishes raw x,y,z values instead of rotated x,y + magnitude
"""

from __future__ import annotations
import argparse
import time
import os

import numpy as np
import cv2
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from anyskin import AnySkinBase


class AnySkinImagePublisher(Node):
    """ROS 2 node that streams AnySkin tactile data with visualization.

    ‣ Topics:
        /tactile_data        (std_msgs/msg/Float32MultiArray)
                             Default: 15 values [x, y, mag] per magnet (rotated to match viz)
                             With --all_values: 15 values [x, y, z] per magnet (raw sensor data)
        /image_raw           (sensor_msgs/msg/Image)
        /camera_info         (sensor_msgs/msg/CameraInfo)
    ‣ Services:
        /reset               (std_srvs/srv/Trigger)
    ‣ Command-line Arguments:
          --port                    (/dev/ttyACM0)
          --num_mags                (5)
          --baudrate                (115200)
          --burst_mode              (True)
          --temp_filtered           (True)
          --namespace               (None)
          --data_publish_rate_hz    (50.0)
          --image_publish_rate_hz   (30.0)
          --image_width             (400)
          --image_height            (400)
          --viz_mode                (3axis)
          --scaling                 (7.0)
          --all_values              (False)
    """

    def __init__(self, args):
        super().__init__(
            "anyskin_image_publisher",
            namespace=args.namespace if args.namespace else None,
        )

        self.port = args.port
        self.num_mags = args.num_mags
        self.baudrate = args.baudrate
        self.burst_mode = args.burst_mode
        self.temp_filtered = args.temp_filtered
        self.data_publish_rate_hz = args.data_publish_rate_hz
        self.image_publish_rate_hz = args.image_publish_rate_hz
        self.desired_width = args.image_width
        self.desired_height = args.image_height
        self.viz_mode = args.viz_mode
        self.scaling = args.scaling
        self.all_values = args.all_values

        self.sensor = AnySkinBase(
            num_mags=self.num_mags,
            port=self.port,
            baudrate=self.baudrate,
            burst_mode=self.burst_mode,
            temp_filtered=self.temp_filtered,
        )
        self.num_magnets = self.sensor.num_mags
        self.num_dimensions = 3

        # Load background image
        dir_path = os.path.dirname(os.path.realpath(__file__))
        bg_image_path = os.path.join(dir_path, "images/viz_bg.png")

        if os.path.exists(bg_image_path):
            self.bg_image = cv2.imread(bg_image_path)
            self.bg_image = cv2.cvtColor(self.bg_image, cv2.COLOR_BGR2RGB)
            self.bg_image = cv2.resize(
                self.bg_image, (self.desired_width, self.desired_height)
            )
        else:
            self.bg_image = (
                np.ones((self.desired_height, self.desired_width, 3), dtype=np.uint8)
                * 234
            )
            self.get_logger().warn(
                f"Background image not found at {bg_image_path}, using default background"
            )

        self.image_height, self.image_width = self.bg_image.shape[:2]

        # Original chip locations designed for 400x400 image
        original_chip_locations = np.array(
            [
                [204, 222],  # center
                [130, 222],  # left
                [279, 222],  # right
                [204, 157],  # up
                [204, 290],  # down
            ]
        )
        # Scale chip locations based on image dimensions
        scale_x = self.image_width / 400.0
        scale_y = self.image_height / 400.0
        self.chip_locations = original_chip_locations * np.array([scale_x, scale_y])
        self.chip_locations = self.chip_locations.astype(int)

        self.chip_xy_rotations = np.array(
            [-np.pi / 2, -np.pi / 2, np.pi, np.pi / 2, 0.0]
        )

        self._data_publisher = self.create_publisher(
            Float32MultiArray, "tactile_data", 10
        )

        self._cv_bridge = CvBridge()

        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "camera_info", 10
        )
        self._camera_info = CameraInfo()
        self._camera_info.header.frame_id = "anyskin_camera"
        self._camera_info.width = self.image_width
        self._camera_info.height = self.image_height
        self._camera_info.distortion_model = "plumb_bob"
        self._camera_info.d = [0.0] * 5  # Distortion coefficients
        self._camera_info.k = [
            1.0,
            0.0,
            self.image_width / 2.0,  # fx, skew, cx
            0.0,
            1.0,
            self.image_height / 2.0,  # fy, cy
            0.0,
            0.0,
            1.0,  # s, 0, 1
        ]

        self._image_publisher = self.create_publisher(Image, "image_raw", 10)

        self._data_timer = self.create_timer(
            1.0 / self.data_publish_rate_hz,
            self._callback_publish_data,
        )
        self._image_timer = self.create_timer(
            1.0 / self.image_publish_rate_hz,
            self._callback_publish_image,
        )

        # Create service for resetting baseline
        self._reset_baseline_service = self.create_service(
            Trigger, "reset", self._callback_reset_baseline
        )

        self.broadcast_size = int(
            self.sensor.num_mags * 3  # x, y, magnitude per magnet
            if not self.all_values
            else self.sensor.num_mags * self.num_dimensions,
        )
        self.data_magnitude = np.zeros(self.broadcast_size, dtype=np.float32)
        self.current_sensor_data = np.zeros(
            (self.num_magnets, self.num_dimensions), dtype=np.float32
        )

        self._baseline = None

        self.get_logger().info("Calibrating AnySkin sensor to zero...")
        self.calibrate_to_zero(num_samples=10, sample_rate=10.0)
        self.get_logger().info("AnySkin image publisher started.")

    def _visualize_data(self, data):
        """Render sensor data visualization to an image array.

        Args:
            data: Sensor data array of shape (num_mags, 3)

        Returns:
            np.ndarray: RGB image array
        """
        # Start with a copy of the background image
        img = self.bg_image.copy()

        data = data.reshape(-1, 3)
        data_mag = np.linalg.norm(data, axis=1)

        # Draw the chip locations
        for magid, chip_location in enumerate(self.chip_locations):
            if magid >= len(data):
                continue

            if self.viz_mode == "magnitude":
                # Draw circle with magnitude
                radius = int(data_mag[magid] / self.scaling)
                if radius > 0:
                    cv2.circle(img, tuple(chip_location), radius, (255, 83, 72), -1)

            elif self.viz_mode == "3axis":
                radius = int(np.abs(data[magid, -1]) / self.scaling)
                if radius > 0:
                    if data[magid, -1] < 0:
                        cv2.circle(img, tuple(chip_location), radius, (255, 0, 0), 2)
                    else:
                        cv2.circle(img, tuple(chip_location), radius, (255, 0, 0), -1)

                arrow_start = chip_location
                rotation_mat = np.array(
                    [
                        [
                            np.cos(self.chip_xy_rotations[magid]),
                            -np.sin(self.chip_xy_rotations[magid]),
                        ],
                        [
                            np.sin(self.chip_xy_rotations[magid]),
                            np.cos(self.chip_xy_rotations[magid]),
                        ],
                    ]
                )
                data_xy = np.dot(rotation_mat, data[magid, :2])
                arrow_end = (
                    int(chip_location[0] + data_xy[0] / self.scaling),
                    int(chip_location[1] + data_xy[1] / self.scaling),
                )
                cv2.line(img, tuple(arrow_start), arrow_end, (0, 255, 0), 2)

        return img

    def _callback_publish_image(self):
        """Publish the visualization image."""
        try:
            img = self._visualize_data(self.current_sensor_data)

            img_msg = self._cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
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
            _, data = self.sensor.get_sample()
            data = data.reshape(-1, 3)

            if self._baseline is not None:
                diff = data - self._baseline
                self.current_sensor_data = diff

                if not self.all_values:
                    # Publish rotated x, y vectors + magnitude (3 values per magnet)
                    vector_data = []
                    for magid in range(len(diff)):
                        # Rotate x,y components to match visualization
                        rotation_mat = np.array(
                            [
                                [
                                    np.cos(self.chip_xy_rotations[magid]),
                                    -np.sin(self.chip_xy_rotations[magid]),
                                ],
                                [
                                    np.sin(self.chip_xy_rotations[magid]),
                                    np.cos(self.chip_xy_rotations[magid]),
                                ],
                            ]
                        )
                        rotated_xy = np.dot(rotation_mat, diff[magid, :2])
                        magnitude = np.linalg.norm(diff[magid])

                        vector_data.extend([rotated_xy[0], rotated_xy[1], magnitude])

                    self.data_magnitude = np.array(vector_data, dtype=np.float32)
                else:
                    self.data_magnitude = diff.flatten()
            else:
                self.data_magnitude = np.zeros(self.broadcast_size, dtype=np.float32)
                self.current_sensor_data = np.zeros(
                    (self.num_magnets, self.num_dimensions), dtype=np.float32
                )

            msg = Float32MultiArray()
            msg.data = self.data_magnitude.astype(float).tolist()
            if not self.all_values:
                msg.layout.dim = [
                    MultiArrayDimension(
                        label="vectors",
                        size=self.sensor.num_mags,
                        stride=self.sensor.num_mags * 3,
                    ),
                    MultiArrayDimension(
                        label="components",
                        size=3,
                        stride=3,
                    ),
                ]
            else:
                msg.layout.dim = [
                    MultiArrayDimension(
                        label="rows",
                        size=self.sensor.num_mags,
                        stride=self.sensor.num_mags * self.num_dimensions,
                    ),
                    MultiArrayDimension(
                        label="columns",
                        size=self.num_dimensions,
                        stride=self.num_dimensions,
                    ),
                ]
            msg.layout.data_offset = 0
            self._data_publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Data publish failed: {exc}")

    def destroy_node(self):
        """Inform the user that the node is being destroyed."""
        self.get_logger().info("AnySkin image publisher stopped.")
        super().destroy_node()

    def _callback_reset_baseline(self, request, response):
        """Service callback to reset the baseline calibration.

        Args:
            request: Trigger request (empty)
            response: Trigger response with success status and message

        Returns:
            Trigger.Response: Response indicating success or failure
        """
        try:
            self.get_logger().info("Resetting baseline calibration...")
            self.calibrate_to_zero(num_samples=10, sample_rate=10.0)
            response.success = True
            response.message = "Baseline reset successfully"
            self.get_logger().info("Baseline reset complete")
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset baseline: {str(e)}"
            self.get_logger().error(f"Baseline reset failed: {e}")
        return response

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
            self.get_logger().info(f"Calibrating sample {sample_num + 1}/{num_samples}")
            self.get_logger().debug(f"Sample data: {data}")

            samples[sample_num] = data
            time.sleep(1.0 / sample_rate)

        self._baseline = np.mean(
            samples, axis=0
        )  # Shape: (num_magnets, num_dimensions)
        self.get_logger().info(f"Calibration complete. Baseline: {self._baseline}")


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 node that streams AnySkin tactile data with visualization."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Serial port for AnySkin sensor",
    )
    parser.add_argument(
        "--num_mags", type=int, default=5, help="Number of magnets in the sensor"
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--burst_mode", type=bool, default=True, help="Enable burst mode"
    )
    parser.add_argument(
        "--temp_filtered", type=bool, default=True, help="Enable temperature filtering"
    )
    parser.add_argument(
        "--namespace",
        type=str,
        default=None,
        help="ROS2 namespace for the node (e.g., 'sensor1', 'left_hand')",
    )
    parser.add_argument(
        "--data_publish_rate_hz",
        type=float,
        default=50.0,
        help="Data publish rate in Hz",
    )
    parser.add_argument(
        "--image_publish_rate_hz",
        type=float,
        default=30.0,
        help="Image publish rate in Hz",
    )
    parser.add_argument(
        "--image_width",
        type=int,
        default=400,
        help="Width of the output image",
    )
    parser.add_argument(
        "--image_height",
        type=int,
        default=400,
        help="Height of the output image",
    )
    parser.add_argument(
        "--viz_mode",
        type=str,
        default="3axis",
        choices=["magnitude", "3axis"],
        help="Visualization mode",
    )
    parser.add_argument(
        "--scaling",
        type=float,
        default=7.0,
        help="Scaling factor for visualization",
    )
    parser.add_argument(
        "--all_values",
        action="store_true",
        help="Publish raw x,y,z sensor values instead of rotated x,y vectors + magnitude",
    )

    args = parser.parse_args()

    rclpy.init()
    node = AnySkinImagePublisher(args)
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.001)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
