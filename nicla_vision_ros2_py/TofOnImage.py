#!/usr/bin/env python3

# Copyright 2024 HHCM, Istituto Italiano di Tecnologia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs.msg import Range
from cv_bridge import CvBridge, CvBridgeError
import cv2

def draw_dot_on_image(cv_image, range_value:float, color=(0, 0, 255)):

    # Draw a red dot at the center of the image
    height, width, _ = cv_image.shape
    center = (int((width / 2)), int((height / 2)))
    center_off = (center[0] + 6, center[1] - 19)

    cv2.circle(cv_image, center_off, 2, (0, 0, 255), -1)

    # Add a caption with the TOF range value near the center of the image
    caption = f"{range_value:.2f}m"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    color = (0, 0, 255)  # Red
    thickness = 1
    text_size, _ = cv2.getTextSize(caption, font, font_scale, thickness)
    text_x = center_off[0] - text_size[0] // 2
    text_y = center_off[1] - text_size[1] +5  # Slightly above the circle

    cv2.putText(
        cv_image,
        caption,
        (text_x, text_y),
        font,
        font_scale,
        color,
        thickness,
    )

    return True


class TofOnImage(Node):
    def __init__(self, rate) -> None:

        super().__init__("tof_on_image")

        self.declare_parameter("nicla_name", "nicla")
        nicla_name = self.get_parameter("nicla_name").get_parameter_value().string_value

        self.declare_parameter("nicla_cam_topic", "/" + nicla_name + "/camera/image_raw/compressed")
        self.declare_parameter("nicla_cam_compressed", True)
        self.declare_parameter("img_out_topic",  "/" + nicla_name + "/camera_with_tof/image_raw/compressed")
        self.declare_parameter("nicla_range_topic", "/" + nicla_name + "/tof")


        nicla_cam_topic = self.get_parameter("nicla_cam_topic").get_parameter_value().string_value
        self.nicla_cam_compressed = self.get_parameter("nicla_cam_compressed").get_parameter_value().bool_value
        img_out_topic = self.get_parameter("img_out_topic").get_parameter_value().string_value
        nicla_range_topic = self.get_parameter("nicla_range_topic").get_parameter_value().string_value

        if self.nicla_cam_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, nicla_cam_topic, self.image_callback, 10

            )
            self.image = CompressedImage()
        else:
            self.image_sub = self.create_subscription(
                Image, nicla_cam_topic, self.image_callback, 10
            )
            self.image = Image()

        self.range_sub = self.create_subscription(
            Range, nicla_range_topic, self.range_callback, 10
        )

        self.img_pub = self.create_publisher(
            CompressedImage, img_out_topic, 10
        )

        self.out_image = CompressedImage()
        self.range = Range()

        self.bridge = CvBridge()

        self.img_arrived = False

        self.get_logger().info("TofOnImage initialized")

        self.timer = self.create_timer(1 / rate, self.run)

    def image_callback(self, msg):
        self.img_arrived = True
        self.image = msg

    def range_callback(self, msg):
        self.range = msg

    def run(self):

        if not self.img_arrived:
            return

        try:
            if self.nicla_cam_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(
                    self.image, "passthrough"
                )
            else:
                cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        draw_dot_on_image(cv_image, self.range.range)

        try:
            self.out_image.data = self.bridge.cv2_to_compressed_imgmsg(
                cv_image, dst_format="jpg"
            ).data
        except CvBridgeError as e:
            rospy.logerr(e)

        self.img_pub.publish(self.out_image)

        return True

    def stop(self):
        pass
