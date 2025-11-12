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
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def project_point(pt, cam_info:CameraInfo):
    """Project a 3D point (camera coords) into 2D pixel using intrinsics."""
    fx = cam_info.k[0]
    fy = cam_info.k[4]
    cx = cam_info.k[2]
    cy = cam_info.k[5]

    u = fx * pt[0] / pt[2] + cx
    v = fy * pt[1] / pt[2] + cy
    return int(round(u)), int(round(v))

def quat_to_rot(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix


def draw_tf_on_image(cv_image:np.ndarray, cam_T_target:TransformStamped, cam_info:CameraInfo):

    target_origin = np.array([cam_T_target.transform.translation.x, 
                              cam_T_target.transform.translation.y, 
                              cam_T_target.transform.translation.z])
    target_R = quat_to_rot(
        [cam_T_target.transform.rotation.x,
        cam_T_target.transform.rotation.y,
        cam_T_target.transform.rotation.z,
        cam_T_target.transform.rotation.w] 
    )

    x_axis_cam = target_R.dot(np.array([1, 0, 0]))
    y_axis_cam = target_R.dot(np.array([0, 1, 0]))
    z_axis_cam = target_R.dot(np.array([0, 0, 1]))

    ax_length = 0.1  # Length of the axes in meters
    x_axis_end = target_origin + x_axis_cam * ax_length
    y_axis_end = target_origin + y_axis_cam * ax_length
    z_axis_end = target_origin + z_axis_cam * ax_length

    p0 = project_point(target_origin, cam_info)
    px_end = project_point(x_axis_end, cam_info)
    py_end = project_point(y_axis_end, cam_info)
    pz_end = project_point(z_axis_end, cam_info)
    
    if (0 <= p0[0] < cv_image.shape[1]) and (0 <= p0[1] < cv_image.shape[0]):
        cv2.circle(cv_image, p0, 6, (0, 0, 0), -1)  # black dot
    else:
        rospy.logwarn("Origin out of bounds: %s, from 3D %s", p0, target_origin)
    
    cv2.line(cv_image, p0, px_end, (0, 0, 255), 4)  # Red X-axis, color in BGR
    cv2.line(cv_image, p0, py_end, (0, 255, 0), 4)  # Green Y-axis color in BGR
    cv2.line(cv_image, p0, pz_end, (255, 0, 0), 4)  # Blue Z-axis color in BGR

    caption = cam_T_target.child_frame_id
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    color = (0, 0, 255)  # Red
    thickness = 1
    text_size, _ = cv2.getTextSize(caption, font, font_scale, thickness)
    text_x = p0[0] - text_size[0] // 2
    text_y = p0[1] - text_size[1] +5  # Slightly above the circle

    cv2.putText(cv_image, "X", (px_end[0] + 5, px_end[1] - 5),
                font, font_scale, (0, 0, 255), thickness)
    cv2.putText(cv_image, "Y", (py_end[0] + 5, py_end[1] - 5),
                font, font_scale, (0, 255, 0), thickness)
    cv2.putText(cv_image, "Z", (pz_end[0] + 5, pz_end[1] - 5),
                font, font_scale, (255, 0, 0), thickness)

    cv2.putText(
        cv_image,
        caption,
        (text_x, text_y),
        font,
        0.6,
        (0, 0, 0),
        thickness,
    )


class TFOnImage(Node):
    def __init__(self, rate) -> None:

        super().__init__("tf_on_image")

        self.declare_parameter("cam_topic", "/nicla/camera/image_raw/compressed")
        self.declare_parameter("cam_info_topic", "/nicla/camera/camera_info")
        self.declare_parameter("cam_compressed", True)
        self.declare_parameter("img_out_topic", "/nicla/camera_with_tf/image_raw/compressed")
        self.declare_parameter("target_frames", ["target"])

        cam_topic = self.get_parameter("cam_topic").get_parameter_value().string_value
        cam_info_topic = self.get_parameter("cam_info_topic").get_parameter_value().string_value
        self.cam_compressed = self.get_parameter("cam_compressed").get_parameter_value().bool_value
        img_out_topic = self.get_parameter("img_out_topic").get_parameter_value().string_value
        self.target_frames = self.get_parameter("target_frames").get_parameter_value().string_array_value
        
        if self.cam_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, cam_topic, self.image_callback, 10

            )
            self.image = CompressedImage()
        else:
            self.image_sub = self.create_subscription(
                Image, cam_topic, self.image_callback, 10
            )
            self.image = Image()

        self.get_logger().info(f'Waiting for camera info on topic: {cam_info_topic}')
        ret_cam_info, self.cam_info = wait_for_message(
            CameraInfo, self, cam_info_topic)
        if ret_cam_info:
            self.get_logger().info("Camera info received.")
        else:
            self.get_logger().error("Camera info not received, shutdown.")
            exit()

        self.img_pub = self.create_publisher(
            CompressedImage, img_out_topic, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cv_image = np.array([])
        self.out_image = CompressedImage()

        self.cam_T_targets = []
        for target_frame in self.target_frames:
            self.cam_T_targets.append(TransformStamped())

        self.bridge = CvBridge()

        self.img_arrived = False

        self.get_logger().info("TFOnImage initialized")

        self.timer = self.create_timer(1 / rate, self.run)


    def image_callback(self, msg):
        self.img_arrived = True
        self.image = msg


    def get_image(self):
        try:
            if self.cam_compressed:
                self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
                    self.image, "passthrough"
                )
            else:
                self.cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return False   
        return True
    
    def get_transforms(self):
        try:
            for i, target_frame in enumerate(self.target_frames):
                self.cam_T_targets[i] = self.tf_buffer.lookup_transform(
                    self.cam_info.header.frame_id,
                    target_frame,
                    rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Error getting transforms: {e}')
            return False
        
        return True
                

    def run(self):

        if not self.img_arrived:
            return False

        if not self.get_image():
            return False
      
        if not self.get_transforms():
            return False
        
        for cam_T_target in self.cam_T_targets:
            draw_tf_on_image(self.cv_image, cam_T_target, self.cam_info)

        try:
            self.out_image.data = self.bridge.cv2_to_compressed_imgmsg(
                self.cv_image, dst_format="jpg"
            ).data
        except CvBridgeError as e:
            self.get_logger().error(e)
            return False

        self.img_pub.publish(self.out_image)

        return True

    def stop(self):
        pass