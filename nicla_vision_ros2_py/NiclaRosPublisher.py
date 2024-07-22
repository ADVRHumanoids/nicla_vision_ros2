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

import struct
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nicla_vision_ros2.msg import AudioData, AudioDataStamped, AudioInfo
from cv_bridge import CvBridge, CvBridgeError

from nicla_vision_ros2_py import NiclaReceiverUDP, NiclaReceiverTCP


class NiclaRosPublisher(Node):

    def __init__(self, rate):
        # instantiating the node
        super().__init__("nicla_receiver")

        # Declare parameters with default values

        # used for some topic message header, be sure to use the same
        # name here and in the urdf for proper rviz visualization
        self.declare_parameter("nicla_name", "nicla")

        self.declare_parameter("receiver_ip", rclpy.Parameter.Type.STRING)

        # server address and port (the address of the machine
        # running this code, any available port)
        self.declare_parameter("receiver_port", 8002)
        self.declare_parameter("connection_type", "udp")

        self.declare_parameter("enable_range", True)
        self.declare_parameter("enable_camera_raw", False)
        self.declare_parameter("enable_camera_compressed", True)
        self.declare_parameter("enable_audio", True)
        self.declare_parameter("enable_audio_stamped", False)
        self.declare_parameter("enable_audio_recognition_vosk", False)
        self.declare_parameter("audio_recognition_model_path", "")
        self.declare_parameter("audio_recognition_grammar", [""])
        self.declare_parameter("audio_recognition_listen_seconds", 2.0)
        self.declare_parameter("audio_recognition_wave_output_filename", "")
        self.declare_parameter("enable_imu", True)

        # Check if receiver_ip parameter is set and retrieve its value
        ip = None
        if self.has_parameter("receiver_ip"):
            ip = (
                self.get_parameter("receiver_ip")
                .get_parameter_value()
                .string_value
            )
        else:
            self.get_logger().error('Parameter "receiver_ip" not set!')
            raise Exception('Parameter "receiver_ip" not set!')

        nicla_name = (
            self.get_parameter("nicla_name").get_parameter_value().string_value
        )
        port = (
            self.get_parameter("receiver_port")
            .get_parameter_value()
            .integer_value
        )
        connection_type = (
            self.get_parameter("connection_type")
            .get_parameter_value()
            .string_value
        )

        self.enable_range = (
            self.get_parameter("enable_range").get_parameter_value().bool_value
        )
        self.enable_camera_raw = (
            self.get_parameter("enable_camera_raw")
            .get_parameter_value()
            .bool_value
        )
        self.enable_camera_compressed = (
            self.get_parameter("enable_camera_compressed")
            .get_parameter_value()
            .bool_value
        )
        self.enable_audio = (
            self.get_parameter("enable_audio").get_parameter_value().bool_value
        )
        self.enable_audio_stamped = (
            self.get_parameter("enable_audio_stamped")
            .get_parameter_value()
            .bool_value
        )
        self.enable_audio_recognition_vosk = (
            self.get_parameter("enable_audio_recognition_vosk")
            .get_parameter_value()
            .bool_value
        )
        self.audio_recognition_model_path = (
            self.get_parameter("audio_recognition_model_path")
            .get_parameter_value()
            .string_value
        )
        self.audio_recognition_grammar = (
            self.get_parameter("audio_recognition_grammar")
            .get_parameter_value()
            .string_array_value
        )
        if (
            len(self.audio_recognition_grammar) == 0
            or self.audio_recognition_grammar[0] == ""
        ):
            self.audio_recognition_grammar = "[]"
        self.audio_recognition_grammar = str(
            self.audio_recognition_grammar
        ).replace("'", '"')

        self.audio_recognition_listen_seconds = (
            self.get_parameter("audio_recognition_listen_seconds")
            .get_parameter_value()
            .double_value
        )
        self.audio_recognition_wave_output_filename = (
            self.get_parameter("audio_recognition_wave_output_filename")
            .get_parameter_value()
            .string_value
        )
        self.enable_imu = (
            self.get_parameter("enable_imu").get_parameter_value().bool_value
        )

        sensor_string = []
        if self.enable_range:
            sensor_string.append("range")
        if self.enable_camera_raw:
            sensor_string.append("camera_raw")
        if self.enable_camera_compressed:
            sensor_string.append("camera_compressed")
        if self.enable_audio:
            sensor_string.append("audio")
        if self.enable_audio_stamped:
            sensor_string.append("audio_stamped")
        if self.enable_imu:
            sensor_string.append("imu")

        str_msg = (
            f"Initializing at {ip}:{port} with {connection_type}"
            + f"connection with sensors: {sensor_string}"
        )
        self.get_logger().info(str_msg)

        if self.enable_range:
            range_topic = nicla_name + "/tof"
            self.range_pub = self.create_publisher(Range, range_topic, 5)
            self.range_msg = Range()
            self.range_msg.header.frame_id = nicla_name + "_tof"
            self.range_msg.radiation_type = Range.INFRARED
            self.range_msg.min_range = 0.0
            self.range_msg.max_range = 3.6
            self.range_msg.field_of_view = (
                0.471239  # 27degrees according to VL53L1X spec
            )

        if self.enable_camera_raw:
            # Default topic name of image transport
            # (which is not available in python so we do not use it)
            image_raw_topic = nicla_name + "/camera/image_raw"
            self.image_raw_msg = Image()
            self.image_raw_msg.header.frame_id = nicla_name + "_camera"
            self.image_raw_pub = self.create_publisher(
                Image, image_raw_topic, 5
            )

        if self.enable_camera_compressed:
            image_compressed_topic = (
                nicla_name + "/camera/image_raw/compressed"
            )
            self.image_compressed_msg = CompressedImage()
            self.image_compressed_msg.header.frame_id = nicla_name + "_camera"
            self.image_compressed_msg.format = "jpeg"
            self.image_compressed_pub = self.create_publisher(
                CompressedImage, image_compressed_topic, 5
            )

        if self.enable_camera_raw or self.enable_camera_compressed:

            self.cv_bridge = CvBridge()

            camera_info_topic = nicla_name + "/camera/camera_info"
            self.camera_info_msg = CameraInfo()
            self.camera_info_msg.header.frame_id = nicla_name + "_camera"
            self.camera_info_msg.height = 240
            self.camera_info_msg.width = 320
            self.camera_info_msg.distortion_model = "plumb_rob"
            self.camera_info_msg.k = [
                416.650528,
                0.000000,
                166.124514,
                0.000000,
                419.404643,
                104.410543,
                0.000000,
                0.000000,
                1.000000,
            ]
            self.camera_info_msg.d = [
                0.176808,
                -0.590488,
                -0.008412,
                0.015473,
                0.000000,
            ]
            self.camera_info_msg.p = [
                421.373566,
                0.000000,
                168.731782,
                0.000000,
                0.000000,
                426.438812,
                102.665989,
                0.000000,
                0.000000,
                0.000000,
                1.000000,
                0.000000,
            ]
            self.camera_info_pub = self.create_publisher(
                CameraInfo, camera_info_topic, 5
            )

        if self.enable_audio:
            audio_topic = nicla_name + "/audio"
            self.audio_msg = AudioData()
            self.audio_pub = self.create_publisher(AudioData, audio_topic, 10)

        if self.enable_audio_stamped:
            audio_stamped_topic = nicla_name + "/audio_stamped"
            self.audio_stamped_msg = AudioDataStamped()
            self.audio_stamped_pub = self.create_publisher(
                AudioDataStamped, audio_stamped_topic, 10
            )

        if self.enable_audio or self.enable_audio_stamped:
            audio_info_topic = nicla_name + "/audio_info"
            self.audio_info_msg = AudioInfo()
            self.audio_info_msg.channels = 1
            self.audio_info_msg.sample_rate = 16000
            self.audio_info_msg.sample_format = "S16LE"
            self.audio_info_msg.bitrate = 0
            self.audio_info_msg.coding_format = "raw"
            self.audio_info_pub = self.create_publisher(
                AudioInfo, audio_info_topic, 1
            )

        if self.enable_imu:
            imu_topic = nicla_name + "/imu"
            self.imu_msg = Imu()
            self.imu_msg.header.frame_id = nicla_name + "_imu"
            self.imu_msg.orientation.x = 0.0
            self.imu_msg.orientation.y = 0.0
            self.imu_msg.orientation.z = 0.0
            self.imu_msg.orientation.w = 1.0
            self.imu_pub = self.create_publisher(Imu, imu_topic, 5)

        if connection_type == "udp":
            self.nicla_receiver_server = NiclaReceiverUDP(
                ip,
                port,
                enable_range=self.enable_range,
                enable_image=self.enable_camera_raw
                or self.enable_camera_compressed,
                enable_audio=self.enable_audio or self.enable_audio_stamped,
                enable_imu=self.enable_imu,
            )
        elif connection_type == "tcp":
            self.nicla_receiver_server = NiclaReceiverTCP(
                ip,
                port,
                enable_range=self.enable_range,
                enable_image=self.enable_camera_raw
                or self.enable_camera_compressed,
                enable_audio=self.enable_audio or self.enable_audio_stamped,
                enable_imu=self.enable_imu,
            )
        else:
            self.get_logger().error(
                "Connection type ", connection_type, " not known"
            )
            raise Exception("Connection type not known")

        if self.enable_audio_recognition_vosk:
            if not self.audio_recognition_model_path:
                str_print = (
                    "Path for VOSK recognizer model is" +
                    "an empty string! Please provide " +
                    "'audio_recognition_model_path' arg"
                )
                self.get_logger().error(str_print)
                exit()

            from nicla_vision_ros2_py import SpeechRecognizer

            self.speech_recognizer = SpeechRecognizer.SpeechRecognizer(
                self.audio_recognition_model_path,
                self.audio_recognition_grammar,
                self.audio_recognition_listen_seconds,
                self.audio_recognition_wave_output_filename,
            )
            self.speech_recognizer_pub = self.create_publisher(
                String, nicla_name + "/audio_recognized", 10
            )

        self.nicla_receiver_server.serve()

        self.timer = self.create_timer(1 / rate, self.run)

    def run(self):

        if self.enable_range and (
            (range := self.nicla_receiver_server.get_range()) is not None
        ):
            self.range_msg.header.stamp = Time(seconds=range[0]).to_msg()

            self.range_msg.range = int.from_bytes(range[1], "little") / 1000
            self.range_pub.publish(self.range_msg)

        # PUBLISH IMAGE
        if self.enable_camera_raw or self.enable_camera_compressed:

            if (image := self.nicla_receiver_server.get_image()) is not None:

                # Publish info
                self.camera_info_msg.header.stamp = Time(
                    seconds=image[0]
                ).to_msg()
                self.camera_info_pub.publish(self.camera_info_msg)

                img_raw = image[1]

                # PUBLISH COMPRESSED
                if self.enable_camera_compressed:
                    self.image_compressed_msg.header.stamp = Time(
                        seconds=image[0]
                    ).to_msg()

                    try:
                        self.image_compressed_msg.data = (
                            self.cv_bridge.cv2_to_compressed_imgmsg(
                                img_raw, dst_format="jpg"
                            ).data
                        )
                    except CvBridgeError as e:
                        self.get_logger().error(e)

                    self.image_compressed_pub.publish(
                        self.image_compressed_msg
                    )

                # PUBLISH IMG RAW
                if self.enable_camera_raw:

                    self.image_raw_msg.header.stamp = Time(
                        seconds=image[0]
                    ).to_msg()
                    self.image_raw_msg.height = img_raw.shape[0]
                    self.image_raw_msg.width = img_raw.shape[1]
                    self.image_raw_msg.encoding = (
                        "bgr8"  # Assuming OpenCV returns BGR format
                    )
                    self.image_raw_msg.is_bigendian = 0  # Not big endian
                    self.image_raw_msg.step = (
                        img_raw.shape[1] * 3
                    )  # Width * number of channels

                    # Convert the OpenCV image to ROS Image format using
                    # cv_bridge
                    try:
                        self.image_raw_msg.data = self.cv_bridge.cv2_to_imgmsg(
                            img_raw, encoding="bgr8"
                        ).data
                    except CvBridgeError as e:
                        self.get_logger().error(e)

                    self.image_raw_pub.publish(self.image_raw_msg)

        # AUDIO DATA
        if self.enable_audio or self.enable_audio_stamped:

            self.audio_info_pub.publish(self.audio_info_msg)

            if (
                audio_data := self.nicla_receiver_server.get_audio()
            ) is not None:

                if self.enable_audio:
                    self.audio_msg.data = audio_data[1]
                    self.audio_pub.publish(self.audio_msg)

                if self.enable_audio_stamped:
                    self.audio_stamped_msg.header.stamp = Time(
                        seconds=audio_data[0]
                    ).to_msg()
                    self.audio_stamped_msg.audio.data = audio_data[1]
                    self.audio_stamped_pub.publish(self.audio_stamped_msg)

                if self.enable_audio_recognition_vosk:
                    audio_recognized = self.speech_recognizer.process_audio(
                        audio_data[1]
                    )

                    if audio_recognized:  # if not empty
                        msg = String()
                        msg.data = audio_recognized
                        self.speech_recognizer_pub.publish(msg)

        # IMU DATA
        if self.enable_imu and (
            (imu := self.nicla_receiver_server.get_imu()) is not None
        ):
            self.imu_msg.header.stamp = Time(seconds=imu[0]).to_msg()

            try:
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = struct.unpack(
                    "<ffffff", imu[1]
                )  # <: little endian
            except Exception as e:
                self.get_logger().error(f"imu pack has {len(imu[1])} bytes")
                raise e

            self.imu_msg.angular_velocity.x = 0.017453 * gyro_x
            self.imu_msg.angular_velocity.y = 0.017453 * gyro_y
            self.imu_msg.angular_velocity.z = 0.017453 * gyro_z
            self.imu_msg.linear_acceleration.x = 9.80665 * acc_x
            self.imu_msg.linear_acceleration.y = 9.80665 * acc_y
            self.imu_msg.linear_acceleration.z = 9.80665 * acc_z
            self.imu_pub.publish(self.imu_msg)

    def stop(self):
        self.nicla_receiver_server.stop_serve()
