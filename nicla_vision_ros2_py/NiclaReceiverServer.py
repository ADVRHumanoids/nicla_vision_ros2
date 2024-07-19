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

import queue
import socket
import socketserver
from threading import Thread
import time
import struct
import numpy as np
import cv2

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11


class UDPHandler(socketserver.BaseRequestHandler):
    def handle(self):

        # with udp, self.request is a pair (data, socket)
        packet = self.request[0]
        # socket = self.request[1]

        size_packet = int.from_bytes(packet[:4], "little")

        if size_packet == len(packet[4:]):

            timestamp = time.time()
            data_type = packet[8]
            data = packet[9:]

            if data_type == RANGE_TYPE:
                if self.server.enable_range:
                    try:
                        self.server.range_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.server.range_buffer.get()
                        self.server.range_buffer.put_nowait((timestamp, data))
                else:
                    pass

            elif data_type == IMAGE_TYPE:
                if self.server.enable_image:

                    timestamp_img = int.from_bytes(packet[4:8], "little")
                    idx_img = packet[9]

                    # print("TIMESTAMP: ", timestamp_img)
                    # print("SIZE: ", size_packet)
                    # print("IDX IMG: ", idx_img)

                    if not idx_img:  # first half
                        self.server.last_timestamp_img = timestamp_img
                        self.server.last_idx_img = idx_img
                        self.server.last_half_img = packet[10:]

                    else:  # second half
                        if timestamp_img == self.server.last_timestamp_img:

                            half_img_bin_0 = self.server.last_half_img

                            half_img_bin_1 = packet[10:]

                            half_img_bin_0 = np.asarray(
                                bytearray(half_img_bin_0), dtype="uint8"
                            )
                            half_img_bin_1 = np.asarray(
                                bytearray(half_img_bin_1), dtype="uint8"
                            )

                            half_img_dec_0 = cv2.imdecode(
                                half_img_bin_0, cv2.IMREAD_UNCHANGED
                            )
                            half_img_dec_0 = np.dstack(
                                (
                                    half_img_dec_0[:, :, 2],
                                    half_img_dec_0[:, :, 1],
                                    half_img_dec_0[:, :, 0],
                                )
                            )

                            half_img_dec_1 = cv2.imdecode(
                                half_img_bin_1, cv2.IMREAD_UNCHANGED
                            )
                            half_img_dec_1 = np.dstack(
                                (
                                    half_img_dec_1[:, :, 2],
                                    half_img_dec_1[:, :, 1],
                                    half_img_dec_1[:, :, 0],
                                )
                            )

                            # Stack the images vertically
                            combined_image = np.vstack(
                                (half_img_dec_1, half_img_dec_0)
                            )

                            try:
                                self.server.image_buffer.put_nowait(
                                    (timestamp, combined_image)
                                )
                            except queue.Full:
                                self.server.image_buffer.get()
                                self.server.image_buffer.put_nowait(
                                    (timestamp, combined_image)
                                )
                else:
                    pass

            elif data_type == AUDIO_TYPE:
                if self.server.enable_audio:
                    try:
                        self.server.audio_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.server.audio_buffer.get_nowait()
                        self.server.audio_buffer.put_nowait((timestamp, data))
                else:
                    pass

            elif data_type == IMU_TYPE:
                if self.server.enable_imu:
                    try:
                        self.server.imu_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.server.imu_buffer.get_nowait()
                        self.server.imu_buffer.put_nowait((timestamp, data))
                else:
                    pass

        else:
            print(
                "Warning: received packet of length {}, but expected length was {}!".format(
                    len(packet[4:]), size_packet
                )
            )


class NiclaReceiverUDP(socketserver.UDPServer):

    def __init__(
        self,
        server_ip,
        server_port,
        enable_range=False,
        enable_image=False,
        enable_audio=False,
        enable_imu=False,
    ):

        super().__init__((server_ip, server_port), UDPHandler)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.enable_audio = enable_audio
        self.enable_imu = enable_imu

        if self.enable_range:
            self.range_buffer = queue.Queue(maxsize=10)
        if self.enable_image:
            self.image_buffer = queue.Queue(maxsize=10)
        if self.enable_audio:
            self.audio_buffer = queue.Queue(maxsize=10)
        if self.enable_imu:
            self.imu_buffer = queue.Queue(maxsize=10)

        self.server_thread = None

        self.last_timestamp_img = None
        self.last_idx_img = None
        self.last_half_img = None

    def serve(self):
        self.server_thread = Thread(target=self.serve_forever)
        self.server_thread.start()

    def stop_serve(self):
        print("stopping")
        self.shutdown()
        self.server_thread.join()
        self.server_close()

    def get_range(self):

        if not self.range_buffer.empty():
            return self.range_buffer.get_nowait()
        else:
            return None

    def get_image(self):
        if not self.image_buffer.empty():
            return self.image_buffer.get_nowait()
        else:
            return None

    def get_audio(self):
        if not self.audio_buffer.empty():
            return self.audio_buffer.get_nowait()
        else:
            return None

    def get_imu(self):
        if not self.imu_buffer.empty():
            return self.imu_buffer.get_nowait()
        else:
            return None


class TCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5.0)  # Set a timeout
        self.server.nicla_disconnect = False

        while True:
            try:
                packet = self.request.recv(65000)
                if not packet:
                    break
                timestamp = time.time()
                timestamp = struct.pack(">d", timestamp)
                packet = timestamp + packet
                self.server.receiving_buffer.put_nowait(packet)
            except socket.timeout:
                print("Warning: Nicla disconnected! Resetting server... ")
                self.server.receiving_buffer.queue.clear()
                self.server.nicla_disconnect = True
                break
            except Exception as e:
                print(f"Exception: {e}")
                self.server.receiving_buffer.queue.clear()
                self.server.nicla_disconnect = True
                break  # Break the loop on any other exception


class NiclaReceiverTCP(socketserver.TCPServer):

    def __init__(
        self,
        server_ip,
        server_port,
        enable_range=False,
        enable_image=False,
        enable_audio=False,
        enable_imu=False,
    ):

        super().__init__((server_ip, server_port), TCPHandler)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.enable_audio = enable_audio
        self.enable_imu = enable_imu

        if self.enable_range:
            self.range_buffer = queue.Queue(maxsize=100)
        if self.enable_image:
            self.image_buffer = queue.Queue(maxsize=100)
        if self.enable_audio:
            self.audio_buffer = queue.Queue(maxsize=100)
        if self.enable_imu:
            self.imu_buffer = queue.Queue(maxsize=100)

        if (
            self.enable_range
            or self.enable_image
            or self.enable_audio
            or self.enable_imu
        ):
            self.receiving_buffer = queue.Queue(maxsize=200)

        self.server_thread = None
        self.nicla_disconnect = False

    def serve(self):
        self.thread_regularizer = True
        self.sorting_thread = Thread(target=self.sorting)
        self.server_thread = Thread(target=self.serve_forever)
        self.sorting_thread.start()
        self.server_thread.start()

    def sorting(self):

        bkp_bytes_packets = bytes([])
        timestamp = None
        half_img = None

        while self.thread_regularizer:

            try:
                bytes_packets = self.receiving_buffer.get_nowait()
                timestamp = struct.unpack(">d", bytes_packets[:8])[0]
                bytes_packets = bytes_packets[8:]

                if self.nicla_disconnect:
                    bytes_packets = bytes([])
                    bkp_bytes_packets = bytes([])
            except Exception:
                continue

            bytes_packets = bkp_bytes_packets + bytes_packets
            bkp_bytes_packets = bytes([])

            if len(bytes_packets) < 9:
                print("Got a packet from receiver less than header size!")
                continue
            else:
                total_length = len(bytes_packets)
                loop_termination_flag = True

                while loop_termination_flag:
                    size_packet = int.from_bytes(bytes_packets[:4], "little")

                    if total_length - 4 >= size_packet:
                        packet = bytes_packets[4:size_packet + 4]
                        bytes_packets = bytes_packets[size_packet + 4:]

                        # timestamp = int.from_bytes(packet[:4], "big")

                        data_type = int.from_bytes(packet[4:5], "little")
                        data = packet[5:]

                        if data_type == RANGE_TYPE:
                            if self.enable_range:
                                try:
                                    self.range_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                                except queue.Full:
                                    self.range_buffer.get_nowait()
                                    self.range_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                            else:
                                pass

                        elif data_type == IMAGE_TYPE:
                            if self.enable_image:
                                idx_img = data[0]
                                half_img_bin = np.asarray(
                                    bytearray(data[1:]), dtype="uint8"
                                )

                                half_img_dec = cv2.imdecode(
                                    half_img_bin, cv2.IMREAD_UNCHANGED
                                )

                                half_img_dec = np.dstack(
                                    (
                                        half_img_dec[:, :, 2],
                                        half_img_dec[:, :, 1],
                                        half_img_dec[:, :, 0],
                                    )
                                )

                                if not idx_img:  # first half
                                    half_img = half_img_dec

                                else:  # second half
                                    # Stack the images vertically
                                    combined_image = np.vstack(
                                        (half_img_dec, half_img)
                                    )
                                    try:
                                        self.image_buffer.put_nowait(
                                            (timestamp, combined_image)
                                        )
                                    except queue.Full:
                                        self.image_buffer.get_nowait()
                                        self.image_buffer.put_nowait(
                                            (timestamp, combined_image)
                                        )

                                    half_img = None
                            else:
                                pass

                        elif data_type == AUDIO_TYPE:
                            if self.enable_audio:
                                try:
                                    self.audio_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                                except queue.Full:
                                    self.audio_buffer.get_nowait()
                                    self.audio_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                            else:
                                pass

                        elif data_type == IMU_TYPE:
                            if self.enable_imu:
                                try:
                                    self.imu_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                                except queue.Full:
                                    self.imu_buffer.get_nowait()
                                    self.imu_buffer.put_nowait(
                                        (timestamp, data)
                                    )
                            else:
                                pass

                    else:
                        bkp_bytes_packets = bytes_packets
                        loop_termination_flag = False

                    total_length = len(bytes_packets)
                    if total_length == 0:
                        loop_termination_flag = False

    def stop_serve(self):
        print("stopping")
        self.thread_regularizer = False
        self.sorting_thread.join()
        self.shutdown()
        self.server_thread.join()
        self.server_close()

    def get_range(self):

        if not self.range_buffer.empty():
            return self.range_buffer.get_nowait()
        else:
            return None

    def get_image(self):
        if not self.image_buffer.empty():
            return self.image_buffer.get_nowait()
        else:
            return None

    def get_audio(self):
        if not self.audio_buffer.empty():
            return self.audio_buffer.get_nowait()
        else:
            return None

    def get_imu(self):
        if not self.imu_buffer.empty():
            return self.imu_buffer.get_nowait()
        else:
            return None
