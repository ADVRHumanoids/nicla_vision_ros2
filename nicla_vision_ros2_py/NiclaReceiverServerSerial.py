#!/usr/bin/env python3

import queue
from threading import Thread
import time
import serial
import binascii


import numpy as np
import cv2

START_BYTE = 0xFF
END_BYTE = 0xFE

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11

START_BYTE = 0b11111111 # 0xFF
END_BYTE = 0b11111110 # 0xFE

JPEG_FIRST_BYTE = 0xFF
JPEG_SECOND_BYTE = 0xD8
JPEG_SECOND_TO_LAST_BYTE = 0xFF
JPEG_LAST_BYTE = 0xD9


class NiclaReceiverSerial(serial.Serial):

    def __init__(
        self, 
        port='/dev/ttyACM0', 
        baudrate=115200, 
        timeout=0.1,
        enable_range=False,
        enable_image=False,
        camera_receive_compressed=False,
        camera_pixel_format='rgb565',
        camera_height = 240,
        camera_width = 320,
        enable_audio=False,
        enable_imu=False
    ):
        print("Initializing NiclaReceiverSerial with port:", port,  "baudrate:", baudrate, "timeout:", timeout)
        
        super().__init__(port=port, baudrate=baudrate, timeout=timeout)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.camera_receive_compressed = camera_receive_compressed
        self.camera_pixel_format = camera_pixel_format
        self.camera_height = camera_height
        self.camera_width = camera_width
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
        self.server_thread_stop = False

        self.last_timestamp_img = None
        self.last_idx_img = None
        self.last_half_img = None

    def serve(self):
        self.server_thread_stop = False
        self.server_thread = Thread(target=self.__handle)
        self.server_thread.start()

    def stop_serve(self):
        print("stopping serve")
        self.server_thread_stop = True
        self.server_thread.join()

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
       

    def __handle(self):

        while not self.server_thread_stop:
            first_byte_int = int.from_bytes(self.read(1), "little")
            #print("first_byte_int:", first_byte_int)
            if first_byte_int == START_BYTE:
                size_packet = int.from_bytes(self.read(4), "little")
                #print("size_packet:", size_packet)

                packet = self.read(size_packet)
                #print("packet:", packet)
                last_byte_int = int.from_bytes(self.read(1), "little")

                #print ("lenght of packet:", len(packet), "size_packet:", size_packet)
                if last_byte_int != END_BYTE:
                    print("Received packet with wrong end byte, skipping...")
                    continue
            else:
                continue

            timestamp = time.time()  # int.from_bytes(packet[4:8], "big")

            data_type = packet[4]
            data = packet[5:]

            if len(data) == 0:
                print("Received packet with data of zero len, skipping...")
                continue

            if data_type == RANGE_TYPE:
                if self.enable_range:
                    try:
                        self.range_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.range_buffer.get()
                        self.range_buffer.put_nowait((timestamp, data))

                else:
                    pass

            elif data_type == IMAGE_TYPE:
                if self.enable_image:

                    #for images through serial, the timestamp field is crc-32 instead
                    crc_received = int.from_bytes(packet[0:4], "little")

                    #checksum
                    #checksum_calc = sum(data) & 0xFFFFFFFF #bitwise AND ensures the result fits into (4 bytes) 

                    #crc-32
                    crc_calc = binascii.crc32(data) & 0xFFFFFFFF  # bitwise AND ensures the result fits into (4 bytes)

                    if crc_calc != crc_received:
                        print(f"Checksum mismatch! Calculated {crc_calc}, received {crc_received}")
                        continue

                    if self.camera_receive_compressed:
                        #JPEG standard initial and end bytes check
                        if (data[0] != JPEG_FIRST_BYTE or
                            data[1] != JPEG_SECOND_BYTE or
                            data[-2] != JPEG_SECOND_TO_LAST_BYTE or
                            data[-1] != JPEG_LAST_BYTE):

                            print(f"Received image with wrong jpeg delimiters bytes, skipping it: {data[0]}, {data[1]}, {data[-2]}, {data[-1]}")
                            continue     

                        data_conv = np.asarray(
                            bytearray(data), dtype="uint8"
                        )

                        image = cv2.imdecode(
                            data_conv, cv2.IMREAD_UNCHANGED
                        )

                        #bgr to rgb conversion
                        image = np.dstack(
                            (
                                image[:, :, 2],
                                image[:, :, 1],
                                image[:, :, 0],
                            )
                        )


                    else: #not compressed image 
                        #rgb565_to_rgb888, with red and blue swap for the niclarospublisher using bgr
                        if self.camera_pixel_format == 'rgb565':
                            pixels = np.frombuffer(data, dtype=">u2").reshape((self.camera_height, self.camera_width)) #big-endian unsigned 16-bit integer
                            b = ((pixels >> 11) & 0x1F) << 3
                            g = ((pixels >> 5) & 0x3F) << 2
                            r = (pixels & 0x1F) << 3
                            image = np.stack((r, g, b), axis=-1).astype(np.uint8)

                        #From Bayer RGB
                        elif self.camera_pixel_format == 'bayer':
                            pixels = np.frombuffer(data, dtype=np.uint8).reshape((self.camera_height, self.camera_width))
                            #image = cv2.cvtColor(pixels, cv2.COLOR_BayerGBRG2BGR)  # Convert Bayer to BGR
                            #image = cv2.cvtColor(pixels, cv2.COLOR_BAYER_BG2BGR)  # Convert Bayer to BGR
                            image = cv2.cvtColor(pixels, cv2.COLOR_BAYER_GB2BGR)  # Convert Bayer to BGR

                        
                        else :
                            raise(f'Pixel format {self.camera_pixel_format} not available')
                        
                        #### DEBUGS
                        # Shift the image data by one pixel to the right
                        # data_shifted = data[1:] + data[:1]
                        # pixels_shifted = np.frombuffer(data_shifted, dtype=">u2").reshape((self.camera_height, self.camera_width)) #big-endian unsigned 16-bit integer

                        # def rgb565_to_rgb888(pixels):
                        #     r = ((pixels >> 11) & 0x1F) << 3
                        #     g = ((pixels >> 5) & 0x3F) << 2
                        #     b = (pixels & 0x1F) << 3
                        #     return np.stack((b, g, r), axis=-1).astype(np.uint8)  # BGR for OpenCV

                        # image = rgb565_to_rgb888(pixels)
                        # image_shifted = rgb565_to_rgb888(pixels_shifted)

                        # cv2.imshow("Normal", image)
                        # cv2.imshow("Shifted", image_shifted)
                        # cv2.waitKey(10000)

                    try:
                        self.image_buffer.put_nowait(
                            (timestamp, image)
                        )
                    except queue.Full:
                        print(f"Image queue full! overwriting oldest image")
                        self.image_buffer.get()
                        self.image_buffer.put_nowait(
                            (timestamp, image)
                        )

                else:
                    pass

            elif data_type == AUDIO_TYPE:
                if self.enable_audio:
                    try:
                        self.audio_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.audio_buffer.get_nowait()
                        self.audio_buffer.put_nowait((timestamp, data))

                else:
                    pass

            elif data_type == IMU_TYPE:
                if self.enable_imu:
                    try:
                        self.imu_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.imu_buffer.get_nowait()
                        self.imu_buffer.put_nowait((timestamp, data))
                else:
                    pass


