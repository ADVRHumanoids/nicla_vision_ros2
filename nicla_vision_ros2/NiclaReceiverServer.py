#!/usr/bin/env python

import queue
import socketserver
from threading import Thread

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11


class UDPHandler(socketserver.BaseRequestHandler):
    def handle(self):

        #with udp, self.request is a pair (data, socket)
        packet = self.request[0]
        #socket = self.request[1]
        
        timestamp = int.from_bytes(packet[:4], "big")
        data_type = packet[4]
        data = packet[5:]
        if data_type == RANGE_TYPE:
            if self.server.enable_range:
                self.server.range_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMAGE_TYPE:
            if self.server.enable_image:
                self.server.image_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == AUDIO_TYPE:
            if self.server.enable_audio:
                self.server.audio_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMU_TYPE:
            if self.server.enable_imu:
                self.server.imu_buffer.put_nowait((timestamp, data))
            else:
                pass

class NiclaReceiverUDP(socketserver.UDPServer):

    def __init__(self, server_ip, server_port, enable_range=False, enable_image=False, enable_audio=False, enable_imu=False):

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

        while True: 
            packet = self.request.recv(65000)
            self.server.receiving_buffer.put_nowait(packet) 

class NiclaReceiverTCP(socketserver.TCPServer):

    def __init__(self, server_ip, server_port, enable_range=False, enable_image=False, enable_audio=False, enable_imu=False):

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
        
        if self.enable_range or self.enable_image or self.enable_audio or self.enable_imu:
            self.receiving_buffer = queue.Queue(maxsize=200)

        self.server_thread = None
 

    def serve(self):
        self.thread_regularizer = True
        self.sorting_thread = Thread(target=self.sorting)
        self.server_thread = Thread(target=self.serve_forever)
        self.sorting_thread.start()
        self.server_thread.start()

    def sorting(self):

        bkp_bytes_packets = bytes([])

        while self.thread_regularizer:

            try:  
                bytes_packets = self.receiving_buffer.get_nowait()
            except:
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
                    size_packet = int.from_bytes(bytes_packets[:4], "big")

                    if total_length - 4 >= size_packet:
                        packet = bytes_packets[4:size_packet+4]
                        bytes_packets = bytes_packets[size_packet+4:]

                        timestamp = int.from_bytes(packet[:4], "big")
                        
                        data_type = packet[4]
                        data = packet[5:]
 
                        if data_type == RANGE_TYPE:
                            if self.enable_range:
                                self.range_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == IMAGE_TYPE:
                            if self.enable_image:
                                self.image_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == AUDIO_TYPE:
                            if self.enable_audio:
                                self.audio_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == IMU_TYPE:
                            if self.enable_imu:
                                self.imu_buffer.put_nowait((timestamp, data))
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




        






