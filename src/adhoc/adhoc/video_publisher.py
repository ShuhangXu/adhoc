# realSense/publisher/publisher_node.py
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
import zmq
import time
import os
import sounddevice as sd
import threading
import wave
import csv
from pupil_labs.realtime_api.simple import discover_one_device

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.bridge = CvBridge()
        
        # Initialize ZeroMQ context and sockets
        self.zmq_context = zmq.Context()
        self.video_socket1 = self.zmq_context.socket(zmq.PUB)
        self.video_socket2 = self.zmq_context.socket(zmq.PUB)
        self.audio_socket = self.zmq_context.socket(zmq.PUB)
        self.gaze_video_socket = self.zmq_context.socket(zmq.PUB)
        self.gaze_data_socket = self.zmq_context.socket(zmq.PUB)
        self.video_socket1.bind("tcp://*:5555")
        self.video_socket2.bind("tcp://*:5556")
        self.audio_socket.bind("tcp://*:5557")
        self.gaze_video_socket.bind("tcp://*:5558")
        self.gaze_data_socket.bind("tcp://*:5559")
        
        # Initialize RealSense context
        self.rs_context = rs.context()
        self.devices = self.rs_context.query_devices()
        
        # Ensure there are at least two devices connected
        if len(self.devices) < 2:
            self.get_logger().error('Less than 2 RealSense devices found')
            return
        
        # Get the serial numbers of the devices
        self.serial_numbers = [dev.get_info(rs.camera_info.serial_number) for dev in self.devices]
        
        # Configure and start the pipelines
        self.pipeline1 = rs.pipeline()
        self.pipeline2 = rs.pipeline()
        config1 = rs.config()
        config2 = rs.config()
        
        config1.enable_device(self.serial_numbers[0])
        config1.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        
        config2.enable_device(self.serial_numbers[1])
        config2.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        
        self.pipeline1.start(config1)
        self.pipeline2.start(config2)

        # Create data folder if it doesn't exist
        os.makedirs('data', exist_ok=True)

        # Initialize video writers for local storage
        self.video_writer1 = cv2.VideoWriter('data/output_camera1.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1920, 1080))
        self.video_writer2 = cv2.VideoWriter('data/output_camera2.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1920, 1080))
        self.gaze_video_writer = cv2.VideoWriter('data/output_gaze.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1088, 1080))

        # Open a wave file for audio recording
        self.audio_file = wave.open('data/output_audio.wav', 'wb')
        self.audio_file.setnchannels(2)
        self.audio_file.setsampwidth(2)  # 2 bytes (16 bits) per sample
        self.audio_file.setframerate(44100)

        self.timer = self.create_timer(0.033, self.timer_callback)  # approximately 30 FPS

        # Initialize audio recording
        self.audio_sample_rate = 44100
        self.audio_channels = 2
        self.audio_thread = threading.Thread(target=self.record_audio)
        self.audio_thread.start()

        # Initialize gaze tracker recording
        self.gaze_thread = threading.Thread(target=self.record_gaze)
        self.gaze_thread.start()

    def timer_callback(self):
        frames1 = self.pipeline1.wait_for_frames()
        frames2 = self.pipeline2.wait_for_frames()
        color_frame1 = frames1.get_color_frame()
        color_frame2 = frames2.get_color_frame()
        if not color_frame1 or not color_frame2:
            return

        color_image1 = np.asanyarray(color_frame1.get_data())
        color_image2 = np.asanyarray(color_frame2.get_data())
        timestamp = time.time()
        
        # Save frames locally
        self.video_writer1.write(color_image1)
        self.video_writer2.write(color_image2)
        
        _, buffer1 = cv2.imencode('.jpg', color_image1)
        _, buffer2 = cv2.imencode('.jpg', color_image2)
        
        self.video_socket1.send_pyobj((buffer1.tobytes(), timestamp))
        self.video_socket2.send_pyobj((buffer2.tobytes(), timestamp))

    def record_audio(self):
        with sd.InputStream(samplerate=self.audio_sample_rate, channels=self.audio_channels, dtype='int16') as stream:
            while True:
                audio_data, overflowed = stream.read(self.audio_sample_rate)  # Read 1 second of audio
                timestamp = time.time()
                self.audio_socket.send_pyobj((audio_data, timestamp))
                self.audio_file.writeframes(audio_data)

    def record_gaze(self):
        device = discover_one_device(max_search_duration_seconds=10)
        if device is None:
            print("No device found.")
            return

        print(f"Connecting to {device}...")

        with open('data/gaze_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "worn", "timestamp_unix_seconds"])
            
            try:
                while True:
                    bgr_pixels, frame_datetime = device.receive_scene_video_frame()
                    bgr_pixels = cv2.resize(bgr_pixels, (1088, 1080))
                    self.gaze_video_writer.write(bgr_pixels)
                    timestamp = time.time()
                    
                    _, buffer = cv2.imencode('.jpg', bgr_pixels)
                    self.gaze_video_socket.send_pyobj((buffer.tobytes(), timestamp))
                    
                    gaze_data = device.receive_gaze_datum()
                    writer.writerow([gaze_data.x, gaze_data.y, gaze_data.worn, gaze_data.timestamp_unix_seconds])
                    self.gaze_data_socket.send_pyobj((gaze_data, timestamp))
            except KeyboardInterrupt:
                pass
            finally:
                print("Stopping...")
                device.close()
                self.gaze_video_writer.release()
                cv2.destroyAllWindows()

    def destroy_node(self):
        self.video_writer1.release()
        self.video_writer2.release()
        self.audio_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
