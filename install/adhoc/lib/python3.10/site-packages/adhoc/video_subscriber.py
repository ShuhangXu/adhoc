# realSense/subscriber/subscriber_node.py
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import threading
import time
import zmq
import numpy as np
import wave
import csv

class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.bridge = CvBridge()
        self.frames = {}
        self.timestamps = {}
        self.audio_frames = []
        self.audio_timestamps = []
        self.gaze_frames = []
        self.gaze_timestamps = []
        self.lock = threading.Lock()
        self.recording = False
        self.frame_rate = 30  # Desired frame rate for the video
        self.interval = 1 / self.frame_rate
        self.last_save_time = None
        self.audio_sample_rate = 44100
        self.audio_channels = 2
        self.video_writers = {
            'publisher1_camera1': cv2.VideoWriter('data/output_publisher1_camera1.avi', cv2.VideoWriter_fourcc(*'XVID'), self.frame_rate, (1920, 1080)),
            'publisher1_camera2': cv2.VideoWriter('data/output_publisher1_camera2.avi', cv2.VideoWriter_fourcc(*'XVID'), self.frame_rate, (1920, 1080)),
            'gaze_video': cv2.VideoWriter('data/output_gaze.avi', cv2.VideoWriter_fourcc(*'XVID'), self.frame_rate, (1088, 1080)),
        }

        # Open a wave file for audio recording
        self.audio_file = wave.open('data/output_audio.wav', 'wb')
        self.audio_file.setnchannels(2)
        self.audio_file.setsampwidth(2)  # 2 bytes (16 bits) per sample
        self.audio_file.setframerate(44100)

        # Open a CSV file for gaze data
        self.gaze_data_file = open('data/gaze_data.csv', mode='w', newline='')
        self.gaze_data_writer = csv.writer(self.gaze_data_file)
        self.gaze_data_writer.writerow(["x", "y", "worn", "timestamp_unix_seconds"])

        # Initialize ZeroMQ context and sockets
        self.zmq_context = zmq.Context()
        self.video_sockets = {
            'publisher1_camera1': self.zmq_context.socket(zmq.SUB),
            'publisher1_camera2': self.zmq_context.socket(zmq.SUB),
        }
        self.audio_socket = self.zmq_context.socket(zmq.SUB)
        self.gaze_video_socket = self.zmq_context.socket(zmq.SUB)
        self.gaze_data_socket = self.zmq_context.socket(zmq.SUB)
        self.video_sockets['publisher1_camera1'].connect("tcp://169.254.253.76:5555")
        self.video_sockets['publisher1_camera2'].connect("tcp://169.254.253.76:5556")
        self.audio_socket.connect("tcp://169.254.253.76:5557")
        self.gaze_video_socket.connect("tcp://169.254.253.76:5558")
        self.gaze_data_socket.connect("tcp://169.254.253.76:5559")
        
        for socket in self.video_sockets.values():
            socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.audio_socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.gaze_video_socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.gaze_data_socket.setsockopt(zmq.SUBSCRIBE, b'')

        self.video_threads = {
            'publisher1_camera1': threading.Thread(target=self.receive_frames, args=('publisher1_camera1',)),
            'publisher1_camera2': threading.Thread(target=self.receive_frames, args=('publisher1_camera2',)),
        }
        self.audio_thread = threading.Thread(target=self.receive_audio)
        self.gaze_video_thread = threading.Thread(target=self.receive_gaze_video)
        self.gaze_data_thread = threading.Thread(target=self.receive_gaze_data)

        for thread in self.video_threads.values():
            thread.start()
        self.audio_thread.start()
        self.gaze_video_thread.start()
        self.gaze_data_thread.start()

        self.display_thread = threading.Thread(target=self.display_and_save)
        self.display_thread.start()

    def receive_frames(self, camera):
        while True:
            buffer, timestamp = self.video_sockets[camera].recv_pyobj()
            frame = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            with self.lock:
                self.frames[camera] = frame
                self.timestamps[camera] = timestamp

    def receive_audio(self):
        while True:
            audio_data, timestamp = self.audio_socket.recv_pyobj()
            with self.lock:
                self.audio_frames.append((audio_data, timestamp))
                self.audio_timestamps.append(timestamp)

    def receive_gaze_video(self):
        while True:
            buffer, timestamp = self.gaze_video_socket.recv_pyobj()
            frame = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            if isinstance(frame, np.ndarray):
                with self.lock:
                    self.gaze_frames.append((frame, timestamp))
                    self.gaze_timestamps.append(timestamp)
                    print(f"Received Gaze Frame - Timestamp: {timestamp}, Shape: {frame.shape}")  # Debugging line



    def receive_gaze_data(self):
        while True:
            gaze_data, timestamp = self.gaze_data_socket.recv_pyobj()
            with self.lock:
                self.gaze_data_writer.writerow([gaze_data.x, gaze_data.y, gaze_data.worn, gaze_data.timestamp_unix_seconds])
                self.gaze_timestamps.append(timestamp)

    def display_and_save(self):
        last_gaze_timestamp = 0
        last_timestamps = {'publisher1_camera1': 0, 'publisher1_camera2': 0}
        synced_gaze_frame = None

        while True:
            with self.lock:
                if all(camera in self.frames for camera in ['publisher1_camera1', 'publisher1_camera2']):
                    current_time = time.time()
                    if self.last_save_time is None or current_time - self.last_save_time >= self.interval:
                        self.last_save_time = current_time

                        # Ensure frame timestamps are consistent, fill gaps with the previous frame if needed
                        for camera in ['publisher1_camera1', 'publisher1_camera2']:
                            # print("--------------------------------------------------------")
                            # print(self.timestamps[camera])
                            # print(last_timestamps[camera])
                            # print()
                            if self.timestamps[camera] - last_timestamps[camera] > self.interval:
                                self.frames[camera] = self.frames[camera]
                                print("Package lost")
                            last_timestamps[camera] = self.timestamps[camera]

                        # Get the latest gaze frame that matches the current timestamp range
                        if self.gaze_timestamps:
                            # print(f"Gaze timestamps before update: {self.gaze_timestamps}")
                            while self.gaze_timestamps and self.gaze_timestamps[0] <= self.last_save_time + self.interval:
                                if self.gaze_frames:
                                    frame_tuple = self.gaze_frames.pop(0)
                                    synced_gaze_frame, last_gaze_timestamp = frame_tuple
                                    # print(f"Updating Gaze Frame - Type: {type(synced_gaze_frame)}, Shape: {synced_gaze_frame.shape}, Timestamp: {last_gaze_timestamp}")
                                    # print(f"Gaze timestamps after update: {self.gaze_timestamps}")
                                else:
                                    print("No gaze frames available for synchronization")
                                    break

                        # Resize frames for display
                        small_frame1 = cv2.resize(self.frames['publisher1_camera1'], (480, 270))
                        print("Resized small_frame1")
                        small_frame2 = cv2.resize(self.frames['publisher1_camera2'], (480, 270))
                        print("Resized small_frame2")
                        combined_image = cv2.vconcat([small_frame1, small_frame2])

                        # Display resized frames
                        cv2.imshow('RealSense', combined_image)
                        cv2.waitKey(1)
                        print("Displayed combined image")

                        if synced_gaze_frame is not None and isinstance(synced_gaze_frame, np.ndarray):
                            try:
                                small_gaze_frame = cv2.resize(synced_gaze_frame, (272, 270))
                                cv2.imshow('Gaze Tracker', small_gaze_frame)
                                cv2.waitKey(1)
                                print("Displayed small gaze frame")
                            except Exception as e:
                                print(f"Error resizing gaze frame: {e}")

                        # Save original frames
                        for camera in self.frames.keys():
                            self.video_writers[camera].write(self.frames[camera])
                        print("Saved camera frames")
                        if synced_gaze_frame is not None and isinstance(synced_gaze_frame, np.ndarray):
                            self.video_writers['gaze_video'].write(synced_gaze_frame)
                        print("Saved gaze frame")

                        # Synchronize and save audio
                        if self.audio_frames:
                            synced_audio = []
                            for audio_data, timestamp in self.audio_frames:
                                if timestamp <= self.last_save_time + self.interval:
                                    synced_audio.append(audio_data)
                            if synced_audio:
                                for audio_data in synced_audio:
                                    self.audio_file.writeframes(audio_data)
                                self.audio_frames = [(ad, ts) for ad, ts in self.audio_frames if ts > self.last_save_time + self.interval]
                            print("Synchronized and saved audio")
                #     else:
                #         print(f"Waiting for next interval: {current_time - self.last_save_time} seconds since last save")
                # else:
                #     print("Waiting for frames from both cameras")

            # time.sleep(self.interval / 10)  # Add a small sleep to avoid busy waiting


            
    def destroy_node(self):
        for writer in self.video_writers.values():
            writer.release()
        self.audio_file.close()
        self.gaze_data_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
