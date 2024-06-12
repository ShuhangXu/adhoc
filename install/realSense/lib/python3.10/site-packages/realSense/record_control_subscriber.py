import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2

class RecordControlSubscriber(Node):

    def __init__(self):
        super().__init__('record_control_subscriber')
        self.subscription = self.create_subscription(
            String,
            'record_control',
            self.listener_callback,
            10)
        self.subscription
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline.start(self.config)
        self.recording = False
        self.rgb_writer = None
        self.depth_writer = None
        self.frame_width, self.frame_height = 640, 480

    def listener_callback(self, msg):
        if msg.data == 'start' and not self.recording:
            self.start_recording()
        elif msg.data == 'stop' and self.recording:
            self.stop_recording()

    def start_recording(self):
        self.recording = True
        self.rgb_writer = cv2.VideoWriter('rgb_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (self.frame_width, self.frame_height))
        self.depth_writer = cv2.VideoWriter('depth_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (self.frame_width, self.frame_height))
        self.get_logger().info('Recording started')

    def stop_recording(self):
        self.recording = False
        self.rgb_writer.release()
        self.depth_writer.release()
        self.get_logger().info('Recording stopped')

    def capture_frames(self):
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            if self.recording:
                self.rgb_writer.write(color_image)
                self.depth_writer.write(depth_colormap)

def main(args=None):
    rclpy.init(args=args)
    record_control_subscriber = RecordControlSubscriber()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(record_control_subscriber)

    try:
        executor.spin()
    finally:
        record_control_subscriber.stop_recording()
        record_control_subscriber.pipeline.stop()
        record_control_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
