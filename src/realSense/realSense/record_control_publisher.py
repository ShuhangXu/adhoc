import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class RecordControlPublisher(Node):

    def __init__(self):
        super().__init__('record_control_publisher')
        self.publisher_ = self.create_publisher(String, 'record_control', 10)
        self.get_logger().info('Press "r" to start recording in 10 seconds, "s" to stop recording in 10 seconds, and "q" to quit.')

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{message}"')

    def run(self):
        while True:
            user_input = input('Enter command: ')
            if user_input == 'r':
                start_time = time.time() + 10  # 10 seconds from now
                self.publish_message(f'start,{start_time}')
            elif user_input == 's':
                stop_time = time.time() + 10  # 10 seconds from now
                self.publish_message(f'stop,{stop_time}')
            elif user_input == 'q':
                self.publish_message('quit')
                break
            else:
                self.get_logger().info('Invalid command. Use "r" to start, "s" to stop, and "q" to quit.')

def main(args=None):
    rclpy.init(args=args)
    record_control_publisher = RecordControlPublisher()

    # Run the publisher in a separate thread to allow for interactive input
    thread = threading.Thread(target=record_control_publisher.run)
    thread.start()

    rclpy.spin(record_control_publisher)

    record_control_publisher.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
