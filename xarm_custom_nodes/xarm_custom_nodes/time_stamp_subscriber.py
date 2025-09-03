#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from datetime import datetime
from xarm_msgs.msg import TimeStamp

class TimeStampSubscriber(Node):
    """
    ROS2 Node that subscribes to /time_stamp_topic and logs data to CSV
    """
    
    def __init__(self):
        super().__init__('timestamp_csv_logger')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            TimeStamp,
            '/time_stamp_topic',
            self.timestamp_callback,
            10
        )
        
        # Initialize CSV file
        self.data_dir = 'data'
        self.csv_filename = self.create_csv_file()
        self.csv_file = None
        self.csv_writer = None
        self.setup_csv_writer()
        
        self.get_logger().info(f'TimeStamp CSV Logger started. Logging to: {self.csv_filename}')
    
    def create_csv_file(self):
        """
        Create a new CSV file with timestamp in filename
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.data_dir}/timestamp_data_{timestamp}.csv"
        
        # Create the file and write headers
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['ros_time', 'unity_time', 'received_timestamp'])
        
        return filename
    
    def setup_csv_writer(self):
        """
        Setup CSV writer for continuous writing
        """
        self.csv_file = open(self.csv_filename, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
    
    def timestamp_callback(self, msg: TimeStamp):
        """
        Callback function for processing received TimeStamp messages
        """
        try:
            # Get current timestamp when data is received
            received_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            
            ros_time = msg.ros_time
            unity_time = msg.unity_time
            
            # Write data to CSV
            self.save_to_csv(ros_time, unity_time, received_time)
            
            # self.get_logger().info(f'Logged: ROS={ros_time}, Unity={unity_time}, Received={received_time}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')
    
    def save_to_csv(self, ros_time, unity_time, received_time):
        """
        Save the timestamp data to CSV file
        """
        try:
            # Write row to CSV
            self.csv_writer.writerow([ros_time, unity_time, received_time])
            
            # Flush to ensure data is written immediately
            self.csv_file.flush()
            
        except Exception as e:
            self.get_logger().error(f'Error writing to CSV: {str(e)}')
    
    def destroy_node(self):
        """
        Cleanup when node is destroyed
        """
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    """
    Main function to run the ROS2 node
    """
    rclpy.init(args=args)
    
    try:
        timestamp_logger = TimeStampSubscriber()
        rclpy.spin(timestamp_logger)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'timestamp_logger' in locals():
            timestamp_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()