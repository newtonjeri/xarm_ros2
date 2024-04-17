import rclpy
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles

import os
import csv
from datetime import datetime

class DataFromUnityAnalysisNode(Node):
    def __init__(self):
        super().__init__("analysis_node")

        # Create the execution directory if it doesn't exist
        self.data_dir = 'data'
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        self.data_from_unity_file = f"{self.data_dir}/unity_to_ros2__data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        self.positions = [0]
        self.unity_time = "----"
        # Subscription topic
        self.topic_name = "/joint_info_fromUnity"

        # Subscription
        self.unity_subscription = self.create_subscription(JointNamesAndAngles, self.topic_name, self.subscription_callback, 10)


    def subscription_callback(self, msg):
        self.positions = msg.positions
        self.unity_time = msg.timestamp

        time_data_received = datetime.now().strftime("%m/%d/%Y %H:%M:%S.%f ")
        self.save_to_csv(self.positions[0], self.unity_time, time_data_received )

    def save_to_csv(self, joint1_position, unity_time, time_data_received):
        with open(self.data_from_unity_file, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            if csvfile.tell() == 0:  # Check if the file is empty
                csvwriter.writerow(['Joint1 Position','Time Data is Sent from Unity', 'Time Data Received'])
            
            csvwriter.writerow([joint1_position, unity_time, time_data_received])

def main(args = None):
    rclpy.init(args=args)
    node = DataFromUnityAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
