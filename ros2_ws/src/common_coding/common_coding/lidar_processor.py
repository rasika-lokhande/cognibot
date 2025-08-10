import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from cognibot_interfaces.msg import FeatureCodeArray, FeatureCode   
feature_codes_list = [
    "ForwardClear",
    "LeftOpen",
    "RightOpen"
]

action_biases = {
        'ForwardClear': {'linear': 1.2, 'angular': 0.0},
        'LeftOpen': {'linear': 0.0, 'angular': 0.8},
        'RightOpen': {'linear': 0.0, 'angular': 0.8}
    }


class LidarProcessor(Node):
    """Process LIDAR data to extract features """

    def __init__(self):
        super().__init__('lidar_processor')
       
        
        
        #get feature codes from configuration
        self.feature_codes = {code: FeatureCode() for code in feature_codes_list}
        # Initialize feature properties
        for name, code in self.feature_codes.items():
            code.feature_name = name
            code.activation_level = 0.0
            code.perceptual_evidence = 0.0
            code.action_bias = 0.0
            code.task_weight = 1.0

        # Action biases - how strongly each spatial feature would suggest specific actions
        self.action_biases = action_biases

        #subscribe to LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        #create publisher for updated feature codes
        self.feature_pub = self.create_publisher(
            FeatureCodeArray, '/feature_codes', 10)
        
        self.get_logger().info("LIDAR Processor initialized - Common coding spatial feature extraction ready!")
        


    def lidar_callback(self, msg: LaserScan):
        """Callback when LIDAR data is received"""
        self.process_lidar_data(msg)
        self.activate_features()
        self.publish_feature_states()


    def process_lidar_data(self, msg: LaserScan):
        """ Process LIDAR data to extract spatial features """    
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        
        # Calculate angles for proper sector division
        num_points = len(ranges)
        angles = np.arange(num_points) * msg.angle_increment + msg.angle_min
        angles_deg = np.degrees(angles)
        
        # Define spatial sectors (action-relevant spatial relationships)
        front_mask = (angles_deg >= -30) & (angles_deg <= 30)      # 60° front sector
        left_mask = (angles_deg > 30) & (angles_deg <= 90)         # Left side
        right_mask = (angles_deg >= -90) & (angles_deg < -30)      # Right side
        
        # Extract distance data for each spatial relationship
        front_ranges = ranges[front_mask]
        left_ranges = ranges[left_mask]
        right_ranges = ranges[right_mask]
        
        # Calculate perceptual evidence for each spatial feature
        # These represent how strongly the current spatial configuration supports each action-relevant feature
        
        # Simple and robust
        normalization_factor = msg.range_max * 0.5  # 50% of max range = full evidence

        self.feature_codes['ForwardClear'].perceptual_evidence = \
            min(1.0, np.mean(front_ranges) / normalization_factor)
        self.feature_codes['LeftOpen'].perceptual_evidence = \
            min(1.0, np.max(left_ranges) / normalization_factor)
        self.feature_codes['RightOpen'].perceptual_evidence = \
            min(1.0, np.max(right_ranges) / normalization_factor)


        self.get_logger().info(f"Perceptual evidence: ForwardClear={self.feature_codes['ForwardClear'].perceptual_evidence}, "
                                    f"LeftOpen={self.feature_codes['LeftOpen'].perceptual_evidence}, "
                                    f"RightOpen={self.feature_codes['RightOpen'].perceptual_evidence}")

        self.get_logger().info(f"LIDAR: {len(ranges)} points, max_range: {msg.range_max}")
        self.get_logger().info(f"Sectors: front={len(front_ranges)}, left={len(left_ranges)}, right={len(right_ranges)}")
      


    def calculate_action_strength(self, action_bias: dict) -> float:
        """ Calculate the strength of the action bias """
        return np.sqrt(action_bias['linear']**2 + action_bias['angular']**2)


    def activate_features(self):
        """Calculate activation levels by integrating perceptual evidence with action biases"""
        
        # Calculate activation levels based on perceptual evidence and other factos (TO DO)
        for name, code in self.feature_codes.items():
            code.activation_level = code.perceptual_evidence 


    def publish_feature_states(self):
        """Publish the current state of all features"""
        msg = FeatureCodeArray()
        msg.feature_codes = list(self.feature_codes.values())
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header

        msg.current_task = "spatial_perception"
        msg.confidence_level = np.mean([code.activation_level for code in self.feature_codes.values()])

        self.feature_pub.publish(msg)
            
    
    
        
       




    





def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LIDAR Processor shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()