#!/usr/bin/env python3
from cognibot_interfaces import msg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cognibot_interfaces.msg import FeatureCodeArray

action_biases = {
        'ForwardClear': {'linear': 0.9, 'angular': 0.0},
        'LeftOpen': {'linear': 0.0, 'angular': 1.5},
        'RightOpen': {'linear': 0.0, 'angular': -1.5}
    }


class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        
        # Subscribe to feature codes
        self.feature_sub = self.create_subscription(
            FeatureCodeArray, '/feature_codes', self.feature_callback, 10)
        
        # Publish motor commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        


    def feature_callback(self, msg: FeatureCodeArray):
        """Receive feature codes and generate motor commands"""
 
        max_feature = max(msg.feature_codes, key=lambda feature: feature.activation_level)
        # print(f"Feature '{max_feature.feature_name}' has max activation: {max_feature.activation_level}")
        # TO DO: Apply competition-based action selection

        # Generate action from feature activations
        self.execute_action(msg.feature_codes)

    def execute_action(self, feature_codes):
        """Generate motor commands from ALL feature activations (weighted combination)"""
        
        cmd = Twist()

        # Instead of just the max feature, combine ALL features weighted by activation
        total_linear = 0.0
        total_angular = 0.0

        for feature in feature_codes:
            feature_name = feature.feature_name
            activation = feature.activation_level
            
          
            bias = action_biases[feature_name]
            total_linear += bias['linear'] * activation
            total_angular += bias['angular'] * activation
        
       
        
        cmd.linear.x = total_linear
        cmd.angular.z = total_angular

        self.cmd_vel_pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()
    node.get_logger().info("Action Executor node started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Action Executor node shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()