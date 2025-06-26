#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
import sys
import math
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.global_frame = ''
     
        cli = self.create_client(GetParameters, '/local_planner/dwa_planner/get_parameters')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = GetParameters.Request()
        req.names = ['GLOBAL_FRAME']
        future = cli.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    res = future.result()
                    self.global_frame = getattr(res.values[0], 'string_value')
                except Exception as e:
                    self.get_logger().warn('Service call failed %r' % (e,))
                break


        # Create publisher for goals
        self.goal_publisher = self.create_publisher(
            PoseStamped, 
            '/move_base_simple/goal', 
            10
        )
        
        self.get_logger().info(f'Goal Publisher initialized. Using global frame: {self.global_frame}')
        self.get_logger().info('Usage: python3 goal_publisher.py <x> <y> [theta]')
        self.get_logger().info('Example: python3 goal_publisher.py 2.0 1.5 0.0')

    def publish_goal(self, x, y, theta=0.0):
        """Publish a goal with the given coordinates."""
        goal_msg = PoseStamped()
        
        # Set header
        goal_msg.header.frame_id = self.global_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        
        # Set orientation from theta (yaw angle)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal_msg.pose.orientation.w = math.cos(float(theta) / 2.0)
        
        # Publish the goal
        self.goal_publisher.publish(goal_msg)
        
        self.get_logger().info(
            f'Goal published: x={x}, y={y}, theta={theta} rad in frame "{self.global_frame}"'
        )


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    if len(sys.argv) < 3:
        print("Error: Insufficient arguments")
        print("Usage: python3 goal_publisher.py <x> <y> [theta]")
        print("Example: python3 goal_publisher.py 2.0 1.5 0.0")
        return 1
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    except ValueError:
        print("Error: Invalid numeric arguments")
        print("Usage: python3 goal_publisher.py <x> <y> [theta]")
        return 1
    
    # Create node and publish goal
    goal_publisher = GoalPublisher()
    
    # Give some time for the publisher to be ready
    import time
    time.sleep(0.5)
    
    # Publish the goal
    goal_publisher.publish_goal(x, y, theta)
    
    # Keep the node alive briefly to ensure message is sent
    rclpy.spin_once(goal_publisher, timeout_sec=1.0)
    
    goal_publisher.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
