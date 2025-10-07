
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
from rclpy.action.client import GoalStatus
class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.get_logger().info('NavGoalSender Node has been started.')
        # Create an action client for NavigateToPose
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose') # Default name of the Nav2 navigation action server
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('navigate_to_pose action server is available.')
    def send_goal(self, x, y, yaw=0.0):
        """
        Sends a navigation goal.
        :param x: The x-coordinate of the target point.
        :param y: The y-coordinate of the target point.
        :param yaw: The yaw angle (orientation) of the target point in radians.
        """
        goal_msg = NavigateToPose.Goal()
        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Target point is usually in the 'map' coordinate frame
        pose.header.stamp = self.get_clock().now().to_msg() # Use current timestamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0 # For 2D navigation, z is usually 0
        # Convert yaw (Euler angle) to a quaternion
        # Note: We use a simple helper function here, or directly use the tf_transformations library
        from tf_transformations import quaternion_from_euler

        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        goal_msg.pose = pose
        self.get_logger().info(
            f'Sending goal: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}, '
            f'yaw={yaw:.2f} (rad) in frame "{pose.header.frame_id}"'
        )
        # Send the goal and wait for the result
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # Register a callback function to be called when the goal is accepted by the server
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        # Register a callback function to be called when navigation is complete
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED: # GoalStatus.SUCCEEDED (rclpy.action.client.GoalStatus.SUCCEEDED)
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
        rclpy.shutdown() # Shut down the node after navigation is complete

def main(args=None):
    rclpy.init(args=args)
    nav_goal_sender = NavGoalSender()

    import sys
    # Define the target point
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_yaw = 0.0 # Orientation of the target point, e.g., 0.0 for facing positive x-axis, pi/2 for positive y-axis
    nav_goal_sender.send_goal(target_x, target_y, target_yaw)
    # Keep the node running until navigation is complete or interrupted
    rclpy.spin(nav_goal_sender)
    # If navigation completes before spin exits, this might not be executed
    nav_goal_sender.destroy_node()
    if rclpy.ok(): # Ensure rclpy is still running before attempting to shutdown again
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# python3 set_goal.py -4 2

