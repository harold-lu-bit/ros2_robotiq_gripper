#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class ParallelGripperCommandClient(Node):
    def __init__(self, action_name):
        super().__init__('parallel_gripper_command_client')
        self._action_client = ActionClient(
            self, 
            ParallelGripperCommand, 
            action_name
        )
        self.get_logger().info("ParallelGripperCommand client created")

    def send_goal(self, position, velocity=None, effort=None, joint_name='gripper_joint'):
        # Wait for action server
        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after waiting")
            return None

        # Create goal message
        goal_msg = ParallelGripperCommand.Goal()
        
        # Set up JointState command
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'gripper_frame'
        
        joint_state.name = [joint_name]
        joint_state.position = [float(position)]
        
        if velocity is not None:
            joint_state.velocity = [float(velocity)]
        if effort is not None:
            joint_state.effort = [float(effort)]
            
        goal_msg.command = joint_state
        
        self.get_logger().info(f"Sending goal: position={position}, velocity={velocity}, effort={effort}")
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: position={feedback.state.position}, '
                              f'reached_goal={feedback.reached_goal}, '
                              f'stalled={feedback.stalled}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create client node
    action_name = '/robotiq_gripper_controller/gripper_cmd'
    action_client = ParallelGripperCommandClient(action_name)
    
    # Send a goal - adjust these values as needed
    action_client.send_goal(
        position=0.0,   # 0~1, 1=closed
        velocity=255.0, # 0~255
        effort=128.0,   # 0~255
        joint_name='robotiq_85_left_knuckle_joint'
    )
    
    # Spin to process callbacks
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
