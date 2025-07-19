#!/usr/bin/env python3
"""
Robot Controller Module for FernAssist

This module executes robot actions based on interpretations from the LLM.
It handles movement, manipulation, speech, and gesture commands.
"""

import rospy
import json
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from fernassist.msg import RobotAction, ExecuteAction, ExecuteGoal
from fernassist.action import ExecuteAction as ExecuteActionMsg

class RobotController:
    """Executes robot actions based on LLM interpretations."""
    
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.speech_pub = rospy.Publisher('/speech/output', String, queue_size=10)
        self.gesture_pub = rospy.Publisher('/gesture/command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/fernassist/robot_action', RobotAction, self.execute_action)
        
        # Action clients
        self.execute_action_client = actionlib.SimpleActionClient(
            '/fernassist/execute_action', ExecuteActionMsg
        )
        
        # Parameters
        self.max_velocity = rospy.get_param('~max_velocity', 0.5)
        self.safety_timeout = rospy.get_param('~safety_timeout', 30.0)
        
        # Wait for action server
        rospy.loginfo("Waiting for execute action server...")
        self.execute_action_client.wait_for_server()
        rospy.loginfo("Execute action server connected")
        
        rospy.loginfo("Robot Controller initialized")
    
    def execute_action(self, action_msg):
        """Execute a robot action based on the action message."""
        try:
            action_type = action_msg.action_type
            parameters = json.loads(action_msg.parameters)
            confidence = action_msg.confidence
            
            rospy.loginfo(f"Executing action: {action_type} with confidence: {confidence}")
            
            # Create action goal
            goal = ExecuteGoal()
            goal.action_type = action_type
            goal.parameters = action_msg.parameters
            
            # Send goal to action server
            self.execute_action_client.send_goal(
                goal,
                feedback_cb=self.action_feedback_cb,
                done_cb=self.action_done_cb
            )
            
            # Wait for result with timeout
            self.execute_action_client.wait_for_result(
                rospy.Duration(self.safety_timeout)
            )
            
            if self.execute_action_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                result = self.execute_action_client.get_result()
                rospy.loginfo(f"Action completed successfully: {result.status_message}")
            else:
                rospy.logwarn("Action execution failed or timed out")
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse action parameters: {e}")
        except Exception as e:
            rospy.logerr(f"Error executing action: {e}")
    
    def action_feedback_cb(self, feedback):
        """Callback for action feedback."""
        rospy.loginfo(f"Action progress: {feedback.progress:.1%} - {feedback.status_message}")
    
    def action_done_cb(self, state, result):
        """Callback for action completion."""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Action completed: {result.status_message}")
        else:
            rospy.logwarn(f"Action failed with state: {state}")
    
    def move_robot(self, parameters):
        """Execute robot movement."""
        try:
            # Extract movement parameters
            linear_x = parameters.get('linear_x', 0.0)
            linear_y = parameters.get('linear_y', 0.0)
            angular_z = parameters.get('angular_z', 0.0)
            
            # Create twist message
            twist = Twist()
            twist.linear.x = max(-self.max_velocity, min(self.max_velocity, linear_x))
            twist.linear.y = max(-self.max_velocity, min(self.max_velocity, linear_y))
            twist.angular.z = max(-self.max_velocity, min(self.max_velocity, angular_z))
            
            # Publish movement command
            self.cmd_vel_pub.publish(twist)
            
            rospy.loginfo(f"Moving robot: linear=({linear_x}, {linear_y}), angular={angular_z}")
            
        except Exception as e:
            rospy.logerr(f"Error in robot movement: {e}")
    
    def speak_text(self, parameters):
        """Execute speech output."""
        try:
            text = parameters.get('text', '')
            if text:
                self.speech_pub.publish(String(text))
                rospy.loginfo(f"Speaking: {text}")
            else:
                rospy.logwarn("No text provided for speech")
                
        except Exception as e:
            rospy.logerr(f"Error in speech output: {e}")
    
    def perform_gesture(self, parameters):
        """Execute gesture command."""
        try:
            gesture = parameters.get('gesture', '')
            if gesture:
                self.gesture_pub.publish(String(gesture))
                rospy.loginfo(f"Performing gesture: {gesture}")
            else:
                rospy.logwarn("No gesture specified")
                
        except Exception as e:
            rospy.logerr(f"Error in gesture execution: {e}")
    
    def run(self):
        """Main run loop."""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 