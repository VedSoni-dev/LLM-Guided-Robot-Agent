#!/usr/bin/env python3
"""
System Monitor Module for FernAssist

This module monitors the health and status of all FernAssist components,
providing real-time feedback and error handling.
"""

import rospy
import json
from datetime import datetime
from std_msgs.msg import String
from fernassist.msg import UserIntent, RobotAction, LLMResponse

class SystemMonitor:
    """Monitors system health and provides status feedback."""
    
    def __init__(self):
        rospy.init_node('system_monitor', anonymous=True)
        
        # Publishers
        self.status_pub = rospy.Publisher('/fernassist/system_status', String, queue_size=10)
        self.health_pub = rospy.Publisher('/fernassist/health_status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/fernassist/user_intent', UserIntent, self.monitor_user_intent)
        rospy.Subscriber('/fernassist/robot_action', RobotAction, self.monitor_robot_action)
        rospy.Subscriber('/fernassist/llm_response', LLMResponse, self.monitor_llm_response)
        
        # System state
        self.system_status = {
            'aac_listener': {'status': 'unknown', 'last_seen': None},
            'llm_interpreter': {'status': 'unknown', 'last_seen': None},
            'robot_controller': {'status': 'unknown', 'last_seen': None},
            'overall_health': 'unknown'
        }
        
        # Statistics
        self.stats = {
            'total_inputs': 0,
            'successful_actions': 0,
            'failed_actions': 0,
            'avg_processing_time': 0.0
        }
        
        # Timer for periodic health checks
        self.health_timer = rospy.Timer(rospy.Duration(5.0), self.periodic_health_check)
        
        rospy.loginfo("System Monitor initialized")
    
    def monitor_user_intent(self, intent_msg):
        """Monitor user intent processing."""
        try:
            self.system_status['aac_listener']['status'] = 'active'
            self.system_status['aac_listener']['last_seen'] = rospy.Time.now()
            
            self.stats['total_inputs'] += 1
            
            # Check confidence levels
            if intent_msg.confidence < 0.3:
                rospy.logwarn(f"Low confidence user intent: {intent_msg.raw_text} (confidence: {intent_msg.confidence})")
            
            rospy.loginfo(f"User intent processed: {intent_msg.raw_text}")
            
        except Exception as e:
            rospy.logerr(f"Error monitoring user intent: {e}")
    
    def monitor_robot_action(self, action_msg):
        """Monitor robot action execution."""
        try:
            self.system_status['robot_controller']['status'] = 'active'
            self.system_status['robot_controller']['last_seen'] = rospy.Time.now()
            
            # Track action success/failure
            if action_msg.status == 'completed':
                self.stats['successful_actions'] += 1
            elif action_msg.status == 'failed':
                self.stats['failed_actions'] += 1
                rospy.logwarn(f"Robot action failed: {action_msg.action_type}")
            
            rospy.loginfo(f"Robot action: {action_msg.action_type} - {action_msg.status}")
            
        except Exception as e:
            rospy.logerr(f"Error monitoring robot action: {e}")
    
    def monitor_llm_response(self, response_msg):
        """Monitor LLM response processing."""
        try:
            self.system_status['llm_interpreter']['status'] = 'active'
            self.system_status['llm_interpreter']['last_seen'] = rospy.Time.now()
            
            # Update average processing time
            if response_msg.processing_time > 0:
                current_avg = self.stats['avg_processing_time']
                total_inputs = self.stats['total_inputs']
                self.stats['avg_processing_time'] = (
                    (current_avg * (total_inputs - 1) + response_msg.processing_time) / total_inputs
                )
            
            # Check for processing time anomalies
            if response_msg.processing_time > 10.0:
                rospy.logwarn(f"Slow LLM processing: {response_msg.processing_time}s")
            
            rospy.loginfo(f"LLM response processed in {response_msg.processing_time}s")
            
        except Exception as e:
            rospy.logerr(f"Error monitoring LLM response: {e}")
    
    def periodic_health_check(self, event):
        """Perform periodic health checks on all components."""
        try:
            current_time = rospy.Time.now()
            
            # Check component timeouts (30 seconds)
            timeout_duration = rospy.Duration(30.0)
            
            for component, info in self.system_status.items():
                if component == 'overall_health':
                    continue
                    
                if info['last_seen'] is None:
                    info['status'] = 'unknown'
                elif current_time - info['last_seen'] > timeout_duration:
                    info['status'] = 'timeout'
                    rospy.logwarn(f"Component {component} timeout detected")
                else:
                    info['status'] = 'active'
            
            # Determine overall health
            active_components = sum(1 for comp, info in self.system_status.items() 
                                  if comp != 'overall_health' and info['status'] == 'active')
            total_components = len(self.system_status) - 1  # Exclude overall_health
            
            if active_components == total_components:
                self.system_status['overall_health'] = 'healthy'
            elif active_components > total_components // 2:
                self.system_status['overall_health'] = 'degraded'
            else:
                self.system_status['overall_health'] = 'critical'
            
            # Publish status updates
            self.publish_status()
            
        except Exception as e:
            rospy.logerr(f"Error in periodic health check: {e}")
    
    def publish_status(self):
        """Publish current system status."""
        try:
            # Create status message
            status_msg = {
                'timestamp': datetime.now().isoformat(),
                'system_status': self.system_status,
                'statistics': self.stats
            }
            
            # Publish as JSON string
            self.status_pub.publish(String(json.dumps(status_msg)))
            
            # Publish health status separately
            health_msg = {
                'overall_health': self.system_status['overall_health'],
                'active_components': sum(1 for comp, info in self.system_status.items() 
                                       if comp != 'overall_health' and info['status'] == 'active'),
                'total_components': len(self.system_status) - 1
            }
            self.health_pub.publish(String(json.dumps(health_msg)))
            
        except Exception as e:
            rospy.logerr(f"Error publishing status: {e}")
    
    def get_system_status(self):
        """Get current system status."""
        return self.system_status
    
    def get_statistics(self):
        """Get current system statistics."""
        return self.stats
    
    def run(self):
        """Main run loop."""
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = SystemMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass 