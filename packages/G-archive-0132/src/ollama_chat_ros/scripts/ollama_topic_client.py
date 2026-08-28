#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import sys
import threading
import time

class ChatClientNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ollama_topic_client', anonymous=True)
        
        # 创建发布者和订阅者
        self.message_publisher = rospy.Publisher(
            'chat_message', 
            String, 
            queue_size=10
        )
        
        self.response_subscription = rospy.Subscriber(
            'chat_response',
            String,
            self.response_callback,
            queue_size=10
        )
        
        rospy.loginfo('Chat Client Node initialized')
        self.current_response = ""
        self.is_done = True

    def response_callback(self, msg):
        """Handle incoming chat responses"""
        try:
            response_data = json.loads(msg.data)
            content = response_data.get('content', '')
            self.is_done = response_data.get('is_done', True)
            print(content, end='', flush=True)
            
        except Exception as e:
            rospy.logerr(f"Error processing response: {e}")

    def send_message(self, message: str):
        """Send a chat message"""
        msg = String()
        msg.data = json.dumps({
            "content": message
        })
        self.message_publisher.publish(msg)
        self.is_done = False

def main():
    client_node = ChatClientNode()
    
    # 创建一个线程来运行ROS节点
    def spin_node():
        rospy.spin()
    
    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    time.sleep(3)
    print("Chat Client Node is running")

    try:
        while not rospy.is_shutdown():
            if client_node.is_done:
                user_input = input("\nuser: ")
                if user_input.lower() == 'exit':
                    break
                client_node.send_message(user_input)
                
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        rospy.signal_shutdown("User requested exit")
        sys.exit(0)

if __name__ == '__main__':
    main()