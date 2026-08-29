#!/usr/bin/env python
import rospy
from ollama_chat_ros.srv import Chat, ChatResponse  # 替换为你实际的服务消息类型
import sys,time

class ChatClientNode:
    def __init__(self):
        rospy.init_node('ollama_chat_client')
        self.client = rospy.ServiceProxy('chat_service', Chat)  # 替换为你实际的服务消息类型
        rospy.loginfo('Chat Client Node initialized')

    def send_message(self, message: str):
        """Send a chat message"""
        rospy.wait_for_service('chat_service')
        rospy.loginfo("Received message: %s", message)
        try:
            response = self.client(message)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

def main(args=None):
    client_node = ChatClientNode()
    print("Chat Client Node is running")
    try:
        while True:
            user_input = input("\nuser: ")
            if user_input.lower() == 'exit':
                break
            response = client_node.send_message(user_input)
            if response:
                print(response.content, end='', flush=True)
                print("\nresponse done.")
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print("error:",e)
    finally:
        sys.exit(0)

if __name__ == '__main__':
    main()
