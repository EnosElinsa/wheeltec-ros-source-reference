#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from ollama_chat_ros.srv import Chat, ChatResponse  # 替换为你实际的服务消息类型
import json
import time
from typing import List, Dict, Optional

import openai

class OllamaChatNode:
    def __init__(self):
        rospy.init_node('ollama_server')

        # 创建服务
        self.chat_service = rospy.Service('chat_service', Chat, self.handle_chat_request)

        # ------------------------------------------
        # ---- 参数声明（带默认值） ----
        self.base_url = rospy.get_param('ollama_chat_ros/base_url', 'http://localhost:11434/v1')
        self.api_key = rospy.get_param('ollama_chat_ros/api_key', 'ollama')
        self.use_model = rospy.get_param('ollama_chat_ros/use_model', 'deepseek-r1')
        self.stream = rospy.get_param('ollama_chat_ros/stream', False)
        self.temperature = rospy.get_param('ollama_chat_ros/temperature', 0.5)
        self.history_length = rospy.get_param('ollama_chat_ros/history_length', 10)

        # ---- 用参数创建 openai 客户端 ----
        self.client = openai.OpenAI(base_url=self.base_url, api_key=self.api_key)

        # ---- 模型选择：优先用参数，否则自动 ----
        self.initialize_models()
        if not self.use_model:
            self.select_model()
        else:
            rospy.loginfo(f'Using forced model: {self.use_model}')
        # ---- 初始化对话历史 ----
        self.conversation_history = [{"role": "system", "content": "You are a helpful assistant"}]
        rospy.loginfo('Ollama(OpenAI-compat) Chat Server initialized')

    # ------------ 拉取模型列表 ------------
    def initialize_models(self):
        try:
            models = self.client.models.list()
            self.available_models = [m.id for m in models.data]
            rospy.loginfo(f"Available models: {', '.join(self.available_models)}")
        except Exception as e:
            rospy.logerr(f"Error getting models: {e}")
            self.available_models = []

    def select_model(self):
        if self.use_model:
            return
        if not self.available_models:
            rospy.logerr("No models available")
            return
        self.use_model = self.available_models[0]
        rospy.loginfo(f"Auto-selected model: {self.use_model}")

    # ------------ 服务回调 ------------
    def handle_chat_request(self, req):
        try:
            user_message = req.content
            self.conversation_history.append({"role": "user", "content": user_message})
            rospy.loginfo(f"Received: {user_message}")

            ts = time.time()
            reply = self.get_response(self.conversation_history)
            resp = ChatResponse()
            rospy.loginfo(f"Response ({time.time()-ts:.2f}s): {reply}")

            if reply:
                self.conversation_history.append({"role": "assistant", "content": reply})
                self.conversation_history = self.process_data(self.conversation_history)

                resp.content = reply
                resp.model = self.use_model
                resp.is_done = True
            else:
                resp.content = "Error processing request"
                resp.model = self.use_model
                resp.is_done = False
        except Exception as e:
            rospy.logerr(f"Error processing request: {e}")
            resp.content = "Error processing request"
            resp.model = self.use_model if self.use_model else ""
            resp.is_done = False
        return resp

    # ------------ 真正调用大模型 ------------
    def get_response(self, messages: List[Dict[str, str]]) -> Optional[str]:
        try:
            if not self.stream:
                # 非流式
                resp = self.client.chat.completions.create(
                    model=self.use_model,
                    messages=messages,
                    temperature=self.temperature,
                    stream=False
                )
                return resp.choices[0].message.content

            # 流式（如需）
            full = ""
            for chunk in self.client.chat.completions.create(
                    model=self.use_model,
                    messages=messages,
                    temperature=self.temperature,
                    stream=True
                    ):
                delta = chunk.choices[0].delta.content or ""
                full += delta
            return full

        except Exception as e:
            rospy.logerr(f"OpenAI API error: {e}")
            return None

    # ------------ 其他工具函数 ------------
    def process_data(self, data_list: List[Dict[str, str]]) -> List[Dict[str, str]]:
        return data_list[-self.history_length:]


if __name__ == '__main__':
    node = OllamaChatNode()
    rospy.spin()