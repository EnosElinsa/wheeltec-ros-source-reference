#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import openai, json, time
from typing import List, Dict, Optional

class OllamaTopicNode:
    def __init__(self):
        rospy.init_node('ollama_topic_server', anonymous=True)

        # ---------------- 参数 ----------------
        self.base_url      = rospy.get_param('ollama_chat_ros/base_url',   'http://localhost:11434/v1')
        self.api_key       = rospy.get_param('ollama_chat_ros/api_key',    'ollama')
        self.use_model     = rospy.get_param('ollama_chat_ros/use_model',  'deepseek-r1')
        self.stream        = rospy.get_param('ollama_chat_ros/stream',     False)
        self.temperature   = rospy.get_param('ollama_chat_ros/temperature',0.5)
        self.history_len   = rospy.get_param('ollama_chat_ros/history_length', 10)

        self.client = openai.OpenAI(base_url=self.base_url, api_key=self.api_key)

        # 模型列表初始化
        self.available_models = []
        self.initialize_models()
        if not self.use_model:                # 参数为空就自动选
            self.select_model()
        else:
            rospy.loginfo(f'Using forced model: {self.use_model}')
            
        # 对话历史
        self.history = [{"role": "system", "content": "You are a helpful assistant"}]

        # ROS 通信
        self.pub = rospy.Publisher('chat_response', String, queue_size=10)
        rospy.Subscriber('chat_message', String, self.msg_cb, queue_size=10)

        rospy.loginfo("OllamaTopicNode (ROS1) ready.")

    # ---------- 工具 ----------
    def initialize_models(self):
        try:
            models = self.client.models.list()
            self.available_models = [m.id for m in models.data]
            rospy.loginfo("Available models: %s", ', '.join(self.available_models))
        except Exception as e:
            rospy.logerr("Error getting models: %s", e)
            self.available_models = []

    def select_model(self):
        if self.use_model:
            return
        if not self.available_models:
            rospy.logerr("No models available")
            return
        self.use_model = self.available_models[0]
        rospy.loginfo("Auto-selected model: %s", self.use_model)

    # ---------- 回调 ----------
    def msg_cb(self, msg: String):
        try:
            user = json.loads(msg.data).get('content', '')
            if not user:
                return
            self.history.append({"role": "user", "content": user})
            rospy.loginfo("Received: %s", user)

            t0 = time.time()
            reply = self.get_response(self.history)
            rospy.loginfo("Response (%.2fs): %s", time.time() - t0, reply)

            if reply:
                self.history.append({"role": "assistant", "content": reply})
                self.history = self.history[-self.history_len:]
        except Exception as e:
            rospy.logerr("Error processing message: %s", e)

    # ---------- 真正调用大模型 ----------
    def get_response(self, messages: List[Dict[str, str]]) -> Optional[str]:
        try:
            resp = self.client.chat.completions.create(
                model=self.use_model,
                messages=messages,
                temperature=self.temperature,
                stream=self.stream
            )

            if not self.stream:
                content = resp.choices[0].message.content
                self._publish_chunk(content, is_done=True)
                return content

            # 流式
            full = ""
            for chunk in resp:
                delta = chunk.choices[0].delta.content or ""
                full += delta
                is_done = chunk.choices[0].finish_reason is not None
                self._publish_chunk(delta, is_done)
                if is_done:
                    return full
        except Exception as e:
            rospy.logerr("OpenAI API error: %s", e)
            return None

    # ---------- 发布 ----------
    def _publish_chunk(self, chunk: str, is_done: bool):
        out = String()
        out.data = json.dumps({
            "content": chunk,
            "model"  : self.use_model,
            "is_done": is_done
        })
        self.pub.publish(out)


if __name__ == '__main__':
    try:
        OllamaTopicNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass