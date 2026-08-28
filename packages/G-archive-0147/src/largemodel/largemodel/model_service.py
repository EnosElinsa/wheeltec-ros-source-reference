import os
import yaml
import json
import sys
import rclpy
from rclpy.node import Node
from interfaces.action import Progress
from std_msgs.msg import String
from largemodel.utils import large_model_interface
from largemodel.utils.large_model_interface import ModelInitError
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from largemodel.utils.promot import get_prompt

import time
import re
import functools


def measure_execution_time(func):
    """
    装饰器：测量函数执行时间并使用 ROS 日志打印结果
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        # 记录开始时间
        start_time = time.time()
        # 调用原函数
        result = func(self, *args, **kwargs)
        # 记录结束时间
        end_time = time.time()
        # 计算执行时间
        execution_time = end_time - start_time
        # 使用 ROS 日志系统记录执行时间
        if hasattr(self, 'get_logger'):
            self.get_logger().info(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        else:
            # 如果没有 ROS 日志系统，则打印执行时间
            print(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        return result
    return wrapper

class LargeModelService(Node):
    def __init__(self):
        super().__init__("LargeModelService")

        self.init_param_config()  # 初始化参数配置 
        self.init_largemodel()  # 初始化大模型 
        self.init_ros_comunication()  # 初始化ROS通信 

        self.get_logger().info(
            "LargeModelService node Initialization completed..."
        )  # 打印日志 
        msg = String(data=f"初始化完成，可直接执行功能")
        self.text_pub.publish(msg)

    def init_largemodel(self):
        try:
            # 创建模型接口客户端 
            self.model_client = large_model_interface.model_interface()
        except ModelInitError as e:
            self.fatal(str(e))
        self.model_client.init_Multimodel()  # 初始化执行层模型，决策层模型无需初始化 
        self.model_client.init_Map_mapping(
            self.get_map_mapping()
        )
        self.new_order_cycle = True  # 新指令周期标志 

    def init_param_config(self):
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        # 参数声明 
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("is_dual_model", False)
        # 获取参数服务器参数 
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.isdual_model = (
            self.get_parameter("is_dual_model").get_parameter_value().bool_value
        )
        # 设置夹取启动文件路径 
        self.map_mapping_config = os.path.join(self.pkg_path, "config", "map_mapping.yaml")
        self.seewhat_func = False


    def init_ros_comunication(self):
        # 创建执行动作状态订阅者 
        self.actionstatus_sub = self.create_subscription(
            String, "actionstatus", self.actionstatus_callback, 1
        )
        # 创建动作客户端，连接到 'action_service' 
        self._action_client = ActionClient(self, Progress, "action_service")
        # asr话题订阅者 
        self.asrsub = self.create_subscription(String, "voice_words", self.asr_callback, 1)
        # 创建seewhat订阅者 
        self.seewhat_sub = self.create_subscription(
            String, "seewhat_handle", self.seewhat_callback, 1
        )
        # 创建文字交互发布者 
        self.text_pub = self.create_publisher(String, "feedback_words", 1)
        

    def asr_callback(self, msg):
        self.action_agent(type="text", prompt=msg.data)
        
    def actionstatus_callback(self, msg):
        if (
            msg.data == "finish"
        ):  # 如果收到的是finish则表示当前指令执行完成，开启新的指令执行周期 
            self.new_order_cycle = True
            self.get_logger().info(
                f"The current instruction cycle has ended"
            )  # 当前指令周期已结束...
        # if (
        #     msg.data == "get_current_pose_success"
        # ):
        else:  # 向指令执行层大模型反馈动作执行结果 
            self.get_logger().info(
                f"action_status:{msg.data}"
            ) 


    def seewhat_callback(self, msg):
        self.get_logger().info(
                f"seewhat_use_vision_largemodel"
            ) 
        if msg.data == "seewhat":
            self.action_agent(type="image",prompt="")
        else:
            self.seewhat_func = True
            self.action_agent(type="image",prompt=(f"继续执行{msg.data}"))
    
    def action_agent(self, type, prompt):
        if self.new_order_cycle:  # 判断是否是新任务周期 
            # 判断上一轮对话指令是否完成如果完成就清空历史上下文，开启新的上下文 
            self.model_client.init_Multimodel_history(
                get_prompt()
            )  # 初始化执行层上下文历史
            self.new_order_cycle = False 
            
        if type == "text":
            self.model_client.init_Map_mapping(
                self.get_map_mapping()
            )
            if self.isdual_model :
                execute_instructions = self.model_client.TaskDecision(
                    prompt
                )
                if not execute_instructions[0]:
                    self.get_logger().error(
                        f"LargeScaleModel return: {execute_instructions[1]} ,The format was unexpected. "
                    )
        
        self.instruction_process(
            type=type, prompt=prompt
        )  # 调用执行层大模型生成成动作列表并执行 


    # @measure_execution_time
    def instruction_process(self, type, prompt):
        """
        根据输入信息的类型（文字/图片），构建不同的请求体进行推理，并返回结果）
        Based on the type of input information (text/image), construct different request bodies for inference and return the result.
        """
        if type == "text":
            raw_content = self.model_client.multimodelinfer(prompt)
            json_str = self.extract_json_content(raw_content)
            if json_str is not None:
                # 解析JSON字符串,分离"action"、"response"字段 
                action_plan_json = json.loads(json_str)
                action_list = action_plan_json.get("action", [])
                llm_response = action_plan_json.get("response", "")
            else:
                self.get_logger().info(
                    f"LargeScaleModel return: {json_str},The format was unexpected. The output format of the AI model at the execution layer did not meet the requirements"
                )
                return
            if self.text_chat_mode:
                msg = String(data=f'{llm_response}')
                self.text_pub.publish(msg)
            self.get_logger().info(
                f'"action": {action_list}, "response": {llm_response}'
            )
            self.send_action_service(
                action_list, llm_response
            )  # 异步发送动作列表、回复内容给ActionServer 
        
        elif type == "image":
            if self.seewhat_func :
                prompt_seewhat = "识别图片中的所有物体,并以JSON格式输出其bbox的坐标及其中文名称"
                bbox_json = self.model_client.multimodelinfer(
                    prompt_seewhat, image_path=self.image_save_path, seewhat_func=True
                )
                self.get_logger().info(f'{bbox_json}')
                raw_content = self.model_client.multimodelinfer(
                    prompt+bbox_json, image_path=self.image_save_path
                )
                
            else:
                prompt_seewhat = "机器人反馈:执行seewhat()完成"
                raw_content = self.model_client.multimodelinfer(
                    prompt_seewhat+prompt, image_path=self.image_save_path
                )
            json_str = self.extract_json_content(raw_content)
            if json_str is not None:
                # 解析JSON字符串,分离"action"、"response"字段 
                action_plan_json = json.loads(json_str)
                action_list = action_plan_json.get("action", [])
                llm_response = action_plan_json.get("response", "")
            else:
                self.get_logger().info(
                    f"LargeScaleModel return: {json_str},The format was unexpected. The output format of the AI model at the execution layer did not meet the requirements"
                )
                return
            if self.text_chat_mode and not self.seewhat_func:
                msg = String(data=f'{llm_response}')
                self.text_pub.publish(msg)
            self.get_logger().info(
                f'"response": {llm_response}'
            )
            self.send_action_service(
                action_list, llm_response
            )  # 异步发送动作列表、回复内容给ActionServe
            self.seewhat_func = False
            
    def send_action_service(self, actions, text):
        goal_msg = Progress.Goal()  # 创建目标消息对象 
        goal_msg.actions = actions  # 设置目标消息中的动作列表 
        goal_msg.llm_response = text
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        # 添加目标发送后的响应回调函数 
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()  # 获取目标句柄 
        if not goal_handle.accepted:
            self.get_logger().info(
                "action_client message: action service rejected action list"
            )  # 目标被拒绝...
            return
    #     self.get_logger().info(
    #         "action_client message: action service accepted action list"
    #     )
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
        
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            "Received feedback: {}".format(feedback_msg.feedback.status)
        )
        
    @staticmethod
    def extract_json_content(
        raw_content,
    ):  # 解析变量提取json 
        try:
            # 方法一：分割代码块 
            if "```json" in raw_content:
                # 分割代码块并取中间部分 
                json_str = raw_content.split("```json")[1].split("```")[0].strip()
            elif "```" in raw_content:
                # 处理没有指定类型的代码块 
                json_str = raw_content.split("```")[1].strip()
            else:
                # 直接尝试解析 
                json_str = raw_content

            # 方法二：正则表达式提取（备用方案） 
            if not json_str:

                match = re.search(r"\{.*\}", raw_content, re.DOTALL)
                if match:
                    json_str = match.group()
            return json_str
        except Exception as e:
            return None
    
    def fatal(self, msg: str):
        self.get_logger().fatal(msg)
        rclpy.shutdown()   # 停止 executor
        sys.exit(1)        # 告诉 launch 异常退出

    def get_map_mapping(self):
        '''
        获取地图映射关系
        '''
        with open(self.map_mapping_config, 'r', encoding='utf-8') as file:
            yaml_data = yaml.safe_load(file)
        map_mapping = "#地图映射\n\n"
        # 遍历 YAML 数据，提取符号和名称
        for symbol, area_info in yaml_data.items():
            name = area_info['name']
            map_mapping += f"'{symbol}': '{name}',\n"
        return map_mapping


def main(args=None):
    rclpy.init(args=args)
    model_service = LargeModelService()
    rclpy.spin(model_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
