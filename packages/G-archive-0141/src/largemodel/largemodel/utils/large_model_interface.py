from dashscope import Application
import dashscope
from http import HTTPStatus
from openai import OpenAI, AuthenticationError, APIError
import os,sys
from ament_index_python.packages import get_package_share_directory
from largemodel.utils.promot import get_prompt
import yaml
import base64
import requests
import json
import netifaces
import base64
import time
import functools
def measure_execution_time(func):
    """
    装饰器：测量函数执行时间并使用 ROS 日志打印结果
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        start_time = time.time()
        result = func(self, *args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        
        # 使用 ROS 日志系统记录执行时间
        if hasattr(self, 'get_logger'):
            self.get_logger().info(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        else:
            print(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        return result
    return wrapper

class ModelInitError(RuntimeError):
    """供上层捕获的致命错误"""
    pass

class model_interface:
    def __init__(self):
        self.init_config_param()
        dashscope.api_key = self.tongyi_api_key

    def init_config_param(self):
        self.pkg_path = get_package_share_directory("largemodel")
        config_param_file = os.path.join(
            self.pkg_path, "config", "model_config.yaml"
        )
        with open(config_param_file, "r") as file:
            config_param = yaml.safe_load(file)
        self.tongyi_api_key = config_param.get("tongyi_api_key")
        self.tongyi_app_id = config_param.get("tongyi_app_id")
        self.tongyi_base_url = config_param.get("tongyi_base_url")
        self.multimodel = config_param.get("multimodel")


    def init_Multimodel(self):
        self.multimodel_client = OpenAI(
            api_key=self.tongyi_api_key, base_url=self.tongyi_base_url
        )
        self.init_Multimodel_history(get_prompt())
        while True:
            try:
                model_list=self.multimodel_client.models.list(timeout=5)
                break
            except AuthenticationError as e:
                raise ModelInitError("大模型密钥无效") from e
            except APIError as e:
                print("大模型接口无效",flush=True)
            except Exception as e:
                print(f"大模型连接失败: {e}",flush=True)
            time.sleep(1)


    def init_Multimodel_history(self, system_prompt):
        self.Multimodelmessages = []
        self.Multimodelmessages.append(
            {"role": "user", "content": [{"type": "text", "text": system_prompt}]}
        )
        self.Multimodelmessages.append(
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "#地图映射Map mapping",
                    }
                ],
            }
        )
        self.Multimodelmessages.append(
            {
                "role": "assistant",
                "content": [
                    {
                        "type": "text",
                        "text": "我已经记住所有规则、动作函数和案例了，请开始您的指令吧",
                    }
                ],
            }
        )
        
    def init_Map_mapping(self,map_mapping):
        self.Multimodelmessages[1]["content"][0]["text"] = map_mapping
        # print(self.Multimodelmessages[1])
    

    def TaskDecision(self, user_input: str):  # 任务决策规划
        try:
            response = Application.call(
                api_key=self.tongyi_api_key, app_id=self.tongyi_app_id, prompt=user_input
            )
            if response.status_code == HTTPStatus.OK:
                reply = response.output.text
                self.Multimodelmessages.append({
                    "role": "user",
                    "content": [{"type": "text", "text": user_input}]
                })
                self.Multimodelmessages.append({
                    "role": "assistant",
                    "content": [{"type": "text", "text": reply}]
                })
                print(f"[大模型] {reply}", flush=True)
                return True, reply
            return False, f"code={response.status_code} message={response.message} "
        except Exception as e:
            return False, f"{e}"

    def multimodelinfer(self, prompt, image_path=None, seewhat_func=False):
        """version: 2.0
        通用多模态接口，适用于通义千问平台的多模态模型
        """
        if image_path:
            image_data = self.encode_image(image_path)
            conversation_entry = {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{image_data}"},
                    },
                    {"type": "text", "text": prompt},
                ],
            }
            if seewhat_func:
                completion = self.multimodel_client.chat.completions.create(
                    model=self.multimodel, messages=[conversation_entry],
                )
            else:
                self.Multimodelmessages.append(conversation_entry)
                completion = self.multimodel_client.chat.completions.create(
                    model=self.multimodel, messages=self.Multimodelmessages,
                )
        else:
            conversation_entry = {
                "role": "user",
                "content": [{"type": "text", "text": prompt}],
            }
            self.Multimodelmessages.append(conversation_entry)
            completion = self.multimodel_client.chat.completions.create(
                model=self.multimodel, 
                messages=self.Multimodelmessages,
                response_format={"type":"json_object"}
            )

        self.Multimodelmessages.append(
            {
                "role": "assistant",
                "content": [
                    {"type": "text", "text": completion.choices[0].message.content}
                ],
            }
        )
        return completion.choices[0].message.content


    @staticmethod
    def encode_image(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    @staticmethod
    def get_ip(network_interface):
        addresses = netifaces.ifaddresses(network_interface)
        if netifaces.AF_INET in addresses:
            for info in addresses[netifaces.AF_INET]:
                if "addr" in info:
                    return info["addr"]
