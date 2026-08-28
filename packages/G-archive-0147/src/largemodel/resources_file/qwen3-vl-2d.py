import json
import ast
import os
import requests

from io import BytesIO
from PIL import Image, ImageDraw, ImageFont
from PIL import ImageColor
from openai import OpenAI
import base64


additional_colors = [colorname for (colorname, colorcode) in ImageColor.colormap.items()]

def decode_json_points(text: str):
    """Parse coordinate points from text format"""
    try:
        # 清理markdown标记
        if "```json" in text:
            text = text.split("```json")[1].split("```")[0]

        # 解析JSON
        data = json.loads(text)
        points = []
        labels = []

        for item in data:
            if "point_2d" in item:
                x, y = item["point_2d"]
                points.append([x, y])

                # 获取label，如果没有则使用默认值
                label = item.get("label", f"point_{len(points)}")
                labels.append(label)

        return points, labels

    except Exception as e:
        print(f"Error: {e}")
        return [], []


def plot_bounding_boxes(img_path, bounding_boxes):

    """
        在图像上绘制边界框，并标注名称
    Args:
        img_path: 图像的路径
        bounding_boxes: 包含对象名称的边界框列表，并且位置为标准化的[y1 x1 y2 x2]格式。
    """

    # 加载图像并创建绘图对象
    img = img_path
    width, height = img.size
    print(img.size)

    draw = ImageDraw.Draw(img)

    # 定义颜色列表用于区分不同对象
    colors = [
                 'red',
                 'green',
                 'blue',
                 'yellow',
                 'orange',
                 'pink',
                 'purple',
                 'brown',
                 'gray',
                 'beige',
                 'turquoise',
                 'cyan',
                 'magenta',
                 'lime',
                 'navy',
                 'maroon',
                 'teal',
                 'olive',
                 'coral',
                 'lavender',
                 'violet',
                 'gold',
                 'silver',
             ] + additional_colors

    # 解析边界框信息
    bounding_boxes = parse_json(bounding_boxes)

    font = ImageFont.load_default()

    try:
        json_output = ast.literal_eval(bounding_boxes)
    except Exception as e:
        end_idx = bounding_boxes.rfind('"}') + len('"}')
        truncated_text = bounding_boxes[:end_idx] + "]"
        json_output = ast.literal_eval(truncated_text)

    if not isinstance(json_output, list):
        json_output = [json_output]

    # 绘制每个边界框
    for i, bounding_box in enumerate(json_output):
        color = colors[i % len(colors)]

        # 将标准化坐标映射到原图上，变为绝对坐标
        abs_y1 = int(bounding_box["bbox_2d"][1] / 1000 * height)
        abs_x1 = int(bounding_box["bbox_2d"][0] / 1000 * width)
        abs_y2 = int(bounding_box["bbox_2d"][3] / 1000 * height)
        abs_x2 = int(bounding_box["bbox_2d"][2] / 1000 * width)

        if abs_x1 > abs_x2:
            abs_x1, abs_x2 = abs_x2, abs_x1

        if abs_y1 > abs_y2:
            abs_y1, abs_y2 = abs_y2, abs_y1

        # 绘制矩形框
        draw.rectangle(
            ((abs_x1, abs_y1), (abs_x2, abs_y2)), outline=color, width=3
        )

        # 添加标签文字
        #if "label" in bounding_box:
        #    draw.text((abs_x1 + 8, abs_y1 + 6), bounding_box["label"], fill=color, font=font)

    # 显示最终图像
    img.show()


def plot_points(im, text):
    img = im
    width, height = img.size
    draw = ImageDraw.Draw(img)
    colors = [
                 'red', 'green', 'blue', 'yellow', 'orange', 'pink', 'purple', 'brown', 'gray',
                 'beige', 'turquoise', 'cyan', 'magenta', 'lime', 'navy', 'maroon', 'teal',
                 'olive', 'coral', 'lavender', 'violet', 'gold', 'silver',
             ] + additional_colors

    points, descriptions = decode_json_points(text)
    print("Parsed points: ", points)
    print("Parsed descriptions: ", descriptions)
    if points is None or len(points) == 0:
        img.show()
        return

    font = ImageFont.truetype("NotoSansCJK-Regular.ttc", size=14)

    for i, point in enumerate(points):
        color = colors[i % len(colors)]
        abs_x1 = int(point[0]) / 1000 * width
        abs_y1 = int(point[1]) / 1000 * height
        radius = 2
        draw.ellipse([(abs_x1 - radius, abs_y1 - radius), (abs_x1 + radius, abs_y1 + radius)], fill=color)
        draw.text((abs_x1 - 20, abs_y1 + 6), descriptions[i], fill=color, font=font)

    img.show()


def plot_points_json(im, text):
    img = im
    width, height = img.size
    draw = ImageDraw.Draw(img)
    colors = [
                 'red', 'green', 'blue', 'yellow', 'orange', 'pink', 'purple', 'brown', 'gray',
                 'beige', 'turquoise', 'cyan', 'magenta', 'lime', 'navy', 'maroon', 'teal',
                 'olive', 'coral', 'lavender', 'violet', 'gold', 'silver',
             ] + additional_colors
    font = ImageFont.truetype("NotoSansCJK-Regular.ttc", size=14)

    text = text.replace('```json', '')
    text = text.replace('```', '')
    data = json.loads(text)
    for item in data:
        point_2d = item['point_2d']
        label = item['label']
        x, y = int(point_2d[0] / 1000 * width), int(point_2d[1] / 1000 * height)
        radius = 2
        draw.ellipse([(x - radius, y - radius), (x + radius, y + radius)], fill=colors[0])
        draw.text((x + 2 * radius, y + 2 * radius), label, fill=colors[0], font=font)

    img.show()


# 解析JSON输出
def parse_json(json_output):
    # 移除Markdown代码块标记
    lines = json_output.splitlines()
    for i, line in enumerate(lines):
        if line == "```json":
            json_output = "\n".join(lines[i + 1:])  # 删除 "```json"之前的所有内容
            json_output = json_output.split("```")[0]  # 删除 "```"之后的所有内容
            break  # 找到"```json"后退出循环
    return json_output

#  编码函数： 将本地文件转换为 Base64 编码的字符串
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")



# 调用Qwen3-VL的 API
def inference_with_api(prompt, sys_prompt="You are a helpful assistant.", model_id="qwen3-vl-plus",
                       min_pixels=4 * 32 * 32, max_pixels=2560 * 32 * 32):
    client = OpenAI(
        # 若没有配置环境变量，请用阿里云百炼API Key将下行替换为：api_key="sk-xxx",
        # api_key=os.getenv("api_key"),
        api_key="REDACTED_PUBLIC_REFERENCE",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    )
    local_img = "image.png" 
    # 将xxxx/eagle.png替换为你本地图像的绝对路径
    base64_image = encode_image(local_img)
    messages = [
        {
            "role": "system",
            "content": [{"type": "text", "text": sys_prompt}]},
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "min_pixels": min_pixels,
                    "max_pixels": max_pixels,
                    "image_url": {"url": f"data:image/png;base64,{base64_image}"},
                },
                {"type": "text", "text": prompt},
            ],
        }
    ]
    completion = client.chat.completions.create(
        model=model_id,
        messages=messages,

    )
    return completion.choices[0].message.content



# 新增：直接读本地文件
def run_object_detection_local(img_path, model_response):
    """
    img_path : str 本地图片路径，例如 "image.png"
    model_response : str 模型返回的 JSON 字符串
    """
    image = Image.open(img_path).convert("RGB")   # 支持 jpg/png/...
    plot_bounding_boxes(image, model_response)    # 复用你已有的画框函数


if __name__ == "__main__":
    local_img = "image.png"                       # <--- 放本地文件名
    prompt = """识别图片中的红色盒子，并以JSON格式输出其bbox的坐标及其中文名称"""

    # 1. 先调用 API 拿到 JSON 结果（这里仍用你原来的 inference_with_api）
    response = inference_with_api(prompt,
                                  min_pixels=64 * 32 * 32,
                                  max_pixels=2560 * 32 * 32)
    print(response)

    # 2. 本地画图
    run_object_detection_local(local_img, response)



