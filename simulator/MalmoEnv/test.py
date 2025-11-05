


import os
from openai import OpenAI
import requests
import base64
import requests
import os
from openai import OpenAI
import base64
    
def test1():
    # 1. 首先将本地图片上传到临时存储或OSS，获取URL
    # 这里假设已经获取到图片URL
    image_url = "https://gitee.com/hou-yunlong817/imagehosting/blob/master/malmo_obs.png"

    client = OpenAI(
        api_key="sk-5b18113a2baf4815a51ddf8a78eb3c98",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )

    def check_image_url(url):
        try:
            response = requests.head(url, timeout=10)
            print(f"URL: {url}")
            print(f"Status Code: {response.status_code}")
            print(f"Content-Type: {response.headers.get('Content-Type')}")
            print(f"Content-Length: {response.headers.get('Content-Length')}")
            return response.status_code == 200 and 'image' in response.headers.get('Content-Type', '')
        except Exception as e:
            print(f"Error checking URL: {e}")
            return False

    # 测试你的URL
    image_url = "https://gitee.com/hou-yunlong817/imagehosting/raw/master/malmo_depth.png"
    if check_image_url(image_url):
        print("✅ URL有效，可以继续调用API")
    else:
        print("❌ URL无效，请检查")

    response = client.chat.completions.create(
        model="qwen3-vl-plus",  # 使用Qwen3-VL-Plus模型
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "请描述这张图片的内容"},
                    {"type": "image_url", "image_url": {"url": image_url}}
                ]
            }
        ]
    )

    print(response.choices[0].message.content)
    
    
def test2():


    # 1. 读取本地图片文件并转换为base64编码
    def image_to_base64(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    # 假设你的图片在当前目录下的obs.png
    image_path = "malmo_obs.png"
    base64_image = image_to_base64(image_path)

    # 2. 构建Data URL格式
    # 根据图片类型选择正确的mime类型，PNG图片使用image/png
    data_url = f"data:image/png;base64,{base64_image}"  # [[8]]

    client = OpenAI(
        api_key="sk-5b18113a2baf4815a51ddf8a78eb3c98",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )

    response = client.chat.completions.create(
        model="qwen3-vl-plus",  # 使用Qwen3-VL-Plus模型
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "请描述这张图片的内容,识别出其中的物体。以json格式返回"},
                    {"type": "image_url", "image_url": {"url": data_url}}  # [[1]]
                ]
            }
        ]
    )

    print(response.choices[0].message.content)
    
    
if __name__ == "__main__":
    test2()