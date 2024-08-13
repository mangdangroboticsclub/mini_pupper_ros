#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import PIL
import cv2
import base64
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from langchain_google_vertexai import ChatVertexAI
from langchain_core.messages import HumanMessage
from io import BytesIO
from std_msgs.msg import String


def extract_keyword_constant(input_string):
    input_string = input_string.strip()
    parts = input_string.split()

    keyword = ""
    proportion = 0.0
    orientation = ""

    for part in parts:
        if part.lower() in ['left', 'right', 'center']:
            keyword = part.lower()
        elif part.lower() in ['vertical', 'slanted']:
            orientation = part.lower()
        else:
            proportion = part

    return keyword, proportion, orientation


def ai_image_response(llm, image, text):
    buffered = BytesIO()
    image.save(buffered, format="JPEG")
    image_bytes = buffered.getvalue()

    image_base64 = base64.b64encode(image_bytes).decode('utf-8')
    image_data_url = f"data:image/jpeg;base64,{image_base64}"

    image_message = {
        "type": "image_url",
        "image_url": {
            "url": image_data_url
        }
    }
    text_message = {"type": "text", "text": text}

    message = HumanMessage(content=[text_message, image_message])

    output = llm.invoke([message])
    result = output.content

    return result


class AiImageResponse(Node):
    def __init__(self):
        super().__init__('mini_pupper_maze_service')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_recognition, 10)
        self.direction_publisher_ = self.create_publisher(String, 'direction', 10)
        self.extent_publisher_ = self.create_publisher(String, 'extent_of_movement', 10)
        self.orientation_publisher_ = self.create_publisher(String, 'orientation_of_movement', 10)

    def image_recognition(self, msg):
        direction_input_prompt = """
        There is a black line in the image. You are an expert in identifying the precise 
        position and orientation of the black line from the image.

        The output should follow this format:

        [direction] [proportion] [orientation]

        Where:
        - [direction] is either "left", "right", or "center" depending on the orientation 
        of the line in the image.
        - For vertical lines, "left" means the line is positioned more towards the left 
        side of the image, and "right" means the line is positioned more towards the 
        right side.
        - For slanted lines, "left" means the line is slanted like "\" (going from 
        top-left to bottom-right), and "right" means the line is slanted like "/" 
        (going from top-right to bottom-left). This indicates the orientation of the 
        line itself, not its position in the image.
        - [proportion] is a float value between 0.0 and 1.0 indicating the relative 
        position or orientation of the line. The proportion should be calculated as:
        - If the line is vertical, 
        the proportion = 1 - |center_position - line_position| / total_width
        - If the line is slanted, 
        the proportion = (1 / slope - min_slope) / (max_slope - min_slope), 
        where min_slope and max_slope are the minimum and maximum possible slopes 
        for the given image size
        - [orientation] is either "vertical" or "slanted" depending on the overall 
        shape of the line in the image.

        It's important to note that for slanted lines, the "left" and "right" directions 
        refer to the orientation of the line itself, not its position in the image. So 
        a line that is slanted like "\" would be considered "left", and a line slanted 
        like "/" would be considered "right", regardless of where the line is positioned 
        in the image.

        If the line is seen to be positioned more towards the left side of the image 
        (for vertical lines) or slanted like "\" (for the orientation of the line), 
        output "left [proportion] [orientation]".
        If the line is seen to be positioned more towards the right side of the image 
        (for vertical lines) or slanted like "/" (for the orientation of the line), 
        output "right [proportion] [orientation]".
        If the line is seen to be positioned in the center of the image, output "center 
        (proportion greater than 0.0) [orientation]".
        If no line is detected (empty image), output "empty 1.0 vertical".

        The proportion value should accurately reflect the true position or orientation 
        of the line, with values between 0.0 and 1.0. A proportion of 0.0 is not 
        allowed for "left" or "right" directions, as that would indicate a line 
        strictly in the center.

        To determine the orientation, observe the overall shape of the line in the image. 
        If the line is strictly vertical (no slope), the orientation should be "vertical". 
        If the line has a noticeable slope, the orientation should be "slanted".

        Your output should accurately reflect the true position and orientation of the 
        line in the image, and follow the specified format exactly.
        """

        multi_model = ChatVertexAI(model="gemini-pro-vision")
        self.bridge = CvBridge()
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = PIL.Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        direction_response = extract_keyword_constant(
            ai_image_response(multi_model, image=image, text=direction_input_prompt)
        )
        self.get_logger().info(f"Direction response: {direction_response}")

        message1 = String()
        direction = direction_response[0]
        message1.data = direction
        self.direction_publisher_.publish(message1)

        message2 = String()
        extent = str(direction_response[1])
        message2.data = extent
        self.extent_publisher_.publish(message2)

        message3 = String()
        orientation = direction_response[2]
        message3.data = orientation
        self.orientation_publisher_.publish(message3)


def main():
    rclpy.init()
    minimal_service = AiImageResponse()
    rclpy.spin(minimal_service)


if __name__ == '__main__':
    main()
