#!/usr/bin/env python3
import rclpy
import json
import os
from rclpy.node import Node
from my_command_pkg.srv import Command 
from sentence_transformers import SentenceTransformer, util
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('voice_control')
json_path = os.path.join(pkg_share, 'commands.json')

with open(json_path, 'r', encoding='utf-8') as f:
    commands_json = json.load(f)

model = SentenceTransformer('paraphrase-multilingual-MiniLM-L12-v2')

actions_list = commands_json["actions"]
rooms_dict = commands_json["rooms"]

class Voice_speech(Node):
    def __init__(self):
        super().__init__('voice_speech')
        self.srv = self.create_service(Command, 'voice_speech', self.text_process)
        self.get_logger().info('Command service is up and running.')

    def find_action(self,user_input):
        for action in actions_list:
            if action in user_input:
                print(f"\naction: {action}")
                return action
        print("\nnot found action")
        return None
    
    def find_room(self, user_input, phrase_dict):
        for key, phrases in phrase_dict.items():
            for phrase in phrases:
                if phrase in user_input:
                    print(f"\nroom: {key} from '{phrase}'")
                    return key
        return None
    
    def respond_room(self, action, room):
        if action and room:
            response_text = rooms_dict[room]["response"]
            position = rooms_dict[room]["position"] 
            self.get_logger().info(f"go to room: {response_text} at position {position}")
            return position  # คืนแค่ position
        return [0, 0, 0]  # default ถ้าไม่เจอ action หรือ room

    def text_process(self, request, response):
        command = request.command

        # หา action และ room
        action = self.find_action(command)
        room = self.find_room(command, {k: v["commands"] for k, v in rooms_dict.items()})

        # ได้ position
        position = self.respond_room(action, room)  # ส่งทั้ง action และ room

        if action and room :
            response.pos_x = int(position[0])
            response.pos_y = int(position[1])
            response.pos_z = int(position[2])
        else:
            response.pos_x = 0
            response.pos_y = 0
            response.pos_z = 0
            self.get_logger().info("No command")

        self.get_logger().info(f"Received command: {command} | room: {room} | position: {position}")

        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = Voice_speech()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
