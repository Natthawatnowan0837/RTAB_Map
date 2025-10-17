#!/usr/bin/env python3
import rclpy
import json
import os
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# ----------------------------
# โหลด commands.json
# ----------------------------
pkg_share = get_package_share_directory('voice_control')
json_path = os.path.join(pkg_share, 'commands.json')

with open(json_path, 'r', encoding='utf-8') as f:
    commands_json = json.load(f)

actions_list = commands_json["actions"]
rooms_dict = commands_json["rooms"]


class Voice_processor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Subscriber รอรับข้อมูล action/room จาก topic /voice_cmd
        self.create_subscription(String, '/voice_cmd', self.voice_cmd_callback, 10)

        # Publisher สำหรับส่งค่าพิกัดออกไป (optional)
        self.pub_pos = self.create_publisher(String, '/voice_position', 10)

        self.get_logger().info("voice_processor node is ready and waiting for /voice_cmd.")

    # ----------------------------
    #  ฟังก์ชันหาพิกัดของห้อง
    # ----------------------------
    def respond_room(self, action, room):
        if action and room:
            response_text = rooms_dict[room]["response"]
            position = rooms_dict[room]["position"]
            self.get_logger().info(f"go to room: {response_text} at position {position}")
            return position
        else:
            self.get_logger().info("No valid action or room")
            return [0, 0, 0]

    # ----------------------------
    #  callback เมื่อมีข้อความจาก /voice_cmd
    # ----------------------------
    def voice_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get("action")
            room = data.get("room")
            raw_text = data.get("raw_text")

            self.get_logger().info(f"Received topic: action={action}, room={room}, text='{raw_text}'")

            # เรียกหาพิกัดจาก respond_room
            position = self.respond_room(action, room)

            # ส่งผลลัพธ์ออกทาง topic /voice_position (optional)
            pos_msg = String()
            pos_msg.data = json.dumps({
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "room": room,
                "action": action
            }, ensure_ascii=False)
            self.pub_pos.publish(pos_msg)

            self.get_logger().info(f"Published position: {pos_msg.data}")
            
            if room:
                os.system(f'espeak -vth "{room}" 2>/dev/null')
                
        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Voice_processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
