import json
import os

import rclpy
from rclpy.node import Node

from card_dealer_msgs.msg import RoomConfig, Player

import paho.mqtt.client as mqtt
from dotenv import load_dotenv


class NetpieRoomConfigNode(Node):
    def __init__(self):
        super().__init__("netpie_room_config_node")

        self.declare_parameter("env_file", "")
        env_file = self.get_parameter("env_file").get_parameter_value().string_value

        # ✅ โหลด .env จาก path ที่ส่งมา (ถ้าไม่ส่งมา จะลองหา .env ใน current folder)
        if env_file:
            load_dotenv(dotenv_path=env_file)
            self.get_logger().info(f"Loaded .env from: {env_file}")
        else:
            default_env = str(Path.cwd() / ".env")
            load_dotenv(dotenv_path=default_env)
            self.get_logger().info(f"Loaded .env from: {default_env}")

        # Load .env
        # แนะนำรันจาก workspace ก็ยังอ่านได้เพราะเรา resolve path แบบง่าย:
        load_dotenv() 
        app_id = os.getenv("NETPIE_APP_ID", "7358e1d0-7540-46d5-9abb-493964af4be1")
        key = os.getenv("NETPIE_KEY", "i5SSesdfdWTBXWhxmnMrNwiLF2DXmKJj")
        secret = os.getenv("NETPIE_SECRET", "LiPXd6n8UnhPfZrTQg9cx8TEch1SPHzG")
        host = os.getenv("NETPIE_HOST", "mqtt.netpie.io")
        port = int(os.getenv("NETPIE_PORT", "1883"))
        room_id = os.getenv("ROOM_ID", "101")

        if not app_id or not key or not secret:
            raise RuntimeError("Missing NETPIE_APP_ID / NETPIE_KEY / NETPIE_SECRET in .env")

        # MQTT topic (ปรับ prefix ให้ตรงกับทีมเว็บ)
        self.topic_room_config = f"carddealer/{room_id}/room_config"

        # ROS publisher
        self.pub = self.create_publisher(RoomConfig, "/mqtt/room_config", 10)

        # MQTT client
        # NETPIE โดยทั่วไปใช้ APP_ID เป็น client_id และ KEY/SECRET เป็น user/pass
        self.mqtt = mqtt.Client(client_id=app_id, clean_session=True)
        self.mqtt.username_pw_set(key, secret)

        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.on_disconnect = self.on_disconnect

        self.get_logger().info(f"Connecting NETPIE MQTT {host}:{port} ...")
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()

        self.get_logger().info(f"Subscribed MQTT: {self.topic_room_config}")
        self.get_logger().info("Publishing ROS: /mqtt/room_config (card_dealer_msgs/RoomConfig)")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc}")
        client.subscribe(self.topic_room_config, qos=1)

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8")

        try:
            obj = json.loads(payload)
            rcfg = obj["room_config"]
        except Exception as e:
            self.get_logger().warn(f"Invalid room_config JSON: {e}")
            return

        # Build ROS msg
        out = RoomConfig()
        out.room_code = str(rcfg.get("room_code", ""))
        out.room_status = str(rcfg.get("room_status", ""))
        out.timestamp = str(rcfg.get("timestamp", ""))
        out.num_players = int(rcfg.get("num_players", 0))
        out.card_gain = int(rcfg.get("card_gain", 0))
        out.starting_card = int(rcfg.get("starting_card", 0))

        # players: JSON เป็น dict {"0": {...}, "1": {...}}
        players_dict = rcfg.get("players", {})
        # sort key ให้เป็นลำดับ 0,1,2,3
        for k in sorted(players_dict.keys(), key=lambda x: int(x)):
            p = players_dict[k]
            pl = Player()
            pl.name = str(p.get("name", ""))
            pl.skipped = bool(p.get("skipped", False))
            pl.order = int(p.get("order", 0))
            out.players.append(pl)

        self.pub.publish(out)
        self.get_logger().info(f"Published RoomConfig: status={out.room_status}, num_players={out.num_players}")


def main():
    rclpy.init()
    node = NetpieRoomConfigNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
