import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

# Ensure you have built the custom message package first
from card_dealer_msgs.msg import RoomConfig, Player

import paho.mqtt.client as mqtt
from dotenv import load_dotenv


def _require_env(name: str) -> str:
    val = os.getenv(name)
    if val is None or val.strip() == "":
        raise RuntimeError(f"Missing required env var: {name}")
    return val.strip()


def _extract_room_config(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    Accept multiple payload shapes:
      1) {"room_config": {...}}
      2) {"data": {"room_config": {...}}}
      3) {"data": {"desired": {"room_config": {...}}}}  (shadow-like)
      4) {"data": {"reported": {"room_config": {...}}}} (shadow-like)
      5) {"data": {"state": {"desired": {...}}}} etc. (be tolerant)
    """
    if not isinstance(obj, dict):
        return None

    if "room_config" in obj and isinstance(obj["room_config"], dict):
        return obj["room_config"]

    data = obj.get("data")
    if isinstance(data, dict):
        if "room_config" in data and isinstance(data["room_config"], dict):
            return data["room_config"]

        for key in ("desired", "reported", "state"):
            sub = data.get(key)
            if isinstance(sub, dict):
                if "room_config" in sub and isinstance(sub["room_config"], dict):
                    return sub["room_config"]
                for key2 in ("desired", "reported"):
                    sub2 = sub.get(key2)
                    if isinstance(sub2, dict) and "room_config" in sub2 and isinstance(sub2["room_config"], dict):
                        return sub2["room_config"]

    # Some systems may nest under "payload"
    payload = obj.get("payload")
    if isinstance(payload, dict) and "room_config" in payload and isinstance(payload["room_config"], dict):
        return payload["room_config"]

    return None


class NetpieRoomConfigNode(Node):
    """
    Subscribes room_config from:
      - Custom topic: carddealer/{ROOM_ID}/room_config
      - NETPIE Shadow response: subscribe @private/# and publish @shadow/data/get after connect

    Publishes ROS2:
      - /mqtt/room_config   (card_dealer_msgs/RoomConfig)
    """

    def __init__(self):
        super().__init__("netpie_room_config_node")

        # ---- Load .env ----
        self.declare_parameter("env_file", "")
        env_file = self.get_parameter("env_file").get_parameter_value().string_value

        if env_file:
            load_dotenv(dotenv_path=env_file)
            self.get_logger().info(f"Loaded .env from: {env_file}")
        else:
            default_env = str(Path.cwd() / ".env")
            load_dotenv(dotenv_path=default_env)
            self.get_logger().info(f"Loaded .env from: {default_env}")

        # ---- Read NETPIE config (no hard-coded secrets) ----
        app_id = _require_env("NETPIE_APP_ID")
        key = _require_env("NETPIE_KEY")
        secret = _require_env("NETPIE_SECRET")
        host = os.getenv("NETPIE_HOST", "mqtt.netpie.io").strip()
        port = int(os.getenv("NETPIE_PORT", "1883").strip())
        room_id = os.getenv("ROOM_ID", "101").strip()

        # Topics
        self.topic_room_config = f"carddealer/{room_id}/room_config"
        self.topic_shadow_get = "@shadow/data/get"
        self.topic_private_sub = "@private/#"

        # ROS publisher
        self.pub = self.create_publisher(RoomConfig, "/mqtt/room_config", 10)

        # MQTT client
        self.mqtt = mqtt.Client(client_id=app_id, clean_session=True)
        self.mqtt.username_pw_set(key, secret)

        # Optional TLS (if NETPIE_PORT=8883 etc.)
        use_tls = os.getenv("NETPIE_USE_TLS", "false").strip().lower() in ("1", "true", "yes")
        if use_tls:
            self.mqtt.tls_set()

        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.on_disconnect = self.on_disconnect

        self.get_logger().info(f"Connecting NETPIE MQTT {host}:{port} ...")
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()

        self.get_logger().info(f"Will subscribe MQTT: {self.topic_room_config}")
        self.get_logger().info(f"Will subscribe MQTT: {self.topic_private_sub} (for shadow responses)")
        self.get_logger().info("Publishing ROS: /mqtt/room_config (card_dealer_msgs/RoomConfig)")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc}")

        # Subscribe both sources:
        client.subscribe(self.topic_room_config, qos=1)
        client.subscribe(self.topic_private_sub, qos=1)

        self.get_logger().info(f"Subscribed MQTT: {self.topic_room_config}")
        self.get_logger().info(f"Subscribed MQTT: {self.topic_private_sub}")

        # Request shadow data once connected
        client.publish(self.topic_shadow_get, payload="", qos=1)
        self.get_logger().info("Requested shadow data via @shadow/data/get")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def on_message(self, client, userdata, msg):
        topic = getattr(msg, "topic", "")
        payload = msg.payload.decode("utf-8", errors="ignore")

        # Helpful debug (truncate long payload)
        self.get_logger().debug(f"MQTT RX topic={topic} payload={payload[:300]}")

        try:
            obj = json.loads(payload)
        except Exception as e:
            self.get_logger().warn(f"MQTT message is not valid JSON (topic={topic}): {e}")
            return

        rcfg = _extract_room_config(obj)
        if rcfg is None:
            # Not a room_config payload; keep silent-ish but informative
            self.get_logger().warn(f"No room_config found in MQTT payload (topic={topic})")
            return

        out = self._build_room_config_msg(rcfg)
        self.pub.publish(out)
        self.get_logger().info(
            f"Published /mqtt/room_config from topic={topic} "
            f"(status={out.room_status}, num_players={out.num_players})"
        )

    def _build_room_config_msg(self, rcfg: Dict[str, Any]) -> RoomConfig:
        out = RoomConfig()
        out.room_code = str(rcfg.get("room_code", ""))
        out.room_status = str(rcfg.get("room_status", ""))
        out.timestamp = str(rcfg.get("timestamp", ""))
        out.num_players = int(rcfg.get("num_players", 0) or 0)
        out.card_gain = int(rcfg.get("card_gain", 0) or 0)
        out.starting_card = int(rcfg.get("starting_card", 0) or 0)

        # players can be dict {"0": {...}} or list [{"name":...}, ...]
        players_val = rcfg.get("players", {})
        if isinstance(players_val, dict):
            keys = sorted(players_val.keys(), key=lambda x: int(x) if str(x).isdigit() else str(x))
            for k in keys:
                p = players_val.get(k, {})
                if not isinstance(p, dict):
                    continue
                out.players.append(self._build_player_msg(p))
        elif isinstance(players_val, list):
            for p in players_val:
                if isinstance(p, dict):
                    out.players.append(self._build_player_msg(p))

        return out

    @staticmethod
    def _build_player_msg(p: Dict[str, Any]) -> Player:
        pl = Player()
        pl.name = str(p.get("name", ""))
        pl.skipped = bool(p.get("skipped", False))
        pl.order = int(p.get("order", 0) or 0)
        return pl


def main():
    rclpy.init()
    node = NetpieRoomConfigNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()