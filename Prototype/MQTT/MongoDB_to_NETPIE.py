#!/usr/bin/env python3
import json
import sys
import time
from datetime import datetime


import paho.mqtt.client as mqtt
from pymongo import MongoClient

import os  
from dotenv import load_dotenv  

# Load .env file into environment variables
load_dotenv()

# ---------------- NETPIE from .env ----------------
appid  = os.getenv('NETPIE_APP_ID')
key    = os.getenv('NETPIE_KEY')
secret = os.getenv('NETPIE_SECRET')

mqtt_host      = os.getenv('NETPIE_HOST', 'mqtt.netpie.io')
mqtt_port      = int(os.getenv('NETPIE_PORT', '1883'))
mqtt_keepalive = 60

# ---------------- MongoDB from .env ----------------
MONGO_URI       = os.getenv('MONGO_URI')
DB_NAME         = os.getenv('MONGO_DB_NAME', 'card_dealer_db')
COLLECTION_NAME = os.getenv('MONGO_COLLECTION_ROOMS', 'rooms')

# Quick safety check (optional but useful)
if not all([appid, key, secret, MONGO_URI]):
    raise RuntimeError("Missing required env vars. Check your .env file.")

# ---------------- MQTT Setup ----------------
client = mqtt.Client(client_id=appid, clean_session=True)
client.username_pw_set(username=key, password=secret)
# ---------------- Room Code Symbols ----------------
CARD_SYMBOLS = ["♠", "♥", "♦", "♣", "♤", "♧", "♡", "♢"]

def on_connect(client, userdata, flags, rc):
    print("Connected to NETPIE with result code", rc)
    client.subscribe('@msg/test')

def on_message(client, userdata, msg):
    print(f"[NETPIE] {msg.topic}: {msg.payload.decode()}")

client.on_connect = on_connect
client.on_message = on_message


# ---------------- Helper Functions ----------------
def is_valid_room_code(code: str, length: int = 4) -> bool:
    """Check that code is exactly 4 symbols and each is in CARD_SYMBOLS."""
    if not isinstance(code, str):
        return False
    if len(code) != length:
        return False
    return all(ch in CARD_SYMBOLS for ch in code)


def get_room_by_code(room_code: str):
    """Query MongoDB for a room with the given room_code."""
    client_mongo = MongoClient(MONGO_URI)
    db = client_mongo[DB_NAME]
    col = db[COLLECTION_NAME]
    doc = col.find_one({"room_code": room_code})
    return doc

def build_room_payload(room_doc):
    """
    Build payload to publish to NETPIE.

    All fields are inside data.room_config:
      data: {
        room_config: {
          room_code,
          room_status,
          num_players,
          players,
          card_gain,
          starting_card,
          timestamp
        }
      }
    """
    return {
        "data": {
            "room_config": {
                "room_code": room_doc.get("room_code"),
                "room_status": room_doc.get("room_status"),
                "num_players": room_doc.get("num_players"),
                "players": room_doc.get("players", []),
                "card_gain": room_doc.get("card_gain"),
                "starting_card": room_doc.get("starting_card"),
                "timestamp": datetime.now().strftime("%H:%M:%S"),
            }
        }
    }

def publish_room_to_netpie(room_doc):
    """Publish room configuration to NETPIE (shadow + msg)."""
    payload = build_room_payload(room_doc)
    json_payload = json.dumps(payload, ensure_ascii=False)

    # Publish to shadow (for dashboard)
    client.publish("@shadow/data/update", json_payload)

    # Optional: debug/graph topic
    client.publish("@msg/room_config", json_payload)

    print("→ Sent room config to NETPIE:")
    print(json.dumps(payload, ensure_ascii=False, indent=2))

# ---------------- Main Logic ----------------
def main():
    # 1. Connect to NETPIE
    client.connect(mqtt_host, mqtt_port, mqtt_keepalive)
    client.loop_start()
    time.sleep(0.3)  # wait for connection setup

    # 2. Get room_code input (CLI arg or prompt)
    if len(sys.argv) >= 2:
        room_code = sys.argv[1]
    else:
        room_code = input("Enter 4-symbol room code (e.g. ♦♧♦♤): ").strip()

    # 3. Validate room_code format
    if not is_valid_room_code(room_code):
        print(f"[ERROR] Invalid room code: {room_code}")
        print(f"Room code must be 4 symbols from: {''.join(CARD_SYMBOLS)}")
        client.loop_stop()
        client.disconnect()
        sys.exit(1)

    # 4. Query MongoDB
    room_doc = get_room_by_code(room_code)
    if room_doc is None:
        print(f"[ERROR] No room found in MongoDB for code: {room_code}")
        client.loop_stop()
        client.disconnect()
        sys.exit(1)

    # Debug print from DB
    print("Room found in MongoDB:")
    print({
        "room_code": room_doc.get("room_code"),
        "room_status": room_doc.get("room_status"),
        "num_players": room_doc.get("num_players"),
        "players": room_doc.get("players"),
        "card_gain": room_doc.get("card_gain"),
        "starting_card": room_doc.get("starting_card"),
    })

    # 5. Publish once to NETPIE
    publish_room_to_netpie(room_doc)

    time.sleep(2)  # ensure publish goes out

    client.loop_stop()
    client.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()

