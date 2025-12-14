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
room_id_env    = os.getenv('ROOM_ID', '101') 

# ---------------- MongoDB from .env ----------------
MONGO_URI       = os.getenv('MONGO_URI')
DB_NAME         = os.getenv('MONGO_DB_NAME', 'card_dealer_db')
COLLECTION_NAME = os.getenv('MONGO_COLLECTION_ROOMS', 'rooms')

# Quick safety check
if not all([appid, key, secret, MONGO_URI]):
    raise RuntimeError("Missing required env vars. Check your .env file.")

# ---------------- MQTT Setup ----------------
# [FIX 1] Reverted to using the correct appid (No UUID) so we get RC 0
client = mqtt.Client(client_id=appid, clean_session=True)
client.username_pw_set(username=key, password=secret)

# ---------------- Room Code Symbols ----------------
CARD_SYMBOLS = ["♠", "♥", "♦", "♣", "♤", "♧", "♡", "♢"]

def on_connect(client, userdata, flags, rc):
    # Log the result code
    print(f"Connected to NETPIE (ID: {appid}) with result code {rc}")

def on_message(client, userdata, msg):
    print(f"[NETPIE] {msg.topic}: {msg.payload.decode()}")

client.on_connect = on_connect
client.on_message = on_message

# ---------------- Helper Functions ----------------
def is_valid_room_code(code: str, length: int = 4) -> bool:
    if not isinstance(code, str):
        return False
    if len(code) != length:
        return False
    return all(ch in CARD_SYMBOLS for ch in code)

def get_room_by_code(room_code: str):
    client_mongo = MongoClient(MONGO_URI)
    db = client_mongo[DB_NAME]
    col = db[COLLECTION_NAME]
    doc = col.find_one({"room_code": room_code})
    return doc

def build_room_payload(room_doc):
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
    """Publish room configuration to NETPIE."""
    payload = build_room_payload(room_doc)
    json_payload = json.dumps(payload, ensure_ascii=False)

    # 1. Publish to Shadow
    # [FIX 2] Added retain=True so the listener gets it immediately after reconnecting
    client.publish("@shadow/data/update", json_payload, qos=1, retain=True)

    # 2. Publish to the specific ROS topic
    target_topic = f"carddealer/{room_id_env}/room_config"
    client.publish(target_topic, json_payload, qos=1, retain=True)

    print("→ Sent room config to NETPIE:")
    print(f"   Target Topic: {target_topic} (Retained)")
    print(json.dumps(payload, ensure_ascii=False, indent=2))

def RoomCode_to_NETPIE(room_code: str):
    try:
        client.connect(mqtt_host, mqtt_port, mqtt_keepalive)
        client.loop_start()
        time.sleep(1) # Wait for connection

        if not is_valid_room_code(room_code):
            print(f"[ERROR] Invalid room code: {room_code}")
            return

        room_doc = get_room_by_code(room_code)
        if room_doc is None:
            print(f"[ERROR] No room found in MongoDB for code: {room_code}")
            return

        publish_room_to_netpie(room_doc)
        
        # Wait for publish to finish
        time.sleep(2) 
        
    except Exception as e:
        print(f"Error publishing to NETPIE: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("Room Published to NETPIE (Connection Closed)")

# ---------------- Main Logic ----------------
def main():
    client.connect(mqtt_host, mqtt_port, mqtt_keepalive)
    client.loop_start()
    time.sleep(1)

    if len(sys.argv) >= 2:
        room_code = sys.argv[1]
    else:
        room_code = input("Enter 4-symbol room code (e.g. ♦♧♦♤): ").strip()

    if not is_valid_room_code(room_code):
        print(f"[ERROR] Invalid room code: {room_code}")
        client.loop_stop()
        client.disconnect()
        sys.exit(1)

    room_doc = get_room_by_code(room_code)
    if room_doc is None:
        print(f"[ERROR] No room found in MongoDB for code: {room_code}")
        client.loop_stop()
        client.disconnect()
        sys.exit(1)

    publish_room_to_netpie(room_doc)

    time.sleep(2)
    client.loop_stop()
    client.disconnect()
    print("Done.")
    
if __name__ == "__main__":
    main()