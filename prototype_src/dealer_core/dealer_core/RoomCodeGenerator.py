import random
from typing import Dict, Any
from pymongo import MongoClient, errors

import os  
from dotenv import load_dotenv  

# Load .env file into environment variables
load_dotenv()

# ====== CONFIG =======
MONGO_URI       = os.getenv('MONGO_URI')
DB_NAME         = os.getenv('MONGO_DB_NAME', 'card_dealer_db')
COLLECTION_NAME = os.getenv('MONGO_COLLECTION_ROOMS', 'rooms')

# ================================

CARD_SYMBOLS = ["♠", "♥", "♦", "♣", "♤", "♧", "♡", "♢"]
VALID_ROOM_STATUSES = {"pending","running", "paused", "finished"}


def generate_room_code(length: int = 4) -> str:
    """Generate a random room code from CARD_SYMBOLS."""
    return "".join(random.choice(CARD_SYMBOLS) for _ in range(length))


def is_valid_room_code(code: str, length: int = 4) -> bool:
    """Check that code is the correct length and only uses allowed symbols."""
    if not isinstance(code, str):
        return False
    if len(code) != length:
        return False
    return all(ch in CARD_SYMBOLS for ch in code)


def build_minimal_room_document(room_code: str) -> Dict[str, Any]:
    """
    Minimal room document.

    Only room_code is meaningful at creation time.
    All config fields are blank/default so the website can fill them later.
    """
    doc = {
        "room_code": room_code,

        # Default status: waiting to be configured -> use "paused" (you can change later)
        "room_status": "pending",

        # These will be set later by website
        "num_players": 4,    # or 0 if you prefer numbers only
        "players": [],          # empty list; website can push names later
        "card_gain": 1,      # Card gain each turn (int) – set by website
        "starting_card": 5,  # Starting cards in hand (int) – set by website
    }
    return doc


def create_unique_room(
    mongo_uri: str = MONGO_URI,
    db_name: str = DB_NAME,
    collection_name: str = COLLECTION_NAME,
) -> Dict[str, Any]:
    """
    Create a room with a unique room_code and insert it into MongoDB.
    All config fields are left blank for the website to fill.

    Returns the inserted document.
    """
    client = MongoClient(mongo_uri)
    db = client[db_name]
    col = db[collection_name]

    # Ensure unique index on room_code
    col.create_index("room_code", unique=True)

    while True:
        room_code = generate_room_code()

        if not is_valid_room_code(room_code):
            continue

        if col.find_one({"room_code": room_code}) is not None:
            # already used => try again
            continue

        doc = build_minimal_room_document(room_code)

        try:
            result = col.insert_one(doc)
            doc["_id"] = result.inserted_id
            return doc

        except errors.DuplicateKeyError:
            # Rare race condition, loop and try again
            continue


def build_netpie_shadow_payload(room_doc: Dict[str, Any]) -> Dict[str, Any]:
    """
    Optional: payload to send the room info to NETPIE @shadow/data.
    """
    return {
        "data": {
            "room_code": room_doc["room_code"],
            "room_status": room_doc["room_status"],
            "num_players": room_doc["num_players"],
            "players": room_doc["players"],
            "card_gain": room_doc["card_gain"],
            "starting_card": room_doc["starting_card"],
        }
    }


if __name__ == "__main__":
    new_room = create_unique_room()
    print("New minimal room in MongoDB:")
    print(new_room)
    print(new_room["room_code"])
    netpie_payload = build_netpie_shadow_payload(new_room)
    print("\nNETPIE shadow/data payload:")
    print(netpie_payload)