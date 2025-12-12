// src/controllers/room.controller.js
import Room from "../models/Room.model.js";
import { publishRoomConfig } from "../services/netpie.service.js";

/** Find room by code whether schema is FLAT or NESTED */
async function findRoomByCode(roomCode) {
  return Room.findOne({
    $or: [{ room_code: roomCode }, { "room_config.room_code": roomCode }],
  }).exec();
}

/** Update room by code whether schema is FLAT or NESTED, and write BOTH paths */
async function updateRoomByCode(roomCode, $setObj) {
  return Room.findOneAndUpdate(
    {
      $or: [{ room_code: roomCode }, { "room_config.room_code": roomCode }],
    },
    { $set: $setObj },
    { new: true, runValidators: false }
  ).exec();
}

/** Convert a room document into a "roomConfig-like" object for NETPIE */
function toRoomConfig(roomDoc) {
  // If nested schema exists, use it
  if (roomDoc?.room_config) return roomDoc.room_config;

  // Otherwise map from flat schema -> config-like
  return {
    room_code: roomDoc.room_code,
    room_status: roomDoc.room_status,
    players: roomDoc.players || [],
    timestamp: roomDoc.timestamp || "00:00:00",
    num_players: roomDoc.num_players || roomDoc.numPlayers || 0,
    card_gain: roomDoc.card_gain || roomDoc.cardGain || 0,
    starting_card: roomDoc.starting_card || roomDoc.startingCards || 0,
  };
}

// -------------------- 1) JOIN --------------------
// POST /api/rooms/join  { roomCode }
export async function joinRoom(req, res) {
  try {
    const { roomCode } = req.body;

    if (!roomCode || typeof roomCode !== "string") {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const room = await findRoomByCode(roomCode);

    if (!room) {
      return res.status(404).json({ error: "Room code not found" });
    }

    const status = room.room_config?.room_status ?? room.room_status;

    if (status === "finished") {
      return res
        .status(400)
        .json({ error: "This room is already finished and cannot be joined." });
    }

    return res.json({ ok: true, room });
  } catch (err) {
    console.error("joinRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// -------------------- 2) START --------------------
// POST /api/rooms/start
// body: { roomCode, numPlayers, cardGain, startingCards, players }
export async function startRoom(req, res) {
  try {
    const { roomCode, numPlayers, cardGain, startingCards, players } = req.body;

    if (!roomCode) {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const playersWithOrder = Array.isArray(players)
      ? players.map((p, index) => ({
          name: p?.name ?? `P${index + 1}`,
          skipped: !!p?.skipped,
          order: index + 1,
        }))
      : [];

    const room = await updateRoomByCode(roomCode, {
      // FLAT
      room_status: "running",
      num_players: numPlayers,
      card_gain: cardGain,
      starting_card: startingCards,
      players: playersWithOrder,
      timestamp: "00:00:00",

      // NESTED
      "room_config.room_status": "running",
      "room_config.num_players": numPlayers,
      "room_config.card_gain": cardGain,
      "room_config.starting_card": startingCards,
      "room_config.players": playersWithOrder,
      "room_config.timestamp": "00:00:00",
    });

    if (!room) {
      return res.status(404).json({ error: "Room not found" });
    }

    const netpiePayload = publishRoomConfig(toRoomConfig(room));

    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("startRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// -------------------- 3) PAUSE --------------------
// POST /api/rooms/pause  { roomCode }
export async function pauseRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) return res.status(400).json({ error: "roomCode is required" });

    const room = await updateRoomByCode(roomCode, {
      room_status: "paused",
      "room_config.room_status": "paused",
    });

    if (!room) return res.status(404).json({ error: "Room code not found" });

    const netpiePayload = publishRoomConfig(toRoomConfig(room));
    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("pauseRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// -------------------- 4) STOP --------------------
// POST /api/rooms/stop  { roomCode }
export async function stopRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) return res.status(400).json({ error: "roomCode is required" });

    const room = await updateRoomByCode(roomCode, {
      room_status: "finished",
      "room_config.room_status": "finished",
    });

    if (!room) return res.status(404).json({ error: "Room code not found" });

    const netpiePayload = publishRoomConfig(toRoomConfig(room));
    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("stopRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}
