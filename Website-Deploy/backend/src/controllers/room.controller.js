// src/controllers/room.controller.js
import Room from "../models/Room.model.js";
import { publishRoomConfig } from "../services/netpie.service.js";

// POST /api/rooms/join  { roomCode }
export async function joinRoom(req, res) {
  try {
    const { roomCode } = req.body;

    if (!roomCode || typeof roomCode !== "string") {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const room = await Room.findOne({ room_code: roomCode }).exec();
    if (!room) return res.status(404).json({ error: "Room code not found" });

    if (room.room_status === "finished") {
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

// POST /api/rooms/start
// body: { roomCode, numPlayers, cardGain, startingCards, players }
export async function startRoom(req, res) {
  try {
    const { roomCode, numPlayers, cardGain, startingCards, players } = req.body;

    if (!roomCode) return res.status(400).json({ error: "roomCode is required" });

    const playersWithOrder = Array.isArray(players)
      ? players.map((p, index) => ({
          name: p?.name ?? `P${index + 1}`,
          skipped: !!p?.skipped,
          order: index + 1,
        }))
      : [];

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      {
        $set: {
          room_status: "running",
          num_players: Number(numPlayers || playersWithOrder.length || 0),
          card_gain: Number(cardGain || 0),
          starting_card: Number(startingCards || 0),
          players: playersWithOrder,
          timestamp: "00:00:00",
        },
      },
      { new: true, runValidators: true }
    ).exec();

    if (!room) return res.status(404).json({ error: "Room not found" });

    const netpiePayload = publishRoomConfig(room); // flat object is fine
    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("startRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// POST /api/rooms/pause  { roomCode }
export async function pauseRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) return res.status(400).json({ error: "roomCode is required" });

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      { $set: { room_status: "paused" } },
      { new: true, runValidators: true }
    ).exec();

    if (!room) return res.status(404).json({ error: "Room code not found" });

    const netpiePayload = publishRoomConfig(room);
    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("pauseRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// POST /api/rooms/stop  { roomCode }
export async function stopRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) return res.status(400).json({ error: "roomCode is required" });

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      { $set: { room_status: "finished" } },
      { new: true, runValidators: true }
    ).exec();

    if (!room) return res.status(404).json({ error: "Room code not found" });

    const netpiePayload = publishRoomConfig(room);
    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("stopRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}
