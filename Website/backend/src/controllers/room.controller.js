// src/controllers/room.controller.js
import Room from "../models/Room.model.js";
import { publishRoomConfig } from "../services/netpie.service.js";

// 1) JOIN: validate room code & availability
// POST /api/rooms/join
export async function joinRoom(req, res) {
  try {
    const { roomCode } = req.body;

    if (!roomCode || typeof roomCode !== "string") {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const room = await Room.findOne({ room_code: roomCode }).exec();

    if (!room) {
      return res.status(404).json({ error: "Room code not found" });
    }

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

// 2) START: update room info & set status = running แล้วส่งไป NETPIE
// Frontend calls: POST /api/rooms/start  with body { roomCode, numPlayers, cardGain, startingCards, players }
export async function startRoom(req, res) {
  const { roomCode, numPlayers, cardGain, startingCards, players } = req.body;

  try {
    if (!roomCode) {
      return res.status(400).json({ error: "roomCode is required" });
    }

    // ใส่ลำดับที่นั่งให้ player ด้วย (order: 1,2,3,...)
    const playersWithOrder = Array.isArray(players)
      ? players.map((p, index) => ({
          ...p,
          order: index + 1,
        }))
      : [];

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      {
        room_status: "running",
        num_players: numPlayers,
        card_gain: cardGain,
        starting_card: startingCards,
        players: playersWithOrder,
        updatedAt: new Date(),
      },
      { new: true }
    ).exec();

    if (!room) {
      return res.status(404).json({ error: "Room not found" });
    }

    // ส่ง room_config ทั้งก้อนขึ้น NETPIE
    const netpiePayload = publishRoomConfig(room);

    return res.json({
      ok: true,
      room,
      netpiePayload,
    });
  } catch (err) {
    console.error("startRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// 3) PAUSE: set room_status = "paused" และส่งขึ้น NETPIE
// POST /api/rooms/pause  { roomCode }
export async function pauseRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      { room_status: "paused", updatedAt: new Date() },
      { new: true }
    ).exec();

    if (!room) {
      return res.status(404).json({ error: "Room code not found" });
    }

    const netpiePayload = publishRoomConfig(room);

    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("pauseRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}

// 4) STOP: set room_status = "finished" และส่งขึ้น NETPIE
// POST /api/rooms/stop  { roomCode }
export async function stopRoom(req, res) {
  try {
    const { roomCode } = req.body;
    if (!roomCode) {
      return res.status(400).json({ error: "roomCode is required" });
    }

    const room = await Room.findOneAndUpdate(
      { room_code: roomCode },
      { room_status: "finished", updatedAt: new Date() },
      { new: true }
    ).exec();

    if (!room) {
      return res.status(404).json({ error: "Room code not found" });
    }

    const netpiePayload = publishRoomConfig(room);

    return res.json({ ok: true, room, netpiePayload });
  } catch (err) {
    console.error("stopRoom error:", err);
    return res.status(500).json({ error: "Internal server error" });
  }
}
