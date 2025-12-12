// src/models/Room.model.js
import mongoose from "mongoose";

const PlayerSchema = new mongoose.Schema(
  {
    name: { type: String, required: true, trim: true },
    skipped: { type: Boolean, default: false },
    order: { type: Number, required: true },
  },
  { _id: false }
);

const RoomSchema = new mongoose.Schema(
  {
    room_code: { type: String, required: true, unique: true },
    room_status: {
      type: String,
      enum: ["pending", "running", "paused", "finished"],
      default: "pending",
    },
    num_players: { type: Number, default: 0 },
    players: { type: [PlayerSchema], default: [] },
    timestamp: { type: String, default: "00:00:00" },
    card_gain: { type: Number, default: 0 },
    starting_card: { type: Number, default: 0 },
  },
  { timestamps: true }
);

export default mongoose.models.Room || mongoose.model("Room", RoomSchema);
