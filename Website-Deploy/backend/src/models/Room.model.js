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

const RoomConfigSchema = new mongoose.Schema(
  {
    room_code: { type: String, required: true },
    room_status: {
      type: String,
      enum: ["pending", "running", "paused", "finished"],
      default: "pending",
    },
    players: { type: [PlayerSchema], default: [] },
    timestamp: { type: String, default: "00:00:00" },
    num_players: { type: Number, default: 0 },
    card_gain: { type: Number, default: 0 },
    starting_card: { type: Number, default: 0 },
  },
  { _id: false }
);

const RoomSchema = new mongoose.Schema(
  {
    room_config: { type: RoomConfigSchema, required: true },
  },
  { timestamps: true }
);

// Unique by room code
RoomSchema.index({ "room_config.room_code": 1 }, { unique: true });

export default mongoose.models.Room || mongoose.model("Room", RoomSchema);
