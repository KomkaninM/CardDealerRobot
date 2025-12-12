// src/models/Room.model.js
import mongoose from "mongoose";

const playerSchema = new mongoose.Schema(
  {
    name: { type: String, required: true },
    skipped: { type: Boolean, default: false },
    order: { type: Number }, // optional – for turn order
  },
  { _id: false }
);

const roomSchema = new mongoose.Schema(
  {
    room_code: { type: String, required: true, unique: true },
    room_status: {
      type: String,
      enum: ["waiting", "running", "paused", "finished"],
      default: "waiting",
    },
    num_players: { type: Number, default: null },
    players: { type: [playerSchema], default: [] },
    card_gain: { type: Number, default: null },
    starting_card: { type: Number, default: null },
  },
  {
    timestamps: true,
  }
);

const Room = mongoose.model("Room", roomSchema);

export default Room;