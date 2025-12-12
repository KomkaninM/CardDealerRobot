// src/server.js
import "dotenv/config.js";
import express from "express";
import cors from "cors";
import { connectDB } from "./config/db.js";
import roomRoutes from "./routes/room.routes.js";

const app = express();
const PORT = process.env.PORT || 3222;

app.use(cors());
app.use(express.json());

// Health check
app.get("/api/health", (req, res) => {
  res.json({ status: "ok" });
});

// Room routes
app.use("/api/rooms", roomRoutes);

connectDB().then(() => {
  app.listen(PORT, () => {
    console.log(` Server listening on http://localhost:${PORT}`);
  });
});
