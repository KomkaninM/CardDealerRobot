// src/server.js
import "dotenv/config.js";
import express from "express";
import cors from "cors";
import { connectDB } from "./config/db.js";
import roomRoutes from "./routes/room.routes.js";

const app = express();

// Render provides PORT automatically
const PORT = Number(process.env.PORT || 3222);

// If you use cookies / IP-based logic behind Render proxy
app.set("trust proxy", 1);

// --------- CORS (set this on Render) ----------
// Example:
// CORS_ORIGINS=https://your-frontend.vercel.app,http://localhost:5173
const allowedOrigins = (process.env.CORS_ORIGINS || "")
  .split(",")
  .map((s) => s.trim())
  .filter(Boolean);

app.use(
  cors({
    origin: (origin, cb) => {
      // allow non-browser tools (Postman/Insomnia) or same-origin
      if (!origin) return cb(null, true);

      // if you didn't set CORS_ORIGINS yet, allow all (good for debugging)
      if (allowedOrigins.length === 0) return cb(null, true);

      if (allowedOrigins.includes(origin)) return cb(null, true);
      return cb(new Error(`CORS blocked for origin: ${origin}`));
    },
    credentials: true,
    methods: ["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
  })
);

// Body parsing
app.use(express.json({ limit: "1mb" }));
app.use(express.urlencoded({ extended: true }));

// Health check
app.get("/api/health", (req, res) => {
  res.json({ status: "ok" });
});

// Room routes
app.use("/api/rooms", roomRoutes);

// Basic error handler (so Render logs show errors clearly)
app.use((err, req, res, next) => {
  console.error("❌ API Error:", err?.message || err);
  res.status(500).json({ error: "Internal Server Error" });
});

async function start() {
  try {
    await connectDB();
    app.listen(PORT, "0.0.0.0", () => {
      console.log(`✅ Server listening on port ${PORT}`);
    });
  } catch (err) {
    console.error("❌ Failed to start server:", err?.message || err);
    process.exit(1);
  }
}

start();
