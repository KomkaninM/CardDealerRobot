// src/server.js
import "dotenv/config.js";
import express from "express";
import cors from "cors";
import { connectDB } from "./config/db.js";
import roomRoutes from "./routes/room.routes.js";

const app = express();
const PORT = Number(process.env.PORT || 3222);

app.set("trust proxy", 1);

// ---------- CORS ----------
const allowedOrigins = (process.env.CORS_ORIGINS || "")
  .split(",")
  .map((s) => s.trim())
  .filter(Boolean);

const corsOptions = {
  origin: (origin, cb) => {
    // allow Postman/Insomnia or server-to-server
    if (!origin) return cb(null, true);

    // if env not set yet, allow all (debugging)
    if (allowedOrigins.length === 0) return cb(null, true);

    if (allowedOrigins.includes(origin)) return cb(null, true);

    return cb(new Error(`CORS blocked for origin: ${origin}`));
  },
  credentials: true,
  methods: ["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"],
};

// IMPORTANT: must be before routes
app.use(cors(corsOptions));
// IMPORTANT: respond to preflight for ALL routes
app.options("*", cors(corsOptions));

// ---------- Parsers ----------
app.use(express.json({ limit: "1mb" }));
app.use(express.urlencoded({ extended: true }));

// ---------- Routes ----------
app.get("/api/health", (req, res) => {
  res.status(200).json({ status: "ok" });
});

// Your join URL must be: /api/rooms/join
app.use("/api/rooms", roomRoutes);

// ---------- 404 handler (helps find wrong path like /rooms/join) ----------
app.use((req, res) => {
  res.status(404).json({
    error: "Not Found",
    path: req.originalUrl,
    hint: "Try /api/health or /api/rooms/join",
  });
});

// ---------- Error handler ----------
app.use((err, req, res, next) => {
  console.error("❌ API Error:", err?.message || err);
  res.status(500).json({
    error: "Internal Server Error",
    message: err?.message || String(err),
  });
});

async function start() {
  try {
    await connectDB();
    app.listen(PORT, "0.0.0.0", () => {
      console.log(`✅ Server listening on port ${PORT}`);
      console.log(`✅ Health: /api/health`);
      console.log(`✅ Rooms:  /api/rooms/*`);
    });
  } catch (err) {
    console.error("❌ Failed to start server:", err?.message || err);
    process.exit(1);
  }
}

start();
