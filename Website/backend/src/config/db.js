// src/config/db.js
import mongoose from "mongoose";

export async function connectDB() {
  const uri = process.env.MONGO_URI;
  const dbName = process.env.DB_NAME || "card_dealer_db";

  if (!uri) {
    throw new Error("MONGO_URI is not defined in .env");
  }

  try {
    await mongoose.connect(uri, { dbName });
    console.log(`✅ Connected to MongoDB db=${dbName}`);
  } catch (err) {
    console.error("❌ MongoDB connection error:", err.message);
    process.exit(1);
  }
}
