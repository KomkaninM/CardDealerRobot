// src/services/netpie.service.js
import mqtt from "mqtt";
import dotenv from "dotenv";

dotenv.config();

const appid = process.env.NETPIE_APP_ID;
const key = process.env.NETPIE_KEY;
const secret = process.env.NETPIE_SECRET;
const host = process.env.NETPIE_HOST || "mqtt.netpie.io";
const port = parseInt(process.env.NETPIE_PORT || "1883", 10);

if (!appid || !key || !secret) {
  console.error("[NETPIE] Missing NETPIE env vars. Check .env");
}

// สร้าง client ตัวเดียวใช้ทั้ง backend
const client = mqtt.connect({
  host,
  port,
  clientId: appid,
  username: key,
  password: secret,
});

client.on("connect", () => {
  console.log("[NETPIE] Backend connected");
});

client.on("error", (err) => {
  console.error("[NETPIE] MQTT error:", err.message);
});

/**
 * สร้าง payload room_config ให้เหมือนที่ Python ส่งออก
 * {
 *   data: {
 *     room_config: { ... }
 *   }
 * }
 */
export function buildRoomConfigPayload(room) {
  return {
    data: {
      room_config: {
        room_code: room.room_code,
        room_status: room.room_status,
        players: room.players || [],
        timestamp: new Date().toLocaleTimeString("en-GB", {
          hour12: false,
          hour: "2-digit",
          minute: "2-digit",
          second: "2-digit",
        }),
        num_players: room.num_players,
        card_gain: room.card_gain,
        starting_card: room.starting_card,
      },
    },
  };
}

/**
 * Publish room_config ไป NETPIE ที่
 * - @shadow/data/update
 * - @msg/room_config
 */
export function publishRoomConfig(room) {
  const payload = buildRoomConfigPayload(room);
  const jsonPayload = JSON.stringify(payload);

  const shadowTopic = "@shadow/data/update";
  const msgTopic = "@msg/room_config";

  client.publish(shadowTopic, jsonPayload, { qos: 0 }, (err) => {
    if (err) {
      console.error("[NETPIE] Publish error to shadow:", err.message);
    } else {
      console.log("[NETPIE] Published room_config to", shadowTopic);
    }
  });

  client.publish(msgTopic, jsonPayload, { qos: 0 }, (err) => {
    if (err) {
      console.error("[NETPIE] Publish error to msg:", err.message);
    } else {
      console.log("[NETPIE] Published room_config to", msgTopic);
    }
  });

  return payload;
}
