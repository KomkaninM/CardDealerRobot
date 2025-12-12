// src/routes/room.routes.js
import { Router } from "express";
import {
  joinRoom,
  startRoom,
  pauseRoom,
  stopRoom,
} from "../controllers/room.controller.js";

const router = Router();

router.post("/join", joinRoom);
router.post("/start", startRoom);
router.post("/pause", pauseRoom);
router.post("/stop", stopRoom);

export default router;
