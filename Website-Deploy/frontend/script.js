document.addEventListener("DOMContentLoaded", () => {
  // ======= DOM REFERENCES =======

  // Screens
  const joinScreen = document.getElementById("joinScreen");
  const consoleScreen = document.getElementById("consoleScreen");

  // Join screen elements
  const joinSlots = Array.from(joinScreen.querySelectorAll(".code-slot"));
  const suitButtons = Array.from(joinScreen.querySelectorAll(".suit-btn"));
  const joinBtn = document.getElementById("joinBtn");
  const resetBtn = document.getElementById("resetBtn");

  // Console screen elements
  const roomCodeSlots = Array.from(
    consoleScreen.querySelectorAll(".room-code-view .code-slot")
  );
  const logoutBtn = document.getElementById("logoutBtn");
  const startBtn = document.getElementById("startBtn");
  const pauseBtn = document.getElementById("pauseBtn");
  const stopBtn = document.getElementById("stopBtn");

  const cardGainInput = document.getElementById("cardGain");
  const numPlayersInput = document.getElementById("numPlayers");
  const startingCardsInput = document.getElementById("startingCards");
  const playerList = document.getElementById("playerList");

  // ======= STATE =======
  let currentIndex = 0; // next slot index to fill (0..3)
  let locked = false;   // lock code input after 4 symbols

  const API_BASE = "http://localhost:3222/api";

  // ======= JOIN SCREEN LOGIC =======

  function allSlotsFilled() {
    return joinSlots.every(slot => !slot.classList.contains("empty"));
  }

  function updateJoinButton() {
    joinBtn.disabled = !allSlotsFilled();
  }

  function resetJoinScreen() {
    joinSlots.forEach(slot => {
      slot.textContent = "?";
      slot.classList.add("empty");
    });
    currentIndex = 0;
    locked = false;
    updateJoinButton();
  }

  suitButtons.forEach(btn => {
    btn.addEventListener("click", () => {
      if (locked) return;

      const symbol = btn.textContent.trim();

      joinSlots[currentIndex].textContent = symbol;
      joinSlots[currentIndex].classList.remove("empty");

      // Move to next slot, but do NOT loop back
      if (currentIndex < joinSlots.length - 1) {
        currentIndex++;
      }

      updateJoinButton();

      if (allSlotsFilled()) {
        locked = true; // don’t allow further changes
      }
    });
  });

  // Allow clicking on slots to change the "active" index (only if not locked)
  joinSlots.forEach((slot, idx) => {
    slot.addEventListener("click", () => {
      if (locked) return;
      currentIndex = idx;
    });
  });

  resetBtn.addEventListener("click", () => {
    resetJoinScreen();
  });

  // ======= CONSOLE SCREEN HELPERS =======

  function showConsoleScreen(roomCode) {
    // Fill console room code slots
    roomCodeSlots.forEach((slot, i) => {
      slot.textContent = roomCode[i] || "?";
    });

    joinScreen.classList.add("hidden");
    consoleScreen.classList.remove("hidden");
  }

  function showJoinScreen() {
    consoleScreen.classList.add("hidden");
    joinScreen.classList.remove("hidden");
    resetJoinScreen();

    // Reset buttons to "pending" default
    startBtn.disabled = false;
    pauseBtn.disabled = true;
    stopBtn.disabled = true;
    startBtn.textContent = "Start";
    setFieldsEditable(true);
  }

  // Toggle editability of settings + player list
  function setFieldsEditable(isEditable) {
    // Settings inputs
    [cardGainInput, numPlayersInput, startingCardsInput].forEach(input => {
      input.disabled = !isEditable;
    });

    // Player name inputs
    const nameInputs = playerList.querySelectorAll(".player-name");
    nameInputs.forEach(inp => {
      inp.disabled = !isEditable;
    });

    // Player control buttons (up/down/skip)
    const controlButtons = playerList.querySelectorAll(".player-controls button");
    controlButtons.forEach(btn => {
      btn.disabled = !isEditable;
    });
  }

  // Room status → button states + editable state + Start/Resume label
  function applyRoomStatus(room) {
    const status = room.room_status; // "pending" | "running" | "paused" | "finished"
    console.log("Room status:", status);

    if (status === "pending") {
      startBtn.disabled = false;
      pauseBtn.disabled = true;
      stopBtn.disabled = true;
      startBtn.textContent = "Start";
      setFieldsEditable(true);  // allow editing before first start
    } else if (status === "running") {
      startBtn.disabled = true;   // already running → block new START
      pauseBtn.disabled = false;
      stopBtn.disabled = false;
      startBtn.textContent = "Start";
      setFieldsEditable(false);  // 🔒 no editing while running
    } else if (status === "paused") {
      startBtn.disabled = false;  // allow resume
      pauseBtn.disabled = false;
      stopBtn.disabled = false;
      startBtn.textContent = "Resume"; // 🔁 show Resume when paused
      setFieldsEditable(true);   // ✅ editable ONLY in paused (and pending)
    } else if (status === "finished") {
      startBtn.disabled = true;
      pauseBtn.disabled = true;
      stopBtn.disabled = true;
      startBtn.textContent = "Start";
      setFieldsEditable(false);  // 🔒 no editing after finished
    }
  }

  // Build player list rows, using optional playersData from DB
  function buildPlayerRows(playersData) {
    playerList.innerHTML = "";

    let rowsData;

    if (Array.isArray(playersData) && playersData.length > 0) {
      rowsData = playersData;
      numPlayersInput.value = playersData.length;
    } else {
      let count = parseInt(numPlayersInput.value, 10);
      if (isNaN(count) || count < 1) count = 1;
      if (count > 8) count = 8;
      numPlayersInput.value = count;

      rowsData = Array.from({ length: count }, (_, i) => ({
        name: `Player ${i + 1}`,
        skipped: false
      }));
    }

    rowsData.forEach((p, idx) => {
      const row = document.createElement("div");
      row.className = "player-row";
      row.dataset.index = String(idx);

      if (p.skipped) {
        row.classList.add("skipped");
      }

      row.innerHTML = `
        <span class="player-order">${idx + 1}</span>
        <input
          type="text"
          class="player-name"
          placeholder="Player ${idx + 1}"
        />
        <div class="player-controls">
          <button type="button" class="player-up" aria-label="Move up">↑</button>
          <button type="button" class="player-down" aria-label="Move down">↓</button>
          <button type="button" class="player-skip" aria-label="Skip player">
            ${p.skipped ? "Skipped" : "Skip"}
          </button>
        </div>
      `;

      const nameInput = row.querySelector(".player-name");
      nameInput.value = p.name || `Player ${idx + 1}`;

      const upBtn = row.querySelector(".player-up");
      const downBtn = row.querySelector(".player-down");
      const skipBtn = row.querySelector(".player-skip");

      upBtn.addEventListener("click", () => movePlayerRow(row, -1));
      downBtn.addEventListener("click", () => movePlayerRow(row, 1));
      skipBtn.addEventListener("click", () => toggleSkip(row, skipBtn));

      playerList.appendChild(row);
    });
  }

  function refreshPlayerOrder() {
    const rows = Array.from(playerList.querySelectorAll(".player-row"));
    rows.forEach((row, index) => {
      const orderEl = row.querySelector(".player-order");
      if (orderEl) {
        orderEl.textContent = index + 1;
      }
    });
  }

  function movePlayerRow(row, delta) {
    const rows = Array.from(playerList.querySelectorAll(".player-row"));
    const index = rows.indexOf(row);
    const newIndex = index + delta;

    if (newIndex < 0 || newIndex >= rows.length) return;

    playerList.removeChild(row);
    if (delta > 0) {
      playerList.insertBefore(row, rows[newIndex].nextSibling);
    } else {
      playerList.insertBefore(row, rows[newIndex]);
    }

    refreshPlayerOrder();
  }

  function toggleSkip(row, button) {
    const isSkipped = row.classList.toggle("skipped");
    button.textContent = isSkipped ? "Skipped" : "Skip";
  }

  function resetPlayerStates() {
    const rows = Array.from(playerList.querySelectorAll(".player-row"));
    rows.forEach(row => {
      row.classList.remove("skipped");
      const skipBtn = row.querySelector(".player-skip");
      if (skipBtn) {
        skipBtn.textContent = "Skip";
      }
    });
  }

  // When numPlayers changes, rebuild rows (if console is visible)
  numPlayersInput.addEventListener("change", () => {
    if (!consoleScreen.classList.contains("hidden")) {
      buildPlayerRows();
    }
  });

  // Fill console settings + players from a room document
  function hydrateConsoleFromRoom(room) {
    if (room.card_gain != null) {
      cardGainInput.value = room.card_gain;
    }
    if (room.starting_card != null) {
      startingCardsInput.value = room.starting_card;
    }

    let numPlayers = room.num_players;
    if (!numPlayers && Array.isArray(room.players)) {
      numPlayers = room.players.length;
    }
    if (numPlayers) {
      numPlayersInput.value = numPlayers;
    }

    if (Array.isArray(room.players) && room.players.length > 0) {
      buildPlayerRows(room.players);
    } else {
      buildPlayerRows();
    }
  }

  // ======= GAME STATE & API HELPERS =======

  function getGameState() {
    const roomCode = roomCodeSlots.map(slot => slot.textContent.trim()).join("");
    const cardGain = parseInt(cardGainInput.value, 10) || 0;
    const numPlayers = parseInt(numPlayersInput.value, 10) || 0;
    const startingCards = parseInt(startingCardsInput.value, 10) || 0;

    const players = Array.from(
      playerList.querySelectorAll(".player-row")
    ).map(row => ({
      name: row.querySelector(".player-name").value.trim(),
      skipped: row.classList.contains("skipped")
    }));

    return {
      roomCode,
      cardGain,
      numPlayers,
      startingCards,
      players
    };
  }

  async function joinRoomOnServer(roomCode) {
    try {
      const res = await fetch(`${API_BASE}/rooms/join`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ roomCode }),
      });

      if (!res.ok) {
        const errBody = await res.json().catch(() => ({}));
        console.error("Join error:", res.status, errBody);
        if (res.status === 404) {
          alert("Room code not found. Please check your code.");
        } else {
          alert(errBody.error || "Failed to join room.");
        }
        return null;
      }

      const data = await res.json();
      console.log("Joined room:", data);
      return data.room;
    } catch (err) {
      console.error("Join network error:", err);
      alert("Cannot reach server. Please try again.");
      return null;
    }
  }

  async function sendRoomUpdate(action) {
    const state = getGameState();
    const roomCode = state.roomCode;

    if (!roomCode || roomCode.includes("?")) {
      alert("Invalid room code.");
      return null;
    }

    let url = `${API_BASE}/rooms/${action}`;
    let body = { roomCode };

    if (action === "start") {
      body = {
        roomCode,
        cardGain: state.cardGain,
        numPlayers: state.numPlayers,
        startingCards: state.startingCards,
        players: state.players,
      };
    }

    try {
      const res = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });

      if (!res.ok) {
        const errBody = await res.json().catch(() => ({}));
        console.error(`${action.toUpperCase()} error:`, res.status, errBody);
        alert(errBody.error || `Failed to ${action} room.`);
        return null;
      }

      const data = await res.json();
      console.log(`${action.toUpperCase()} response:`, data);
      return data.room;
    } catch (err) {
      console.error(`${action.toUpperCase()} network error:`, err);
      alert("Cannot reach server. Please try again.");
      return null;
    }
  }

  // ======= BUTTON HANDLERS =======

  // Join: validate code with backend, then show console + hydrate from DB
  joinBtn.addEventListener("click", async () => {
    if (!allSlotsFilled()) return;

    const finalCode = joinSlots.map(slot => slot.textContent.trim()).join("");

    const room = await joinRoomOnServer(finalCode);
    if (!room) return; // invalid or finished

    showConsoleScreen(finalCode);
    hydrateConsoleFromRoom(room);  // restore config + players from MongoDB
    applyRoomStatus(room);         // set buttons + editable state
  });

  // Logout: go back to join screen, clear players
  logoutBtn.addEventListener("click", () => {
    playerList.innerHTML = "";
    showJoinScreen();
  });

  startBtn.addEventListener("click", async () => {
    const room = await sendRoomUpdate("start");
    if (room) {
      // If status was pending → running (Start)
      // If status was paused  → running (Resume)
      const fromStatus = room.room_status;
      alert("Game STARTED for room " + room.room_code);
      applyRoomStatus(room);
    }
  });

  pauseBtn.addEventListener("click", async () => {
    const room = await sendRoomUpdate("pause");
    if (room) {
      alert("Room " + room.room_code + " is now PAUSED.");
      applyRoomStatus(room);
    }
  });

  stopBtn.addEventListener("click", async () => {
    const room = await sendRoomUpdate("stop");
    if (room) {
      resetPlayerStates();
      alert("Room " + room.room_code + " is now FINISHED. Players reset locally.");
      applyRoomStatus(room);
    }
  });

  // ======= INIT =======

  updateJoinButton();
  // consoleScreen is hidden initially in HTML; player rows will be built after join
});
