const joints = ["J1", "J2", "J3", "J4", "J5", "J6"];
const defaultPercents = [80, 80, 80, 80, 80, 80];

const els = {
  mode: document.querySelector("#modeBadge"),
  modeCard: document.querySelector("#modeCard"),
  modeTitle: document.querySelector("#modeTitle"),
  modeText: document.querySelector("#modeText"),
  runId: document.querySelector("#runId"),
  dryRun: document.querySelector("#dryRun"),
  startBtn: document.querySelector("#startBtn"),
  applyBtn: document.querySelector("#applyBtn"),
  stopBtn: document.querySelector("#stopBtn"),
  homeBtn: document.querySelector("#homeBtn"),
  estopBtn: document.querySelector("#estopBtn"),
  reset80Btn: document.querySelector("#reset80Btn"),
  message: document.querySelector("#message"),
  logPath: document.querySelector("#logPath"),
  poseSummary: document.querySelector("#poseSummary"),
  jointControls: document.querySelector("#jointControls"),
  cameraButtons: document.querySelectorAll("[data-camera-profile]"),
  cameraProfileText: document.querySelector("#cameraProfileText"),
  trajectoryCards: document.querySelector("#trajectoryCards"),
  addTrajectoryBtn: document.querySelector("#addTrajectoryBtn"),
  body: document.querySelector("#telemetryBody"),
  events: document.querySelector("#events"),
};

let lastStatus = {};
let trajectorySlotCount = 3;
let currentCameraProfile = "light";
const sliders = [];
const outputs = [];

function fmt(v, digits = 2) {
  const n = Number(v || 0);
  return n.toFixed(digits);
}

function labelPose(q = []) {
  return joints.map((joint, i) => `${joint} ${fmt(q[i], 1)}°`).join("  ");
}

function jointPercent() {
  return sliders.map((slider) => Number(slider.value));
}

function syncJointLabels() {
  sliders.forEach((slider, i) => {
    outputs[i].textContent = `${slider.value}%`;
  });
}

function setSliders(values) {
  values.forEach((value, i) => {
    sliders[i].value = Math.round(Number(value));
  });
  syncJointLabels();
}

function settings() {
  return { joint_percent: jointPercent(), camera_profile: currentCameraProfile };
}

async function post(path, data = {}) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data),
  });
  const json = await res.json();
  if (!res.ok) throw new Error(json.error || res.statusText);
  return json;
}

function buildJointControls() {
  els.jointControls.innerHTML = joints.map((joint, i) => `
    <div class="joint-row" data-joint="${joint}">
      <div class="joint-name">${joint}</div>
      <button class="step" data-joint-index="${i}" data-step="-5">-5</button>
      <button class="step" data-joint-index="${i}" data-step="-1">-1</button>
      <input id="joint${i}" type="range" min="0" max="200" value="${defaultPercents[i]}" step="1" />
      <button class="step" data-joint-index="${i}" data-step="1">+1</button>
      <button class="step" data-joint-index="${i}" data-step="5">+5</button>
      <output>${defaultPercents[i]}%</output>
    </div>
  `).join("");
  els.jointControls.querySelectorAll("input").forEach((slider) => {
    sliders.push(slider);
    outputs.push(slider.parentElement.querySelector("output"));
    slider.addEventListener("input", syncJointLabels);
  });
}

function renderTrajectories(trajectories = [], status = {}) {
  const profiles = status.camera_profiles || {};
  const bySlot = new Map(trajectories.map((trajectory) => [Number(trajectory.slot), trajectory]));
  const recordingSlot = Number(status.recording_slot || 0);
  const maxSavedSlot = Math.max(0, ...trajectories.map((trajectory) => Number(trajectory.slot || 0)));
  trajectorySlotCount = Math.max(3, trajectorySlotCount, maxSavedSlot, recordingSlot);
  const isReplaying = status.mode === "replaying" || status.mode === "replay_starting";
  els.addTrajectoryBtn.disabled = trajectorySlotCount >= 20;
  els.trajectoryCards.innerHTML = Array.from({ length: trajectorySlotCount }, (_, index) => index + 1).map((slot) => {
    const trajectory = bySlot.get(slot);
    const isRecording = recordingSlot === slot;
    const title = trajectory?.label || `Pose ${slot}`;
    const duration = trajectory ? `${fmt(trajectory.duration_s, 1)}s / ${trajectory.sample_count || 0} samples` : "no trajectory";
    const cameraProfile = trajectory?.camera_profile || "";
    const cameraLabel = cameraProfile && profiles[cameraProfile] ? profiles[cameraProfile].label : cameraProfile;
    const start = trajectory?.start_q_deg ? labelPose(trajectory.start_q_deg) : "start --";
    const end = trajectory?.end_q_deg ? labelPose(trajectory.end_q_deg) : "end --";
    const recordDisabled = (recordingSlot && !isRecording) || isReplaying;
    const replayDisabled = !trajectory || recordingSlot || isReplaying;
    const resetDisabled = !trajectory && !isRecording;
    return `
      <article class="trajectory-card ${isRecording ? "recording" : ""}">
        <header>
          <strong>${isRecording ? `Recording Pose ${slot}` : title}</strong>
          <span>${trajectory ? new Date(trajectory.saved_ms).toLocaleTimeString("zh-CN", { hour12: false }) : "--"}</span>
        </header>
        <p>${duration}${cameraLabel ? ` · ${cameraLabel}` : ""}</p>
        <small>${start}<br />${end}</small>
        <div class="trajectory-actions">
          <button data-record-slot="${slot}" ${recordDisabled || isRecording ? "disabled" : ""}>Record</button>
          <button data-stop-record-slot="${slot}" ${isRecording ? "" : "disabled"}>Stop Rec</button>
          <button data-replay-slot="${slot}" data-speed="0.25" ${replayDisabled ? "disabled" : ""}>Replay</button>
          <button data-reset-slot="${slot}" ${resetDisabled ? "disabled" : ""}>Reset</button>
        </div>
      </article>
    `;
  }).join("");
}

function renderStatus(status) {
  lastStatus = status || {};
  currentCameraProfile = status.camera_profile || currentCameraProfile || "light";
  const profiles = status.camera_profiles || {};
  const profile = profiles[currentCameraProfile] || {};
  const isLive = !status.dry_run;
  document.body.classList.toggle("live-mode", isLive);
  els.modeCard.classList.toggle("live", isLive);
  els.modeCard.classList.toggle("dry", !isLive);
  els.modeTitle.textContent = isLive ? "LIVE" : "DRY-RUN";
  els.modeText.textContent = isLive ? "J1-J6 real motor control" : "simulation console";
  els.startBtn.textContent = isLive ? "Start live tuning" : "Start dry-run";
  els.mode.textContent = status.mode || "idle";
  els.runId.textContent = status.run_id || "no run";
  els.dryRun.textContent = status.dry_run ? "dry-run" : "live";
  els.message.textContent = status.message || "";
  els.logPath.textContent = status.log_path || "no log";
  els.poseSummary.textContent = labelPose(status.q_deg || []);
  els.cameraButtons.forEach((button) => {
    const active = button.dataset.cameraProfile === currentCameraProfile;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", active ? "true" : "false");
  });
  els.cameraProfileText.textContent = `${profile.label || currentCameraProfile} selected · ${profile.note || ""}`;
  if (Array.isArray(status.joint_percent) && document.activeElement?.tagName !== "INPUT") {
    setSliders(status.joint_percent);
  }
  renderTrajectories(status.trajectories || [], status);
  const rows = joints.map((joint, i) => `
    <tr>
      <td>${joint}</td>
      <td>${fmt(status.q_deg?.[i], 1)}</td>
      <td>${fmt((status.effective_scale?.[i] || 0) * 100, 0)}%</td>
      <td>${fmt(status.tau_model_nm?.[i])}</td>
      <td>${fmt(status.tau_cmd_nm?.[i])}</td>
    </tr>
  `);
  els.body.innerHTML = rows.join("");
}

function addEvent(event) {
  const div = document.createElement("div");
  div.className = `event ${event.level || ""}`;
  div.textContent = `[${event.ts || "--:--:--"}] ${event.message || ""}`;
  els.events.prepend(div);
  while (els.events.children.length > 140) {
    els.events.lastChild.remove();
  }
}

function logClient(message, level = "info") {
  addEvent({
    ts: new Date().toLocaleTimeString("zh-CN", { hour12: false }),
    level,
    message,
  });
}

buildJointControls();
syncJointLabels();

els.jointControls.addEventListener("click", (event) => {
  const button = event.target.closest("button.step");
  if (!button) return;
  const i = Number(button.dataset.jointIndex);
  const step = Number(button.dataset.step);
  sliders[i].value = Math.min(200, Math.max(0, Number(sliders[i].value) + step));
  syncJointLabels();
});

els.reset80Btn.addEventListener("click", () => setSliders(defaultPercents));

els.cameraButtons.forEach((button) => {
  button.addEventListener("click", async () => {
    currentCameraProfile = button.dataset.cameraProfile || "light";
    const s = settings();
    logClient(`select ${currentCameraProfile} camera`, "warn");
    try { renderStatus(await post("/api/apply", s)); } catch (e) { addEvent({ level: "danger", message: e.message }); }
  });
});

els.startBtn.addEventListener("click", async () => {
  const s = settings();
  logClient(`POST /api/start ${s.camera_profile} ${JSON.stringify(s.joint_percent)}`);
  try { renderStatus(await post("/api/start", s)); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

els.applyBtn.addEventListener("click", async () => {
  const s = settings();
  logClient(`POST /api/apply ${s.camera_profile} ${JSON.stringify(s.joint_percent)}`);
  try { renderStatus(await post("/api/apply", s)); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

els.trajectoryCards.addEventListener("click", async (event) => {
  const record = event.target.closest("button[data-record-slot]");
  const stopRecord = event.target.closest("button[data-stop-record-slot]");
  const replay = event.target.closest("button[data-replay-slot]");
  const reset = event.target.closest("button[data-reset-slot]");
  try {
    if (record) {
      const slot = Number(record.dataset.recordSlot);
      logClient(`start recording Pose ${slot}`, "warn");
      renderStatus(await post("/api/start-recording", { slot, label: `Pose ${slot}`, ...settings() }));
    }
    if (stopRecord) {
      const slot = Number(stopRecord.dataset.stopRecordSlot);
      logClient(`stop recording Pose ${slot}`);
      renderStatus(await post("/api/stop-recording", { slot }));
    }
    if (replay) {
      const slot = Number(replay.dataset.replaySlot);
      const speed = Number(replay.dataset.speed);
      if (!confirm(`Replay Pose ${slot} at ${Math.round(speed * 100)}% speed? Keep your hand near the arm.`)) return;
      logClient(`replay Pose ${slot} at ${Math.round(speed * 100)}%`, "warn");
      renderStatus(await post("/api/replay-trajectory", { slot, speed, ...settings() }));
    }
    if (reset) {
      const slot = Number(reset.dataset.resetSlot);
      if (!confirm(`Reset Pose ${slot} trajectory?`)) return;
      logClient(`reset Pose ${slot}`, "warn");
      renderStatus(await post("/api/reset-trajectory", { slot }));
    }
  } catch (e) {
    addEvent({ level: "danger", message: e.message });
  }
});

els.addTrajectoryBtn.addEventListener("click", () => {
  if (trajectorySlotCount >= 20) return;
  trajectorySlotCount += 1;
  renderTrajectories(lastStatus.trajectories || [], lastStatus);
  logClient(`added Pose ${trajectorySlotCount} slot`);
});

els.stopBtn.addEventListener("click", async () => {
  logClient("POST /api/stop");
  try { renderStatus(await post("/api/stop")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

els.homeBtn.addEventListener("click", async () => {
  if (!confirm("Return J1-J6 to Home and disable? Keep supporting the payload.")) return;
  logClient("POST /api/home-disable", "warn");
  try { renderStatus(await post("/api/home-disable")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

els.estopBtn.addEventListener("click", async () => {
  if (!confirm("Emergency disable may let the loaded arm drop. Continue?")) return;
  logClient("POST /api/emergency-disable", "danger");
  try { renderStatus(await post("/api/emergency-disable")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

fetch("/api/status", { cache: "no-store" }).then((r) => r.json()).then((status) => {
  renderStatus(status);
  logClient(`connected to ${status.dry_run ? "dry-run" : "LIVE"} service`);
});

const source = new EventSource("/api/events");
source.onopen = () => logClient("event stream connected");
source.onerror = () => logClient("event stream disconnected; polling still active", "warn");
source.onmessage = (message) => {
  const payload = JSON.parse(message.data);
  if (payload.type === "status") renderStatus(payload.status);
  if (payload.type === "event") addEvent(payload.event);
};

setInterval(async () => {
  try {
    renderStatus(await fetch("/api/status", { cache: "no-store" }).then((r) => r.json()));
  } catch (e) {
    addEvent({ level: "danger", message: `poll failed: ${e.message}` });
  }
}, 2000);
