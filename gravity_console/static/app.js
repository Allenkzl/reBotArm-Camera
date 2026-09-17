const joints = ["J1", "J2", "J3", "J4", "J5", "J6"];

const els = {
  base: document.querySelector("#basePercent"),
  percent: document.querySelector("#percentReadout"),
  trimJ2: document.querySelector("#trimJ2"),
  trimJ3: document.querySelector("#trimJ3"),
  trimJ4: document.querySelector("#trimJ4"),
  trimJ2Out: document.querySelector("#trimJ2Out"),
  trimJ3Out: document.querySelector("#trimJ3Out"),
  trimJ4Out: document.querySelector("#trimJ4Out"),
  mode: document.querySelector("#modeBadge"),
  modeCard: document.querySelector("#modeCard"),
  modeTitle: document.querySelector("#modeTitle"),
  modeText: document.querySelector("#modeText"),
  runId: document.querySelector("#runId"),
  dryRun: document.querySelector("#dryRun"),
  startBtn: document.querySelector("#startBtn"),
  message: document.querySelector("#message"),
  logPath: document.querySelector("#logPath"),
  body: document.querySelector("#telemetryBody"),
  events: document.querySelector("#events"),
};

function settings() {
  return {
    base_percent: Number(els.base.value),
    trims: {
      j2: Number(els.trimJ2.value),
      j3: Number(els.trimJ3.value),
      j4: Number(els.trimJ4.value),
    },
  };
}

function syncLabels() {
  els.percent.textContent = `${els.base.value}%`;
  els.trimJ2Out.textContent = `${els.trimJ2.value}%`;
  els.trimJ3Out.textContent = `${els.trimJ3.value}%`;
  els.trimJ4Out.textContent = `${els.trimJ4.value}%`;
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

function fmt(v, digits = 2) {
  const n = Number(v || 0);
  return n.toFixed(digits);
}

function renderStatus(status) {
  const isLive = !status.dry_run;
  document.body.classList.toggle("live-mode", isLive);
  els.modeCard.classList.toggle("live", isLive);
  els.modeCard.classList.toggle("dry", !isLive);
  els.modeTitle.textContent = isLive ? "LIVE" : "DRY-RUN";
  els.modeText.textContent = isLive ? "J1-J6 real motor control" : "simulation console";
  els.startBtn.textContent = isLive ? "Start live test" : "Start dry-run";
  els.mode.textContent = status.mode || "idle";
  els.runId.textContent = status.run_id || "no run";
  els.dryRun.textContent = status.dry_run ? "dry-run" : "live";
  els.message.textContent = status.message || "";
  els.logPath.textContent = status.log_path || "no log";
  if (status.base_percent !== undefined) {
    els.base.value = Math.round(status.target_percent ?? status.base_percent);
    els.trimJ2.value = Math.round(status.trims?.j2 ?? 0);
    els.trimJ3.value = Math.round(status.trims?.j3 ?? 0);
    els.trimJ4.value = Math.round(status.trims?.j4 ?? 0);
    syncLabels();
  }
  const rows = joints.map((joint, i) => `
    <tr>
      <td>${joint}</td>
      <td>${fmt(status.q_deg?.[i], 1)}</td>
      <td>${fmt(status.tau_model_nm?.[i])}</td>
      <td>${fmt(status.tau_payload_nm?.[i])}</td>
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
  while (els.events.children.length > 120) {
    els.events.lastChild.remove();
  }
}

document.querySelector("#quickButtons").addEventListener("click", (event) => {
  const button = event.target.closest("button[data-percent]");
  if (!button) return;
  els.base.value = button.dataset.percent;
  syncLabels();
});

["input", "change"].forEach((name) => {
  [els.base, els.trimJ2, els.trimJ3, els.trimJ4].forEach((el) => el.addEventListener(name, syncLabels));
});

document.querySelector("#startBtn").addEventListener("click", async () => {
  try { renderStatus(await post("/api/start", settings())); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});
document.querySelector("#applyBtn").addEventListener("click", async () => {
  try { renderStatus(await post("/api/apply", settings())); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});
document.querySelector("#stopBtn").addEventListener("click", async () => {
  try { renderStatus(await post("/api/stop")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});
document.querySelector("#homeBtn").addEventListener("click", async () => {
  if (!confirm("Return J1-J6 to Home and disable? Keep supporting the payload.")) return;
  try { renderStatus(await post("/api/home-disable")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});
document.querySelector("#estopBtn").addEventListener("click", async () => {
  if (!confirm("Emergency disable may let the loaded arm drop. Continue?")) return;
  try { renderStatus(await post("/api/emergency-disable")); } catch (e) { addEvent({ level: "danger", message: e.message }); }
});

syncLabels();
fetch("/api/status").then((r) => r.json()).then(renderStatus);

const source = new EventSource("/api/events");
source.onmessage = (message) => {
  const payload = JSON.parse(message.data);
  if (payload.type === "status") renderStatus(payload.status);
  if (payload.type === "event") addEvent(payload.event);
};
