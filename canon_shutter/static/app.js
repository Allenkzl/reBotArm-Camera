"use strict";
const $ = (id) => document.getElementById(id);
let latest = null;
let polling = false;
let submitting = false;
let lastResultId = "";
let lastCandidates = "";
const labels = {disconnected:"未连接", scanning:"搜索中", connecting:"连接中", authenticating:"验证配对中", identifying:"注册遥控器中", connected:"已连接", shooting:"发送快门中", error:"连接失败"};

function showError(message) {
  $("alert").textContent = message;
  $("alert").hidden = !message;
}

function render(data) {
  latest = data;
  const online = data.device_online;
  const device = data.device || {};
  const working = Boolean(submitting || data.pending || device.busy);
  const connected = online && device.connected;
  $("usbBadge").className = "badge" + (online ? " online" : "");
  $("usbBadge").querySelector("span").textContent = online ? "USB 设备已就绪" : "USB 设备离线";
  $("stateBadge").textContent = online ? (labels[device.state] || "等待状态") : "设备离线";
  $("stateBadge").className = "state " + (connected ? "connected" : device.state === "error" ? "error" : working ? "working" : "");
  $("cameraName").textContent = connected ? (device.camera_name || "Canon EOS") : device.saved_name && device.saved_address ? device.saved_name : "等待你的相机";
  $("statusMessage").textContent = online ? device.message : (data.error || "连接 XIAO 的 USB 数据线后，设备状态会自动显示。");
  $("remoteName").textContent = device.remote_name || "XIAO-Shutter";
  $("savedCamera").textContent = device.saved_address || "—";
  $("firmware").textContent = online ? device.firmware : "等待固件";
  $("shotCount").textContent = String(device.shots_sent || 0).padStart(2, "0");
  $("connect").disabled = !online || working || connected;
  $("connect").replaceChildren(document.createTextNode(device.saved_address ? "连接相机" : "配对新相机"));
  $("disconnect").disabled = !online || working || !connected;
  $("forget").disabled = !online || working;
  $("scan").disabled = !online || working || connected;
  $("shutter").disabled = !connected || working;
  $("shutterHint").textContent = working ? "操作进行中，请稍候…" : connected ? "点击一次，发送一次快门指令" : "连接相机后即可拍照";
  const cameras = device.candidates || [];
  const key = JSON.stringify(cameras.map(c => [c.address, c.name, c.rssi]));
  if (key !== lastCandidates) {
    const previous = $("cameraSelect").value;
    $("cameraSelect").replaceChildren(...cameras.map(camera => {
      const option = document.createElement("option");
      option.value = camera.address;
      option.textContent = `${camera.name} · ${camera.address} · ${camera.rssi} dBm`;
      return option;
    }));
    if (cameras.some(c => c.address === previous)) $("cameraSelect").value = previous;
    lastCandidates = key;
  }
  $("candidates").hidden = !online || !cameras.length || connected;
  $("connectSelected").disabled = !online || working || !cameras.length || connected;
  $("cameraSelect").disabled = working;
  $("events").replaceChildren(...(data.events || []).slice().reverse().map(event => {
    const li = document.createElement("li");
    const timestamp = document.createElement("time");
    timestamp.textContent = event.time;
    const text = document.createElement("span");
    text.textContent = event.message;
    li.append(timestamp, text);
    return li;
  }));
  if (data.last_result && data.last_result.id !== lastResultId) {
    lastResultId = data.last_result.id;
    showError(data.last_result.ok ? "" : data.last_result.message);
  }
}

async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("/api/status", {cache:"no-store", signal:AbortSignal.timeout(2500)});
    if (!response.ok) throw new Error("无法读取设备状态");
    render(await response.json());
  } catch (error) {
    render({device_online:false, device:{}, error:"本机服务连接中断，请检查服务是否仍在运行。", events:latest?.events || []});
  } finally {
    polling = false;
  }
}

async function command(action, address = "") {
  if (submitting) return;
  submitting = true;
  showError("");
  if (latest) render(latest);
  try {
    const response = await fetch("/api/command", {
      method:"POST", headers:{"Content-Type":"application/json", "X-Shutter-Client":"1"},
      body:JSON.stringify({action, address}), signal:AbortSignal.timeout(5000)
    });
    const answer = await response.json();
    if (!response.ok) throw new Error(answer.error || "操作提交失败");
    // Keep controls locked until a snapshot confirms the queued command or its result.
    if (latest) latest.pending = answer.id;
  } catch (error) {
    showError(`${error.message}。操作不会自动重试。`);
  } finally {
    submitting = false;
    if (latest) render(latest);
    poll();
  }
}

$("connect").addEventListener("click", () => command(latest?.device?.saved_address ? "connect" : "pair"));
$("disconnect").addEventListener("click", () => command("disconnect"));
$("forget").addEventListener("click", () => command("forget"));
$("scan").addEventListener("click", () => command("scan"));
$("connectSelected").addEventListener("click", () => command("connect", $("cameraSelect").value));
$("shutter").addEventListener("click", () => command("shutter"));
poll();
setInterval(poll, 750);
