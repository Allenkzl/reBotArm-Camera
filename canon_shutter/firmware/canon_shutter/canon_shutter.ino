// XIAO ESP32S3 Sense / Arduino ESP32 3.3.8 / ArduinoJson 7.
// Canon protocol references and MIT notice: ../../THIRD_PARTY_NOTICES.md
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <Preferences.h>
#include <host/ble_gap.h>
#include <atomic>
#include <vector>

static constexpr char VERSION[] = "canon-shutter-0.1.1";
static constexpr char REMOTE_NAME[] = "XIAO-Shutter";
static BLEUUID SERVICE("00050000-0000-1000-0000-d8492fffa821");
static BLEUUID IDENTIFY("00050002-0000-1000-0000-d8492fffa821");
static BLEUUID CONTROL("00050003-0000-1000-0000-d8492fffa821");

struct Candidate { String address; String name; uint8_t addressType; int rssi; };
struct Command { char id[40]; char action[16]; char address[18]; };
static SemaphoreHandle_t stateLock, outputLock;
static QueueHandle_t commands;
static Preferences prefs;
static BLEClient *client;
static BLERemoteCharacteristic *control = nullptr;  // Only BLE worker accesses this.
static std::atomic<bool> linkUp(false), ready(false), busy(false);
static std::atomic<int> authResult(0);  // 0 pending, 1 authenticated, -1 failed
static std::atomic<int> authReason(0);
static std::atomic<uint32_t> stageDeadline(0);
static std::atomic<uint16_t> activeHandle(BLE_HS_CONN_HANDLE_NONE);
static String phase = "disconnected", message = "等待连接相机";
static String savedAddress, savedName, activeName;
static uint8_t savedType = BLE_ADDR_PUBLIC;
static uint32_t shotsSent = 0;
static String lastResultId, lastResultMessage;
static bool lastResultOk = false;
static std::vector<Candidate> candidates;

static void setState(const char *value, const String &detail) {
  uint32_t timeout = 0;
  if (!strcmp(value, "connecting")) timeout = 32000;
  if (!strcmp(value, "authenticating")) timeout = 20000;
  if (!strcmp(value, "identifying")) timeout = 15000;
  if (!strcmp(value, "shooting")) timeout = 5000;
  stageDeadline = timeout ? millis() + timeout : 0;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  phase = value;
  message = detail;
  xSemaphoreGive(stateLock);
}

static void output(JsonDocument &doc) {
  xSemaphoreTake(outputLock, portMAX_DELAY);
  serializeJson(doc, Serial);
  Serial.println();
  xSemaphoreGive(outputLock);
}

static void status() {
  JsonDocument doc;
  doc["type"] = "status";
  doc["firmware"] = VERSION;
  doc["remote_name"] = REMOTE_NAME;
  doc["uptime_ms"] = millis();
  doc["connected"] = ready.load() && linkUp.load();
  doc["busy"] = busy.load();
  xSemaphoreTake(stateLock, portMAX_DELAY);
  doc["state"] = phase;
  doc["message"] = message;
  doc["saved_address"] = savedAddress;
  doc["saved_name"] = savedName;
  doc["camera_name"] = activeName;
  doc["shots_sent"] = shotsSent;
  if (lastResultId.length()) {
    JsonObject completed = doc["last_result"].to<JsonObject>();
    completed["id"] = lastResultId;
    completed["ok"] = lastResultOk;
    completed["message"] = lastResultMessage;
  }
  JsonArray devices = doc["candidates"].to<JsonArray>();
  for (const auto &candidate : candidates) {
    JsonObject item = devices.add<JsonObject>();
    item["address"] = candidate.address;
    item["name"] = candidate.name;
    item["rssi"] = candidate.rssi;
  }
  xSemaphoreGive(stateLock);
  output(doc);
}

static void result(const char *id, bool ok, const String &detail) {
  if (id && id[0]) {
    xSemaphoreTake(stateLock, portMAX_DELAY);
    lastResultId = id;
    lastResultOk = ok;
    lastResultMessage = detail;
    xSemaphoreGive(stateLock);
  }
  JsonDocument doc;
  doc["type"] = "result";
  doc["id"] = id;
  doc["ok"] = ok;
  doc["message"] = detail;
  output(doc);
}

class ConnectionCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *peer) override {
    activeHandle = peer->getConnId();
    linkUp = true;
  }
  void onDisconnect(BLEClient *) override {
    linkUp = false;
    ready = false;
    activeHandle = BLE_HS_CONN_HANDLE_NONE;
  }
};

class SecurityCallbacks : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return 123456; }
  void onPassKeyNotify(uint32_t) override {}
  bool onConfirmPIN(uint32_t) override { return false; }  // BR-E1 uses no-IO bonding.
  bool onSecurityRequest() override { return busy.load(); }
  void onAuthenticationComplete(ble_gap_conn_desc *desc) override {
    bool encrypted = desc && desc->sec_state.encrypted;
    authReason = encrypted ? 0 : 1;
    authResult = encrypted ? 1 : -1;
  }
};

static bool disconnectCamera() {
  ready = false;
  control = nullptr;
  if (client->isConnected()) client->disconnect();
  uint32_t started = millis();
  while (client->isConnected() && millis() - started < 4500) delay(20);
  if (client->isConnected()) {
    setState("error", "蓝牙断开尚未完成，请稍后重试或重新插拔设备");
    return false;
  }
  linkUp = false;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  activeName = "";
  xSemaphoreGive(stateLock);
  return true;
}

static bool fail(const String &detail) {
  disconnectCamera();
  setState("error", detail);
  return false;
}

static bool scanCameras() {
  if (!disconnectCamera()) return false;
  setState("scanning", "搜索佳能相机中，请保持相机处于无线遥控器配对画面…");
  BLEScan *scan = BLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  BLEScanResults *found = scan->start(8, false);
  std::vector<Candidate> next;
  if (found) {
    for (uint32_t i = 0; i < found->getCount(); ++i) {
      BLEAdvertisedDevice device = found->getDevice(i);
      if (device.isAdvertisingService(SERVICE)) {
        next.push_back({device.getAddress().toString(), device.haveName() ? String(device.getName().c_str()) : "Canon EOS",
                        device.getAddressType(), device.getRSSI()});
      }
    }
  }
  scan->clearResults();
  xSemaphoreTake(stateLock, portMAX_DELAY);
  candidates = next;
  xSemaphoreGive(stateLock);
  if (next.empty()) {
    setState("disconnected", "未找到佳能遥控服务。请在相机上打开“连接至无线遥控器 → 添加设备”，再重试。");
    return false;
  }
  setState("disconnected", "找到相机，请选择要连接的相机");
  return true;
}

static bool connectCamera(const Candidate &camera) {
  if (!savedAddress.isEmpty() && savedAddress != camera.address) {
    setState("disconnected", "更换相机前，请先点击“清除配对 / 更换相机”");
    return false;
  }
  String attempt = prefs.getString("attempt", "");
  if (attempt.length() && attempt != camera.address) {
    setState("disconnected", "有未完成的配对记录，请先清除配对再换相机");
    return false;
  }
  if (!disconnectCamera()) return false;
  // Retain the attempted peer too: a failed registration may already have
  // created a BLE bond. The user must be able to clear that incomplete pairing.
  if (!prefs.putString("attempt", camera.address) || !prefs.putUChar("attempt_type", camera.addressType))
    return fail("无法保存配对准备信息，请重启设备后重试");
  authResult = 0;
  authReason = 0;
  setState("connecting", "正在连接 " + camera.name + "…");
  if (!client->connect(BLEAddress(camera.address, camera.addressType), camera.addressType, 12000))
    return fail("蓝牙连接失败，请唤醒相机后重试；换相机前请先清除配对。");

  setState("authenticating", "蓝牙已接通，正在验证配对…");
  // Do not treat a transport connection, or merely starting encryption, as ready.
  if (!client->secureConnection()) return fail("蓝牙加密握手未完成，请删除相机端旧遥控记录并重新配对。");
  uint32_t started = millis();
  while (authResult == 0 && linkUp && millis() - started < 12000) delay(20);
  if (authResult != 1 || !linkUp)
    return fail("蓝牙认证失败（代码 " + String(authReason.load()) + "），请检查相机配对画面。");

  setState("identifying", "正在注册 XIAO-Shutter 遥控器…");
  // Arduino 3.3.8 retains a service cache across reconnects. Refresh handles so
  // another camera (or a changed camera firmware) never uses old GATT handles.
  client->getServices();
  BLERemoteService *service = client->getService(SERVICE);
  if (!service) return fail("相机缺少佳能遥控服务，请确认选择的是无线遥控器配对。");
  auto *identify = service->getCharacteristic(IDENTIFY);
  control = service->getCharacteristic(CONTROL);
  if (!identify || !control || !identify->canWrite() || !control->canWrite())
    return fail("相机的身份或快门服务不可用，当前协议可能不兼容。");
  uint8_t identity[sizeof(REMOTE_NAME)];
  identity[0] = 0x03;
  memcpy(identity + 1, REMOTE_NAME, sizeof(REMOTE_NAME) - 1);
  if (!identify->writeValue(identity, sizeof(identity), true) || !linkUp)
    return fail("相机未接受遥控器身份，请重新进入相机的配对画面。");
  delay(300);
  if (!linkUp) return fail("注册后相机断开了连接，请检查相机配对确认提示。");

  // Save address type as well as address; random-address peripherals need both.
  bool persisted = prefs.putString("address", camera.address) > 0;
  persisted = prefs.putString("name", camera.name) > 0 && persisted;
  persisted = prefs.putUChar("addr_type", camera.addressType) > 0 && persisted;
  if (!persisted) return fail("配对信息保存失败，请重启设备后重试。");
  prefs.remove("attempt");
  prefs.remove("attempt_type");
  xSemaphoreTake(stateLock, portMAX_DELAY);
  savedAddress = camera.address;
  savedName = camera.name;
  savedType = camera.addressType;
  activeName = camera.name;
  xSemaphoreGive(stateLock);
  ready = true;
  setState("connected", "相机已连接，可发送快门指令");
  return true;
}

static bool sendShutter() {
  if (!ready || !linkUp || !client->isConnected() || !control) {
    setState("disconnected", "相机未连接，未发送快门指令");
    return false;
  }
  setState("shooting", "正在发送单次快门指令…");
  bool pressed = control->writeValue(uint8_t(0x8c), true);
  delay(200);
  // Always attempt release, even when press returned an error. Never retry press.
  bool released = linkUp && control->writeValue(uint8_t(0x0c), true);
  if (!released) return fail("快门释放未得到确认，已断开连接；请在相机上检查快门状态。");
  if (!pressed) {
    setState("connected", "快门写入未得到确认，已发送释放指令；请检查相机，勿自动重试。");
    return false;
  }
  xSemaphoreTake(stateLock, portMAX_DELAY);
  ++shotsSent;
  xSemaphoreGive(stateLock);
  setState("connected", "快门指令已发送，请在相机上确认照片");
  return true;
}

static bool forgetCamera() {
  if (!disconnectCamera()) return false;
  const char *keys[] = {"address", "attempt"};
  const char *types[] = {"addr_type", "attempt_type"};
  for (size_t i = 0; i < 2; ++i) {
    String old = prefs.getString(keys[i], "");
    if (old.length() != 17) continue;
    BLEAddress address(old, prefs.getUChar(types[i], BLE_ADDR_PUBLIC));
    ble_addr_t peer = {};
    peer.type = address.getType();
    memcpy(peer.val, address.getNative(), 6);
    int rc = ble_gap_unpair(&peer);
    if (rc != 0 && rc != BLE_HS_ENOENT) {
      setState("error", "清除蓝牙配对记录失败，请重启后重试");
      return false;
    }
    delay(250);
  }
  if (!prefs.clear()) {
    setState("error", "清除已保存相机信息失败");
    return false;
  }
  xSemaphoreTake(stateLock, portMAX_DELAY);
  savedAddress = "";
  savedName = "";
  candidates.clear();
  xSemaphoreGive(stateLock);
  setState("disconnected", "本机配对已清除。若重配同一台相机，也请删除相机端的旧遥控记录。");
  return true;
}

static void worker(void *) {
  Command command;
  while (true) {
    if (xQueueReceive(commands, &command, portMAX_DELAY) != pdTRUE) continue;
    bool ok = false;
    String action(command.action);
    if (action == "disconnect") {
      ok = disconnectCamera();
      if (ok) setState("disconnected", "已断开连接，配对信息已保留");
    } else if (action == "forget") {
      ok = forgetCamera();
    } else if (action == "shutter") {
      ok = sendShutter();
    } else if (action == "scan" || action == "pair") {
      ok = scanCameras();
      if (ok && action == "pair" && candidates.size() == 1) ok = connectCamera(candidates.front());
    } else if (action == "connect") {
      Candidate selected;
      bool found = false;
      xSemaphoreTake(stateLock, portMAX_DELAY);
      if (!command.address[0] && savedAddress.length()) {
        selected = {savedAddress, savedName, savedType, 0};
        found = true;
      }
      for (auto &item : candidates) {
        if (item.address == command.address) { selected = item; found = true; break; }
      }
      xSemaphoreGive(stateLock);
      if (found) ok = connectCamera(selected);
      else setState("disconnected", "没有可连接的相机，请先搜索或配对新相机");
    }
    busy = false;
    String detail;
    xSemaphoreTake(stateLock, portMAX_DELAY);
    detail = message;
    xSemaphoreGive(stateLock);
    result(command.id, ok, detail);
    status();
  }
}

static void receiveLine(const String &line) {
  JsonDocument doc;
  if (deserializeJson(doc, line)) { result("", false, "无效 JSON 命令"); return; }
  String id = doc["id"] | "";
  String action = doc["action"] | "";
  if (action == "status") { status(); return; }
  if (action != "pair" && action != "scan" && action != "connect" &&
      action != "disconnect" && action != "forget" && action != "shutter") {
    result(id.c_str(), false, "未知操作"); return;
  }
  if (busy.exchange(true)) { result(id.c_str(), false, "设备忙碌，请等待当前操作结束"); return; }
  Command command = {};
  strlcpy(command.id, id.c_str(), sizeof(command.id));
  strlcpy(command.action, action.c_str(), sizeof(command.action));
  strlcpy(command.address, doc["address"] | "", sizeof(command.address));
  if (xQueueSend(commands, &command, 0) != pdTRUE) {
    busy = false;
    result(command.id, false, "操作队列忙碌，请稍后重试");
  }
}

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(100);
  Serial.begin(115200);
  stateLock = xSemaphoreCreateMutex();
  outputLock = xSemaphoreCreateMutex();
  commands = xQueueCreate(1, sizeof(Command));
  if (!stateLock || !outputLock || !commands || !prefs.begin("canon-shutter", false)) {
    Serial.println("{\"type\":\"fatal\",\"message\":\"初始化失败\"}");
    while (true) delay(1000);
  }
  savedAddress = prefs.getString("address", "");
  savedName = String(prefs.getString("name", "Canon EOS").c_str());
  savedType = prefs.getUChar("addr_type", BLE_ADDR_PUBLIC);
  BLEDevice::init(REMOTE_NAME);
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());
  BLESecurity *security = new BLESecurity();
  security->setAuthenticationMode(true, false, true);
  security->setCapability(ESP_IO_CAP_NONE);
  security->setForceAuthentication(false);
  client = BLEDevice::createClient();
  client->setClientCallbacks(new ConnectionCallbacks());
  if (xTaskCreate(worker, "canon-ble", 12288, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("{\"type\":\"fatal\",\"message\":\"BLE任务创建失败\"}");
    while (true) delay(1000);
  }
  status();
}

void loop() {
  static String line;
  static bool overflow = false;
  static uint32_t lastStatus = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (overflow) result("", false, "命令过长");
      else if (line.length()) receiveLine(line);
      line = "";
      overflow = false;
    } else if (c != '\r') {
      if (line.length() < 512 && !overflow) line += c;
      else overflow = true;
    }
  }
  uint32_t deadline = stageDeadline.load();
  if (busy && deadline && int32_t(millis() - deadline) >= 0) {
    stageDeadline = 0;
    ready = false;
    // Use the NimBLE GAP API from the responsive serial task to unblock a
    // stalled GATT/security operation. Do not reset or replay any command.
    uint16_t handle = activeHandle.load();
    if (handle != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    else ble_gap_conn_cancel();
    setState("error", "等待相机响应超时，正在结束连接；未自动重试");
  }
  if (!busy && !linkUp) {
    xSemaphoreTake(stateLock, portMAX_DELAY);
    if (phase == "connected") {
      phase = "disconnected";
      message = "相机已断开，请唤醒相机并点击连接";
      activeName = "";
    }
    xSemaphoreGive(stateLock);
  }
  if (millis() - lastStatus >= 750) { status(); lastStatus = millis(); }
  delay(5);
}
