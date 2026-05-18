#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESP32-TWAI-CAN.hpp>
#include <EEPROM.h>
#include <NimBLEDevice.h>
#include <lvgl.h>
#include <HTTPClient.h>
#include <Update.h>

#define CAN_TX 17  // Connects to CTX
#define CAN_RX 16  // Connects to CRX

CanFrame rxFrame;  // Create frame to read
const char* ssid = "TECH-EBIKE_BMS1";
const char* password = "12345678";

AsyncWebServer server(80);

// Địa chỉ MAC của màn hình
uint8_t masterAddress[] = {0x80, 0x65, 0x99, 0xBC, 0x5C, 0xF4};

// --- KHAI BÁO CÁC BIẾN TOÀN CỤC ---
uint8_t a4, a5, a6, a7, var1_1, var2_1, var1_2, var2_2, var1_3, var2_3, var1_4, var2_4;
uint8_t var1_31, var2_31, var1_32, var2_32, var1_33, var2_33, var1_34, var2_34;
uint8_t var_100, var_101, var_102, var1_27, var2_27, var1_28, var2_28, var1_29, var2_29, var1_30, var2_30;
uint8_t var1_5, var2_5, var1_6, var2_6, var1_7, var2_7, var1_8, var2_8;
uint8_t var1_9, var2_9, var1_10, var2_10, var1_11, var2_11, var1_12, var2_12;
uint8_t var1_13, var2_13, var1_14, var2_14, var1_15, var2_15, var1_16, var2_16;
uint8_t var1_17, var2_17, var1_18, var2_18, var1_19, var2_19, var1_20, var2_20;
uint8_t var1_21, var2_21, var1_22, var2_22, var1_23, var2_23, var1_24, var2_24;
uint8_t var1_25, var2_25, var1_26, var2_26;

uint8_t soh_lfp;
uint16_t rcap_lfp, v_max_lfp, v_min_lfp, cap_lfp;
uint16_t var3_1, var3_2, var3_3, var3_4, var3_5, var3_6, var3_7, var3_8, var3_9, var3_10;
uint16_t var3_11, var3_12, var3_13, var3_14, var3_15, var3_16, var3_17, var3_18, var3_19, var3_20;
uint16_t var21, var22, var23, var24, var25, var26, var27, var33;

float ampe_charge, a_load, volpin;

typedef struct struct_message {
    int id;      
    int battery; 
} struct_message;
struct_message myData;

int NODE_ID = 5; 
unsigned long lastSend = 0;
esp_now_peer_info_t peerInfo;

// Biến lưu trữ URL OTA và cờ kích hoạt để xử lý trong Loop (tránh crash mạng)
String otaTargetUrl = "";
bool shouldStartOTA = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Box %d -> %s\n", NODE_ID, status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// Hàm tính toán Uptime định dạng chuỗi gửi lên Web
String getUptimeString() {
  unsigned long sec = millis() / 1000;
  unsigned long min = sec / 60;
  unsigned long hr = min / 60;
  unsigned long day = hr / 24;
  return String(day) + "D" + String(hr % 24) + "H" + String(min % 60) + "M" + String(sec % 60) + "S";
}

String getJSONData() {
  String json = "{";
  json += "\"wifi_status\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"uptime\":\"" + getUptimeString() + "\",";
  json += "\"volpin\":" + String(volpin, 2) + ",";
  json += "\"cap_lfp_ok\":" + String((float)rcap_lfp / 1000.0, 2) + ",";
  json += "\"soh_lfp_a\":" + String(soh_lfp) + ",";
  json += "\"min_lfp\":" + String(var3_2 * 0.0001, 2) + ",";
  json += "\"max_lfp\":" + String(var3_1 * 0.0001, 2) + ",";
  json += "\"delta_vol\":" + String(((var3_1 * 0.0001) - (var3_2 * 0.0001)), 2) + ",";
  json += "\"ampe\":" + String(ampe_charge, 2) + ",";
  json += "\"soc\":" + String(var2_32) + ",";
  json += "\"cycle\":" + String(var33) + ",";
  json += "\"vmax\":" + String(var3_1 * 0.0001, 3) + ",";
  json += "\"vmin\":" + String(var3_2 * 0.0001, 3) + ",";
  json += "\"t1\":" + String(var_100) + ",";
  json += "\"t2\":" + String(var_101) + ",";
  json += "\"tfet\":" + String(var_102) + ",";
  json += "\"cells\":[";
  uint16_t all_cells[] = {var3_5, var3_6, var3_7, var3_8, var3_9, var3_10, var3_11, var3_12, var3_13, var3_14, var3_15, var3_16, var3_17, var3_18, var3_19, var3_20, var21, var22, var23, var24, var25, var26};
  for(int i=0; i<22; i++) {
    json += String(all_cells[i] * 0.0001, 4);
    if(i < 21) json += ",";
  }
  json += "]}";
  return json;
}

void read_data_pin_24() {
  // Đọc liên tục hàng đợi dữ liệu CAN nhận được thay vì nhảy IF rời rạc dễ mất gói
  while (ESP32Can.readFrame(rxFrame, 0)) { 
    switch (rxFrame.identifier) {
      case 0x309:
        a4 = rxFrame.data[4]; a5 = rxFrame.data[5]; a6 = rxFrame.data[6]; a7 = rxFrame.data[7];
        ampe_charge = (word(a6, a7)) * 2 * 0.01;
        break;
      case 0x310:
        var1_1 = rxFrame.data[1]; var2_1 = rxFrame.data[0]; var1_2 = rxFrame.data[3]; var2_2 = rxFrame.data[2];
        var1_3 = rxFrame.data[5]; var2_3 = rxFrame.data[4]; var1_4 = rxFrame.data[7]; var2_4 = rxFrame.data[6];
        break;
      case 0x30A:
        var1_31 = rxFrame.data[1]; var2_31 = rxFrame.data[0]; var1_32 = rxFrame.data[3]; var2_32 = rxFrame.data[2];
        var1_33 = rxFrame.data[5]; var2_33 = rxFrame.data[4]; var1_34 = rxFrame.data[7]; var2_34 = rxFrame.data[6];
        break;
      case 0x320:
        var_100 = rxFrame.data[1]; var_101 = rxFrame.data[0]; var_102 = rxFrame.data[3];
        break;
      case 0x30E:
        var1_27 = rxFrame.data[1]; var2_27 = rxFrame.data[0]; var1_28 = rxFrame.data[3]; var2_28 = rxFrame.data[2];
        var1_29 = rxFrame.data[5]; var2_29 = rxFrame.data[4]; var1_30 = rxFrame.data[7]; var2_30 = rxFrame.data[6];
        break;
      case 0x311:
        var1_5 = rxFrame.data[1]; var2_5 = rxFrame.data[0]; var1_6 = rxFrame.data[3]; var2_6 = rxFrame.data[2];
        var1_7 = rxFrame.data[5]; var2_7 = rxFrame.data[4]; var1_8 = rxFrame.data[7]; var2_8 = rxFrame.data[6];
        break;
      case 0x312:
        var1_9 = rxFrame.data[1]; var2_9 = rxFrame.data[0]; var1_10 = rxFrame.data[3]; var2_10 = rxFrame.data[2];
        var1_11 = rxFrame.data[5]; var2_11 = rxFrame.data[4]; var1_12 = rxFrame.data[7]; var2_12 = rxFrame.data[6];
        break;
      case 0x313:
        var1_13 = rxFrame.data[1]; var2_13 = rxFrame.data[0]; var1_14 = rxFrame.data[3]; var2_14 = rxFrame.data[2];
        var1_15 = rxFrame.data[5]; var2_15 = rxFrame.data[4]; var1_16 = rxFrame.data[7]; var2_16 = rxFrame.data[6];
        break;
      case 0x314:
        var1_17 = rxFrame.data[1]; var2_17 = rxFrame.data[0]; var1_18 = rxFrame.data[3]; var2_18 = rxFrame.data[2];
        var1_19 = rxFrame.data[5]; var2_19 = rxFrame.data[4]; var1_20 = rxFrame.data[7]; var2_20 = rxFrame.data[6];
        break;
      case 0x31A:
        var1_21 = rxFrame.data[1]; var2_21 = rxFrame.data[0]; var1_22 = rxFrame.data[3]; var2_22 = rxFrame.data[2];
        var1_23 = rxFrame.data[5]; var2_23 = rxFrame.data[4]; var1_24 = rxFrame.data[7]; var2_24 = rxFrame.data[6];
        break;
      case 0x31B:
        var1_25 = rxFrame.data[1]; var2_25 = rxFrame.data[0]; var1_26 = rxFrame.data[3]; var2_26 = rxFrame.data[2];
        break;
    }
  }

  rcap_lfp = word(var2_31, var1_31);
  soh_lfp = var1_32;

  // Ghép các cặp Byte dữ liệu thành Word thông tin hoàn chỉnh
  var3_1 = word(var2_1, var1_1);   var3_2 = word(var2_2, var1_2);   var3_3 = word(var2_3, var1_3);   var3_4 = word(var2_4, var1_4);
  var3_5 = word(var2_5, var1_5);   var3_6 = word(var2_6, var1_6);   var3_7 = word(var2_7, var1_7);   var3_8 = word(var2_8, var1_8);
  var3_9 = word(var2_9, var1_9);   var3_10 = word(var2_10, var1_10); var3_11 = word(var2_11, var1_11); var3_12 = word(var2_12, var1_12);
  var3_13 = word(var2_13, var1_13); var3_14 = word(var2_14, var1_14); var3_15 = word(var2_15, var1_15); var3_16 = word(var2_16, var1_16);
  var3_17 = word(var2_17, var1_17); var3_18 = word(var2_18, var1_18); var3_19 = word(var2_19, var1_19); var3_20 = word(var2_20, var1_20);
  
  var21 = word(var2_21, var1_21); var22 = word(var2_22, var1_22); var23 = word(var2_23, var1_23); var24 = word(var2_24, var1_24);
  var25 = word(var2_25, var1_25); var26 = word(var2_26, var1_26); var27 = word(var2_27, var1_27); var33 = word(var2_33, var1_33);

  v_max_lfp = var3_3; // ID 0x310 data[4][5] tương ứng Vmax
  v_min_lfp = var3_4; // ID 0x310 data[6][7] tương ứng Vmin

  if (ampe_charge < 0 || ampe_charge > 30) ampe_charge = 0;
  if (a_load > 100 || a_load < 0) a_load = 0;

  // Tính tổng điện áp của bộ pin (22 cells)
  volpin = (var3_5 + var3_6 + var3_7 + var3_8 + var3_9 + var3_10 + var3_11 + var3_12 + var3_13 +
            var3_14 + var3_15 + var3_16 + var3_17 + var3_18 + var3_19 + var3_20 + var21 +
            var22 + var23 + var24 + var25 + var26) * 0.0001;
}

// Tiến hành flash OTA
void performOTA(String url) {
  Serial.println("Starting OTA from: " + url);
  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);
    if (canBegin) {
      WiFiClient* client = http.getStreamPtr();
      size_t written = Update.writeStream(*client);
      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      }
      if (Update.end()) {
        if (Update.isFinished()) {
          Serial.println("OTA Update success. Rebooting...");
          delay(500);
          ESP.restart();
        }
      }
    }
  }
  http.end();
}

void setup() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.begin(115200);
  
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(20);
  ESP32Can.setTxQueueSize(20);
  
  if (ESP32Can.begin(ESP32Can.convertSpeed(250))) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, masterAddress, 6);
    peerInfo.channel = 0; 
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
  
  if (!LittleFS.begin(true)) return;

  // Ghi file index.html tích hợp Container 1, 2, 3
  File file = LittleFS.open("/index.html", FILE_WRITE);
  String htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Tech-Ebike BMS</title>
    <style>
        :root {
            --bg-body: #0d1117;
            --bg-card: #161b22;
            --bg-inner: #1c2128;
            --accent-yellow: #f1f50a;
            --accent-blue: #58a6ff;
            --text-main: #ffffff;
            --text-dim: #8b949e;
            --green-glow: #4ade80;
            --border: rgba(255, 255, 255, 0.1);
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: var(--bg-body);
            color: var(--text-main);
            font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            padding-bottom: 100px;
        }
        .header { text-align: center; padding: 30px 0 10px; }
        .hex-logo {
            width: 120px; height: 130px;
            background: linear-gradient(145deg, #1e2a5e, #0f1e4a);
            margin: 0 auto 15px;
            clip-path: polygon(50% 0%, 100% 25%, 100% 75%, 50% 100%, 0% 75%, 0% 25%);
            display: flex; align-items: center; justify-content: center;
            box-shadow: 0 0 30px rgba(88, 166, 255, 0.2);
        }
        .logo-text { font-size: 70px; font-weight: bold; color: white; text-shadow: 0 0 20px rgba(255,255,255,0.8); }
        .brand-name {
            color: var(--accent-yellow);
            font-size: 28px; font-weight: 900;
            text-transform: uppercase; letter-spacing: 2px;
            margin-bottom: 5px;
        }
        .uptime { color: var(--text-dim); font-size: 12px; margin-bottom: 20px; }
        .status-bar {
            display: flex; justify-content: space-around;
            padding: 15px 0; background: rgba(0,0,0,0.3);
            border-top: 1px solid var(--border); border-bottom: 1px solid var(--border);
            font-size: 14px;
        }
        .status-item span { color: var(--green-glow); font-weight: bold; }
        
        .container1, .container2, .container3 { max-width: 500px; margin: 0 auto; padding: 15px; }
        .hidden { display: none !important; }

        .main-card {
            background: var(--bg-card);
            border-radius: 25px; padding: 30px 20px;
            border: 1px solid var(--border);
            box-shadow: 0 10px 30px rgba(0,0,0,0.5);
            margin-bottom: 20px;
        }
        .row-large { display: flex; justify-content: center; gap: 30px; margin-bottom: 15px; }
        .val-group { text-align: center; }
        .val-big { font-size: 48px; font-weight: 700; color: white; }
        .unit-big { font-size: 20px; color: var(--text-dim); margin-left: 5px; }
        .row-sub { display: flex; justify-content: center; gap: 40px; margin-bottom: 25px; font-size: 16px; }
        .sub-label { color: var(--text-dim); }
        .sub-val { color: var(--green-glow); font-weight: 600; }

        .details-box {
            background: var(--bg-inner);
            border-radius: 15px; padding: 20px;
            display: flex; flex-direction: column; gap: 15px;
        }
        .detail-item { display: flex; justify-content: space-between; font-size: 15px; }
        .d-label { color: var(--text-dim); }
        .d-val { color: var(--green-glow); font-weight: 500; }

        .form-group { display: flex; flex-direction: column; gap: 8px; margin-bottom: 15px; }
        .form-group label { font-size: 14px; color: var(--text-dim); }
        .form-input {
            background: var(--bg-inner); border: 1px solid var(--border);
            padding: 12px; border-radius: 8px; color: white; font-size: 16px;
        }
        .btn-action {
            background: linear-gradient(90deg, #58a6ff, #1f6feb);
            color: white; border: none; padding: 14px; font-weight: bold;
            border-radius: 8px; cursor: pointer; text-transform: uppercase; width: 100%;
            margin-top: 10px; box-shadow: 0 4px 15px rgba(88,166,255,0.3);
        }
        .btn-secondary {
            background: #21262d; border: 1px solid var(--border); color: var(--text-main);
        }
        
        .git-item {
            background: var(--bg-inner); border-radius: 12px; padding: 15px;
            display: flex; justify-content: space-between; align-items: center;
            border: 1px solid var(--border); margin-bottom: 12px;
        }
        .git-info { display: flex; flex-direction: column; gap: 4px; }
        .git-ver { font-weight: bold; color: var(--accent-yellow); font-size: 16px; }
        .git-date { font-size: 12px; color: var(--text-dim); }
        .btn-update-code {
            background: #238636; border: none; color: white; padding: 8px 16px;
            font-size: 13px; font-weight: bold; border-radius: 6px; cursor: pointer;
        }

        .grid-title { padding: 10px 20px; font-size: 14px; color: var(--accent-blue); text-transform: uppercase; }
        .cells-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; padding: 0 15px; }
        .cell { background: var(--bg-card); border-radius: 8px; padding: 10px 5px; text-align: center; border: 1px solid var(--border); }
        .c-idx { font-size: 10px; color: var(--accent-blue); display: block; }
        .c-v { font-size: 14px; font-weight: 600; }

        .bottom-nav {
            position: fixed; bottom: 0; width: 100%; height: 70px;
            background: #0d1117; border-top: 1px solid var(--border);
            display: flex; justify-content: space-around; align-items: center; z-index: 100;
        }
        .nav-item { text-align: center; color: var(--text-dim); font-size: 12px; cursor: pointer; }
        .nav-item.active { color: var(--accent-blue); }

        .label-mini { font-size: 12px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 1px; display: block; margin-bottom: 8px; }
        .battery-container { width: 100px; height: 45px; position: relative; margin: 0 auto; }
        .battery-body { width: 92px; height: 100%; border: 2px solid #ffffff; border-radius: 6px; position: relative; padding: 2px; overflow: hidden; display: flex; align-items: center; justify-content: center; }
        .battery-head { width: 6px; height: 16px; background: #ffffff; position: absolute; right: 0; top: 50%; transform: translateY(-50%); border-radius: 0 3px 3px 0; }
        .battery-level { height: 100%; background: linear-gradient(90deg, #4ade80, #22c55e); position: absolute; left: 0; top: 0; transition: width 0.5s ease-in-out; z-index: 1; }
        .soc-text { position: relative; z-index: 2; font-weight: bold; font-size: 18px; color: #ffffff; text-shadow: 1px 1px 2px rgba(0,0,0,0.8); }
        .low-battery { background: linear-gradient(90deg, #f87171, #ef4444); }
        .mid-battery { background: linear-gradient(90deg, #fbbf24, #f59e0b); }
    </style>
</head>
<body>

    <div class="header">
        <div class="hex-logo"><div class="logo-text">T</div></div>
        <div class="brand-name">Tech-Ebike</div>
        <div class="uptime">TIME: <span id="uptime">0D0H0M0S</span></div>
    </div>

    <div class="status-bar">
        <div class="status-item">Charge: <span id="st-chg">ON</span></div>
        <div class="status-item">Discharge: <span id="st-dis">ON</span></div>
        <div class="status-item">Balance: <span id="st-bal" style="color:#8b949e">OFF</span></div>
    </div>

    <!-- CONTAINER 1 -->
    <div id="c1" class="container1 hidden">
        <div class="main-card">
            <h3 style="margin-bottom: 20px; text-align: center; color: var(--accent-blue);">KẾT NỐI WIFI HỆ THỐNG</h3>
            <div class="form-group">
                <label>Tên Wi-Fi (SSID)</label>
                <input type="text" id="wifi_ssid" class="form-input" placeholder="Nhập tên mạng WiFi">
            </div>
            <div class="form-group">
                <label>Mật khẩu</label>
                <input type="password" id="wifi_pass" class="form-input" placeholder="Nhập mật khẩu">
            </div>
            <button class="btn-action" onclick="connectWiFi()">Kết Nối Online</button>
            <button class="btn-action btn-secondary" onclick="showContainer(2)">Quay Lại Giám Sát</button>
        </div>
    </div>

    <!-- CONTAINER 2 -->
    <div id="c2" class="container2">
        <div class="main-card">
            <div class="row-large">
                <div class="val-group">
                    <span class="label-mini"></span>
                    <div class="battery-container">
                        <div class="battery-head"></div>
                        <div class="battery-body">
                            <div id="battery-level" class="battery-level" style="width: 0%;"></div>
                            <span id="soc" class="soc-text">0%</span>
                        </div>
                    </div>
                </div>
                <div class="val-group"><span class="val-big" id="pwr">0.0W</span></div>
            </div>
            
            <div class="row-sub">
                <div><span class="sub-label">Vol_pin: </span> <span class="sub-val" id="v">0.00V</span></div>
                <div><span class="sub-label">Ampe: </span> <span class="sub-val" id="a">0.00A</span></div>
            </div>

            <div class="details-box">
                <div class="detail-item"><span class="d-label">Ave. Cell Volt.</span><span class="d-val" id="avg">0.000 V</span></div>
                <div class="detail-item"><span class="d-label">Remain Capacity</span><span class="d-val" id="cap">0.0 Ah</span></div>
                <div class="detail-item"><span class="d-label">Cycle</span><span class="d-val" id="cycle">00</span></div>
                <div class="detail-item"><span class="d-label">Delvol</span><span class="d-val" id="delvol">00</span></div>
                <div class="detail-item"><span class="d-label">Vmax</span><span class="d-val" id="vmax">00</span></div>
                <div class="detail-item"><span class="d-label">Vmin</span><span class="d-val" id="vmin">00</span></div>
                <div class="detail-item"><span class="d-label">RCAP</span><span class="d-val" id="rcap">00</span></div>
                <div class="detail-item"><span class="d-label">SOH</span><span class="d-val" id="soh">00</span></div>
                <div class="detail-item"><span class="d-label">T_1</span><span class="d-val" id="t1">0.0 °C</span></div>
                <div class="detail-item"><span class="d-label">T_2</span><span class="d-val" id="t2">0.0 °C</span></div>
                <div class="detail-item"><span class="d-label">T_FET</span><span class="d-val" id="tfet">0.0 °C</span></div>
            </div>
        </div>
        <div class="grid-title">Cell Voltages</div>
        <div id="grid" class="cells-grid"></div>
    </div>

    <!-- CONTAINER 3 -->
    <div id="c3" class="container3 hidden">
        <div class="main-card">
            <h3 style="margin-bottom: 20px; text-align: center; color: var(--green-glow);">DANH SÁCH PHIÊN BẢN GIT</h3>
            <div id="git-list">
                <div class="status-item" style="text-align:center;color:var(--text-dim)">Đang tải danh sách từ GitHub...</div>
            </div>
            <button class="btn-action btn-secondary" style="margin-top:20px;" onclick="showContainer(2)">Quay Lại Giám Sát</button>
        </div>
    </div>

    <div class="bottom-nav">
        <div id="nav-monitor" class="nav-item active" onclick="showContainer(2)">📊<br>Status</div>
        <div id="nav-update" class="nav-item" onclick="handleUpdateTrigger()">🔄<br>Update</div>
    </div>

    <script>
        let isConnectedGlobal = false;

        function showContainer(num) {
            document.getElementById('c1').classList.add('hidden');
            document.getElementById('c2').classList.add('hidden');
            document.getElementById('c3').classList.add('hidden');
            document.getElementById('nav-monitor').classList.remove('active');
            document.getElementById('nav-update').classList.remove('active');

            if(num === 1) {
                document.getElementById('c1').classList.remove('hidden');
                document.getElementById('nav-update').classList.add('active');
            } else if(num === 2) {
                document.getElementById('c2').classList.remove('hidden');
                document.getElementById('nav-monitor').classList.add('active');
            } else if(num === 3) {
                document.getElementById('c3').classList.remove('hidden');
                document.getElementById('nav-update').classList.add('active');
                loadGitVersions();
            }
        }

        function handleUpdateTrigger() {
            if(isConnectedGlobal) {
                showContainer(3);
            } else {
                showContainer(1);
            }
        }

        function connectWiFi() {
            const ssid = document.getElementById('wifi_ssid').value;
            const pass = document.getElementById('wifi_pass').value;
            if(!ssid) return alert("Vui lòng nhập tên WiFi");
            
            alert("Đang gửi yêu cầu kết nối! Đợi khoảng 5-10 giây kiểm tra trạng thái.");
            fetch(`/connect?ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}`)
            .then(res => res.text())
            .then(txt => { console.log(txt); });
        }

        function loadGitVersions() {
            const mockGitData = [
                { ver: "v2.0.4-Beta", date: "May 2026", url: "https://your-server.com/firmware_v204.bin" },
                { ver: "v2.0.3-Stable", date: "April 2026", url: "https://your-server.com/firmware_v203.bin" },
                { ver: "v1.9.0-Legacy", date: "Jan 2026", url: "https://your-server.com/firmware_v190.bin" }
            ];

            let html = '';
            mockGitData.forEach(item => {
                html += `
                <div class="git-item">
                    <div class="git-info">
                        <span class="git-ver">${item.ver}</span>
                        <span class="git-date">Released: ${item.date}</span>
                    </div>
                    <button class="btn-update-code" onclick="triggerOTA('${item.url}')">Cập nhật</button>
                </div>`;
            });
            document.getElementById('git-list').innerHTML = html;
        }

        function triggerOTA(binUrl) {
            if(confirm("Bạn có chắc chắn muốn flash phiên bản này không?")) {
                alert("Chip bắt đầu tải code và cập nhật OTA. Giao diện sẽ tạm ngắt kết nối!");
                fetch(`/ota?url=${encodeURIComponent(binUrl)}`);
            }
        }

        function update() {
            fetch('/data').then(res => res.json()).then(data => {
                isConnectedGlobal = data.wifi_status;
                document.getElementById('uptime').innerText = data.uptime;
                
                if(isConnectedGlobal && !document.getElementById('c1').classList.contains('hidden')) {
                    showContainer(3);
                }

                const soc = data.soc;
                const batLevel = document.getElementById('battery-level');
                document.getElementById('soc').innerText = soc + "%";
                batLevel.style.width = soc + "%";

                batLevel.classList.remove('low-battery', 'mid-battery');
                if (soc <= 20) {
                    batLevel.classList.add('low-battery');
                } else if (soc <= 50) {
                    batLevel.classList.add('mid-battery');
                }
                document.getElementById('v').innerText = data.volpin.toFixed(2) + "V";
                document.getElementById('a').innerText = data.ampe.toFixed(2) + "A";
                document.getElementById('pwr').innerText = (data.volpin * data.ampe).toFixed(1) + "W";
                document.getElementById('cycle').innerText = data.cycle;
                document.getElementById('delvol').innerText = data.delta_vol.toFixed(3) + " V";
                document.getElementById('vmax').innerText = data.max_lfp.toFixed(3) + " V";
                document.getElementById('vmin').innerText = data.min_lfp.toFixed(3) + " V";
                document.getElementById('rcap').innerText = data.cap_lfp_ok.toFixed(2) + " Ah";
                document.getElementById('cap').innerText = data.cap_lfp_ok.toFixed(2) + " Ah";
                document.getElementById('soh').innerText = data.soh_lfp_a + "%";

                document.getElementById('t1').innerText = data.t1 + ".0 °C";
                document.getElementById('t2').innerText = data.t2 + ".0 °C";
                document.getElementById('tfet').innerText = data.tfet + ".0 °C";
                
                let sum = 0;
                let gridHtml = '';
                data.cells.forEach((v, i) => {
                    sum += v;
                    gridHtml += `<div class="cell"><span class="c-idx">${i+1}</span><span class="c-v">${v.toFixed(3)}</span></div>`;
                });
                document.getElementById('avg').innerText = (sum / data.cells.length).toFixed(3) + " V";
                document.getElementById('grid').innerHTML = gridHtml;
            }).catch(err => console.log("Lỗi lấy dữ liệu"));
        }
        setInterval(update, 1000);
    </script>
</body>
</html>
)rawliteral";

  file.print(htmlContent);
  file.close();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request){
    request->send(200, "application/json", getJSONData());
  });

  server.on("/connect", HTTP_GET, [](AsyncWebServerRequest* request){
    if (request->hasParam("ssid") && request->hasParam("pass")) {
      String req_ssid = request->getParam("ssid")->value();
      String req_pass = request->getParam("pass")->value();
      WiFi.begin(req_ssid.c_str(), req_pass.c_str());
      request->send(200, "text/plain", "Đang kết nối...");
    } else {
      request->send(400, "text/plain", "Lỗi tham số");
    }
  });

  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest* request){
    if (request->hasParam("url")) {
      otaTargetUrl = request->getParam("url")->value();
      shouldStartOTA = true; // Bật cờ thực hiện trong loop chính
      request->send(200, "text/plain", "Kích hoạt cập nhật OTA...");
    } else {
      request->send(400, "text/plain", "Thiếu URL");
    }
  });

  server.begin();
}

void loop() {
  read_data_pin_24();
  
  // Gửi ESP-NOW định kỳ mỗi 2 giây
  if (millis() - lastSend > 2000) {
    myData.id = NODE_ID;
    myData.battery = var2_32; 
    esp_now_send(masterAddress, (uint8_t *) &myData, sizeof(myData));
    lastSend = millis();
  }

  // Thực hiện tác vụ tải OTA tại đây để tránh ngắt và xung đột tiến trình WebServer
  if (shouldStartOTA) {
    shouldStartOTA = false;
    performOTA(otaTargetUrl);
  }
}