#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>

// Import cấu hình và thông tin bảo mật từ các file riêng
#include "config.h"
#include "secrets.h"

using namespace websockets;

// ===================== BIẾN TOÀN CỤC =====================
WebsocketsClient client;
unsigned long lastFrameTime = 0;
char lastAICommand = 'S'; // Mặc định là Dừng (Stop)

// ===================== HÀM XỬ LÝ JSON TỪ SERVER =====================
void onMessageCallback(WebsocketsMessage message) {
  if (!message.isText()) return;

  String data = message.data();
  Serial.print(">>> Server: ");
  Serial.println(data);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, data);
  if (error) {
    Serial.print(F("❌ JSON Error: "));
    Serial.println(error.f_str());
    return;
  }

  const char* cmd_ptr = doc["command"];
  const char* mode = doc["mode"] | "auto";
  const char* class_label = doc["class"] | "none";
  float confidence = doc["confidence"] | 0.0;

  Serial.printf(" Mode=%s | Class=%s | Conf=%.2f\n", mode, class_label, confidence);

  // Xử lý các chế độ điều khiển
  if (cmd_ptr) {
    char c = cmd_ptr[0];
    // Chấp nhận các lệnh điều hướng chuẩn
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S' || c == 'A') {
      lastAICommand = c;
      
      // GỬI LỆNH QUA UART2 CHO ARDUINO
      Serial2.write(lastAICommand);
      
      Serial.printf(" [%s] -> Gửi lệnh '%c' tới Arduino\n", mode, lastAICommand);
    }
  }
}

// ===================== WEBSOCKET EVENTS =====================
void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("✅ WebSocket Connected!");
  }
  else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("⚠️ WebSocket Disconnected!");
  }
}

// ===================== KHỞI TẠO CAMERA =====================
void initCamera() {
  camera_config_t config;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed (0x%x)\n", err);
    delay(2000);
    ESP.restart();
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  
  // Khởi tạo UART2 để giao tiếp với Arduino
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  initCamera();

  // Kết nối WiFi sử dụng thông tin từ secrets.h
  WiFi.begin(ssid, password);
  Serial.print("📡 Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n✅ WiFi Connected!");

  // Cấu hình WebSocket
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);

  Serial.println("🌐 Connecting WebSocket...");
  if (!client.connect(server_host, server_port, server_path)) {
    Serial.println("❌ WebSocket Failed! Rebooting in 3s...");
    delay(3000);
    ESP.restart();
  }
}

// ===================== LOOP =====================
void loop() {
  client.poll();

  // Gửi khung hình theo chu kỳ FRAME_INTERVAL (định nghĩa trong config.h)
  if (millis() - lastFrameTime > FRAME_INTERVAL) {
    lastFrameTime = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Camera capture failed");
      return;
    }

    if (client.available()) {
      bool sent = client.sendBinary((const char*)fb->buf, fb->len);
      if (sent) {
        Serial.printf("📤 Frame sent (%d bytes) | AI Cmd: %c\n", fb->len, lastAICommand);
      }
    }

    esp_camera_fb_return(fb);
  }
}