#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <RoboCore_Vespa.h>

// Credenciais da rede (modo AP)
const char* ssid = "Vespa_S1";
const char* password = "12345678";

// WebSocket e servidor
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Servo da Vespa
VespaServo servo;

// Tempo para envio da bateria
unsigned long lastBatteryTime = 0;
const unsigned long batteryInterval = 2000; // 2s

void handleMessage(String msg) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, msg);

  if (err) {
    Serial.println("Erro ao decodificar JSON");
    return;
  }

  // Controle individual: { "servo": 1, "angulo": 90 }
  if (doc.containsKey("servo") && doc.containsKey("angulo")) {
    int s = doc["servo"];
    int a = doc["angulo"];
    servo.move(s, a);
  }

  // Controle conjunto: { "base": 90, "height": 80, "reach": 50, "gripper": 1 }
  if (doc.containsKey("base"))    servo.move(4, doc["base"]);
  if (doc.containsKey("height"))  servo.move(2, doc["height"]);
  if (doc.containsKey("reach"))   servo.move(3, doc["reach"]);
  if (doc.containsKey("gripper")) servo.move(1, doc["gripper"] == 1 ? 180 : 0);
}

void onWebSocketEvent(AsyncWebSocket* server,
                      AsyncWebSocketClient* client,
                      AwsEventType type,
                      void* arg,
                      uint8_t* data,
                      size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String msg = String((char*)data).substring(0, len);
      handleMessage(msg);
    }
  }
}

void setup() {
  Serial.begin(115200);
  servo.begin();

  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP inicializado");

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("WebSocket ativo em /ws");
}

void loop() {
  // Enviar voltagem da bateria a cada 2s
  if (millis() - lastBatteryTime > batteryInterval) {
    lastBatteryTime = millis();
    int raw = analogRead(1); // ADC1
    float vbat = raw * 3.3 / 4095 * 2; // assume divisor 1:2
    StaticJsonDocument<64> doc;
    doc["vbat"] = int(vbat * 1000); // enviar em mV
    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
  }
}
