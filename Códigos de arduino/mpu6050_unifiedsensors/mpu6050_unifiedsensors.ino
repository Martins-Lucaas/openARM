#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;
WebSocketsClient webSocket;

// =======================
// AJUSTES FÁCEIS
// =======================
const char* ssid = "Vespa-0D:71";
const char* password = "12345678";

const float DEADZONE = 3.0;

const float ROLL_MIN = -45;
const float ROLL_MAX = 45;
const float PITCH_MIN = -30;
const float PITCH_MAX = 30;
const float YAW_MIN = -60;
const float YAW_MAX = 60;
const float ANGLE_MIN = 0;
const float ANGLE_MAX = 180;

// LED RGB
#define LED_R 9
#define LED_G 10
#define LED_B 8
void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}

// EMG
const int EMG_PIN = 2;
const int LIMIAR_FECHAMENTO = 600;
const int LIMIAR_ABERTURA = 300;
const unsigned long DEAD_TIME_MS = 1500;

bool garraFechada = false;
unsigned long ultimoComandoEMG = 0;

// =======================
// Kalman
class Kalman {
public:
  Kalman() {
    Q_angle = 0.001;
    Q_bias  = 0.003;
    R_measure = 0.03;
    angle = 0;
    bias = 0;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
  }

  double getAngle(double newAngle, double newRate, double dt) {
    double rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    double S = P[0][0] + R_measure;
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;

    double y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;

    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }

private:
  double Q_angle, Q_bias, R_measure;
  double angle, bias;
  double P[2][2];
};

Kalman kalmanX;
Kalman kalmanY;

unsigned long previousTime = 0;
double yawAngle = 0;
double initialRoll = 0;
double initialPitch = 0;
double lastRollRelative = 0;
double horizontalAngle = 90.0;

float mapAngle(float value, float inMin, float inMax, float outMin, float outMax, float deadZone) {
  if (abs(value) < deadZone) return (outMin + outMax) / 2;
  value = constrain(value, inMin, inMax);
  return map(value * 100, inMin * 100, inMax * 100, outMin * 100, outMax * 100) / 100.0;
}

// =============== WEBSOCKET CALLBACK ====================
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("WebSocket conectado.");
      setLED(false, true, false);  // verde
      break;
    case WStype_DISCONNECTED:
      Serial.println("WebSocket desconectado.");
      setLED(true, true, false);   // amarelo
      break;
    case WStype_TEXT:
      Serial.printf("Recebido: %s\n", payload);
      break;
    case WStype_ERROR:
      Serial.println("Erro no WebSocket.");
      break;
    default:
      break;
  }
}

// =======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(EMG_PIN, INPUT);

  setLED(true, false, false);  // vermelho

  WiFi.begin(ssid, password);
  Serial.print("Conectando à rede WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  setLED(true, true, false); // amarelo

  webSocket.begin("192.168.4.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Erro ao conectar com o MPU6050!");
    setLED(true, false, false);
    return;
  }

  Serial.println("Calibrando posição inicial...");
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double AccX = ax / 16384.0;
    double AccY = ay / 16384.0;
    double AccZ = az / 16384.0;

    initialRoll  += atan2(AccY, AccZ) * 180.0 / PI;
    initialPitch += atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

    delay(5);
  }
  initialRoll /= 100.0;
  initialPitch /= 100.0;
  previousTime = millis();
}

// =======================================================
void loop() {
  webSocket.loop();

  unsigned long currentTime = millis();
  double dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double AccX = ax / 16384.0;
  double AccY = ay / 16384.0;
  double AccZ = az / 16384.0;

  double GyroX = gx / 131.0;
  double GyroY = gy / 131.0;
  double GyroZ = gz / 131.0;

  double rollAcc  = atan2(AccY, AccZ) * 180.0 / PI;
  double pitchAcc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  double roll  = kalmanX.getAngle(rollAcc, GyroX, dt);
  double pitch = kalmanY.getAngle(pitchAcc, GyroY, dt);
  yawAngle += GyroZ * dt;

  double rollRelative = roll - initialRoll;
  double pitchRelative = pitch - initialPitch;
  double deltaRoll = rollRelative - lastRollRelative;
  lastRollRelative = rollRelative;

  horizontalAngle += deltaRoll;
  horizontalAngle = constrain(horizontalAngle, ANGLE_MIN, ANGLE_MAX);

  float base     = mapAngle(rollRelative, ROLL_MIN, ROLL_MAX, ANGLE_MIN, ANGLE_MAX, DEADZONE);
  float altura   = mapAngle(pitchRelative, PITCH_MIN, PITCH_MAX, 60, 120, DEADZONE);
  float distancia = mapAngle(yawAngle, YAW_MIN, YAW_MAX, 60, 120, 0);

  sendServoCommand(4, base);
  sendServoCommand(2, altura);
  sendServoCommand(3, distancia);

  // === EMG para abrir/fechar garra (servo 1) ===
  int emgValor = analogRead(EMG_PIN);
  unsigned long agora = millis();

  if (agora - ultimoComandoEMG > DEAD_TIME_MS) {
    if (!garraFechada && emgValor > LIMIAR_FECHAMENTO) {
      sendServoCommand(1, 180);  // fecha garra
      garraFechada = true;
      ultimoComandoEMG = agora;
      Serial.println("EMG: Fechando garra");
    } else if (garraFechada && emgValor < LIMIAR_ABERTURA) {
      sendServoCommand(1, 0);    // abre garra
      garraFechada = false;
      ultimoComandoEMG = agora;
      Serial.println("EMG: Abrindo garra");
    }
  }

  Serial.print("EMG: ");
  Serial.print(emgValor);
  Serial.print(" | Base: ");
  Serial.print(base);
  Serial.print(" | Altura: ");
  Serial.print(altura);
  Serial.print(" | Distância: ");
  Serial.println(distancia);

  delay(100);
}

void sendServoCommand(uint8_t servo, float angle) {
  String json = "{\"servo\":" + String(servo) + ",\"angulo\":" + String((int)angle) + "}";
  webSocket.sendTXT(json);
}
