/*
  Runway Alignment Guidance System - CENTRE controller

  PDF wiring (OLED intentionally omitted):
    HC-SR04: TRIG GPIO 5, ECHO GPIO 14 through a divider
    Green LED: GPIO 16
    Amber LEFT LED: GPIO 17
    Amber RIGHT LED: GPIO 21 (OLED is not used)
    Red LED: GPIO 18
    Buzzer: GPIO 19

  The controller accepts both packet formats used in this project:
    PDF:       RSSI,A101,L,-45
    Workspace: R,LEFT,-45.0
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- Set these to the same 2.4 GHz hotspot on every node ----------
const char* WIFI_SSID = "moto g54 5G_2518";
const char* WIFI_PASSWORD = "ayush123";
const char* AIRCRAFT_ID = "A101";
const char* PI_IP = "10.193.222.79";
// --------------------------------------------------------------------------

constexpr uint16_t PDF_UDP_PORT = 8888;
constexpr uint16_t WORKSPACE_UDP_PORT = 4210;
constexpr uint16_t PI_UDP_PORT = 8889;
constexpr unsigned long NODE_TIMEOUT_MS = 2500;

// PDF pinout. The PDF assumes anode -> GPIO -> resistor -> LED cathode -> GND.
constexpr uint8_t TRIG_PIN = 5;
constexpr uint8_t ECHO_PIN = 14;
constexpr uint8_t LED_GREEN = 16;
constexpr uint8_t LED_AMBER_LEFT = 17;
constexpr uint8_t LED_RED = 18;
constexpr uint8_t BUZZER_PIN = 19;
constexpr uint8_t LED_AMBER_RIGHT = 21;
constexpr bool LED_ACTIVE_HIGH = true;

constexpr int DRIFT_TRIGGER_DB = 6;
constexpr int CENTER_CLEAR_DB = 3;

WiFiUDP pdfUdp;
WiFiUDP workspaceUdp;
WiFiUDP telemetryUdp;

int rssiLeft = -127;
int rssiCenter = -127;
int rssiRight = -127;
unsigned long lastLeftUpdate = 0;
unsigned long lastRightUpdate = 0;
unsigned long ignoredPackets = 0;
unsigned long lastPacketWarning = 0;

String alignmentState = "WAITING";
String verticalState = "UNKNOWN";

void writeLed(uint8_t pin, bool on) {
  const bool level = LED_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

void allLedsOff() {
  writeLed(LED_GREEN, false);
  writeLed(LED_AMBER_LEFT, false);
  writeLed(LED_AMBER_RIGHT, false);
  writeLed(LED_RED, false);
}

void stopBuzzer() {
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

void testIndicators() {
  // This separates a GPIO/wiring problem from an alignment problem.
  stopBuzzer();
  allLedsOff();
  writeLed(LED_GREEN, true);
  delay(250);
  writeLed(LED_GREEN, false);
  writeLed(LED_AMBER_LEFT, true);
  delay(250);
  writeLed(LED_AMBER_LEFT, false);
  writeLed(LED_AMBER_RIGHT, true);
  delay(250);
  writeLed(LED_AMBER_RIGHT, false);
  writeLed(LED_RED, true);
  delay(250);
  allLedsOff();
  Serial.println("[TEST] LEDs: GREEN -> AMBER_LEFT -> AMBER_RIGHT -> RED");
}

void setIndicators() {
  allLedsOff();
  stopBuzzer();

  if (alignmentState == "DEGRADED") {
    writeLed(LED_RED, true);
    tone(BUZZER_PIN, 400, 100);
  } else if (alignmentState == "CENTERLINE") {
    writeLed(LED_GREEN, true);
  } else if (alignmentState == "CORRECT RIGHT") {
    // Aircraft is too close to the left side; correction is to the right.
    writeLed(LED_AMBER_LEFT, true);
    tone(BUZZER_PIN, 1400, 50);
  } else if (alignmentState == "CORRECT LEFT") {
    // Aircraft is too close to the right side; correction is to the left.
    writeLed(LED_AMBER_RIGHT, true);
    tone(BUZZER_PIN, 1000, 50);
  } else {
    // WAITING/acquiring: both amber LEDs show that alignment is not ready.
    writeLed(LED_AMBER_LEFT, true);
    writeLed(LED_AMBER_RIGHT, true);
  }
}

int getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;
  return static_cast<int>(duration * 0.0343F / 2.0F);
}

void evaluateVertical() {
  const int distance = getDistanceCM();
  if (distance < 0) verticalState = "NO ECHO";
  else if (distance < 10) verticalState = "TOO LOW";
  else if (distance > 30) verticalState = "TOO HIGH";
  else verticalState = "ON GLIDE";
}

void evaluateAlignment() {
  const unsigned long now = millis();
  const bool leftOnline = now - lastLeftUpdate <= NODE_TIMEOUT_MS;
  const bool rightOnline = now - lastRightUpdate <= NODE_TIMEOUT_MS;

  if (!leftOnline || !rightOnline || rssiCenter == -127) {
    alignmentState = "DEGRADED";
    setIndicators();
    return;
  }

  // Larger values mean a stronger signal. A positive diff means that side
  // is stronger than the centre, so the correction is toward the opposite side.
  const int diffL = rssiLeft - rssiCenter;
  const int diffR = rssiRight - rssiCenter;

  if (alignmentState == "CORRECT RIGHT") {
    if (!(diffL > CENTER_CLEAR_DB && diffL >= diffR)) {
      alignmentState = (diffR >= DRIFT_TRIGGER_DB && diffR > diffL)
                         ? "CORRECT LEFT" : "CENTERLINE";
    }
  } else if (alignmentState == "CORRECT LEFT") {
    if (!(diffR > CENTER_CLEAR_DB && diffR >= diffL)) {
      alignmentState = (diffL >= DRIFT_TRIGGER_DB && diffL > diffR)
                         ? "CORRECT RIGHT" : "CENTERLINE";
    }
  } else if (diffL >= DRIFT_TRIGGER_DB && diffL > diffR) {
    alignmentState = "CORRECT RIGHT";
  } else if (diffR >= DRIFT_TRIGGER_DB && diffR > diffL) {
    alignmentState = "CORRECT LEFT";
  } else {
    alignmentState = "CENTERLINE";
  }

  setIndicators();
}

bool parseMessage(const String& message, String& node, float& value) {
  String msg = message;
  msg.trim();

  // PDF format: RSSI,A101,L,-45
  if (msg.startsWith("RSSI,")) {
    const int c1 = msg.indexOf(',');
    const int c2 = msg.indexOf(',', c1 + 1);
    const int c3 = msg.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) return false;
    if (msg.substring(c1 + 1, c2) != AIRCRAFT_ID) return false;
    node = msg.substring(c2 + 1, c3);
    value = msg.substring(c3 + 1).toFloat();
  }
  // Workspace format: R,LEFT,-45.0
  else if (msg.startsWith("R,")) {
    const int c1 = msg.indexOf(',');
    const int c2 = msg.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return false;
    const String sourceNode = msg.substring(c1 + 1, c2);
    if (sourceNode == "LEFT") node = "L";
    else if (sourceNode == "RIGHT") node = "R";
    else return false;
    value = msg.substring(c2 + 1).toFloat();
  } else {
    return false;
  }

  // RSSI is dBm and must be in the valid negative range. Reject malformed
  // toFloat() results instead of treating them as a real 0 dBm packet.
  if (value >= 0.0F || value < -127.0F) return false;
  if (node != "L" && node != "R") return false;
  return true;
}

void processSocket(WiFiUDP& socket, const char* socketName) {
  int packetSize;
  while ((packetSize = socket.parsePacket()) > 0) {
    char buffer[80];
    const int length = socket.read(buffer, sizeof(buffer) - 1);
    if (length <= 0) continue;
    buffer[length] = '\0';

    String node;
    float value;
    if (!parseMessage(String(buffer), node, value)) {
      ++ignoredPackets;
      if (millis() - lastPacketWarning > 1000) {
        Serial.printf("[UDP] Ignored on %s: %s\n", socketName, buffer);
        lastPacketWarning = millis();
      }
      continue;
    }

    const int integerValue = static_cast<int>(value + (value < 0 ? -0.5F : 0.5F));
    if (node == "L") {
      rssiLeft = integerValue;
      lastLeftUpdate = millis();
    } else {
      rssiRight = integerValue;
      lastRightUpdate = millis();
    }
  }
}

void processUDP() {
  processSocket(pdfUdp, "8888");
  processSocket(workspaceUdp, "4210");
}

void sampleCenterRSSI() {
  long sum = 0;
  int validSamples = 0;
  for (int i = 0; i < 5; ++i) {
    const int sample = WiFi.RSSI();
    if (sample <= 0 && sample >= -127) {
      sum += sample;
      ++validSamples;
    }
    delay(5);
  }
  rssiCenter = validSamples == 0 ? -127 : static_cast<int>(sum / validSamples);
}

void sendTelemetry() {
  const String telemetry = String("ILS,") + AIRCRAFT_ID + "," +
                           String(rssiLeft) + "," + String(rssiCenter) + "," +
                           String(rssiRight) + "," + alignmentState + "," +
                           verticalState;
  telemetryUdp.beginPacket(PI_IP, PI_UDP_PORT);
  telemetryUdp.print(telemetry);
  telemetryUdp.endPacket();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("[WIFI] Centre IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WIFI] Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("[WIFI] RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_AMBER_LEFT, OUTPUT);
  pinMode(LED_AMBER_RIGHT, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  allLedsOff();
  stopBuzzer();
  testIndicators();

  connectWiFi();
  pdfUdp.begin(PDF_UDP_PORT);
  workspaceUdp.begin(WORKSPACE_UDP_PORT);

  Serial.printf("[UDP] Listening on %u (PDF) and %u (workspace)\n",
                PDF_UDP_PORT, WORKSPACE_UDP_PORT);
  Serial.printf("[UDP] Accepted: RSSI,%s,L/R,value and R,LEFT/RIGHT,value\n",
                AIRCRAFT_ID);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    alignmentState = "DEGRADED";
    setIndicators();
    delay(250);
    return;
  }

  sampleCenterRSSI();
  processUDP();
  evaluateAlignment();
  evaluateVertical();
  sendTelemetry();

  Serial.printf("L:%4d | C:%4d | R:%4d | ALIGN:%-13s | GLIDE:%s | ignored:%lu\n",
                rssiLeft, rssiCenter, rssiRight, alignmentState.c_str(),
                verticalState.c_str(), ignoredPackets);
  delay(80);
}