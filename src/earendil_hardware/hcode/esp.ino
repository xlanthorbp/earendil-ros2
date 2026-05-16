#include <WiFi.h>

const char* ssid = "necati";
const char* password = "12345678";

const char* host = "10.19.62.135";
const uint16_t port = 8888;

WiFiClient client;

// ===================================================
// JOYSTICK AKTIF / PASIF AYARLARI
// ===================================================
// Bagli olmayan joysticki false yap.
// false olan joystick Raspberry'ye 2048,2048 olarak gider.

const bool USE_JOY1 = false;  // JOY1: rover surus
const bool USE_JOY2 = true ; // JOY2: robot kol servo2 + servo3
const bool USE_JOY3 = false;  // JOY3: robot kol servo1 + servo4

const int JOY_CENTER = 2048;

// ===================================================
// Joystick pinleri
// ===================================================

const int VRx1 = 32;
const int VRy1 = 33;

const int VRx2 = 34;
const int VRy2 = 35;

const int VRx3 = 36;
const int VRy3 = 39;

// ===================================================
// Buton pinleri
// ===================================================

const int btn4   = 4;
const int btn18  = 18;
const int btn19  = 19;
const int btn21  = 21;
const int btn27  = 27;

const int ledPin = 2;

unsigned long lastReconnectTry = 0;
unsigned long lastSendTime = 0;

const unsigned long SEND_INTERVAL_MS = 50;   // 20 Hz

int readAxis(int pin, bool enabled) {
  if (!enabled) {
    return JOY_CENTER;
  }

  return analogRead(pin);
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WiFi baglaniyor...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(ledPin, !digitalRead(ledPin));
  }

  Serial.println();
  Serial.println("WiFi baglandi.");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  digitalWrite(ledPin, LOW);
}

void connectServer() {
  if (client.connected()) return;

  Serial.print("Raspberry server'a baglaniliyor: ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  if (client.connect(host, port)) {
    Serial.println("Raspberry'ye baglandi.");
    client.setNoDelay(true);
    client.println("ESP32_JOYSTICK_CONNECTED");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("Raspberry baglantisi basarisiz.");
    digitalWrite(ledPin, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn18, INPUT_PULLUP);
  pinMode(btn19, INPUT_PULLUP);
  pinMode(btn21, INPUT_PULLUP);
  pinMode(btn27, INPUT_PULLUP);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  analogReadResolution(12);      // 0 - 4095
  analogSetAttenuation(ADC_11db);

  Serial.println();
  Serial.println("ESP32 joystick kumandasi basliyor...");
  Serial.println("Joystick aktiflik ayarlari:");
  Serial.print("JOY1: "); Serial.println(USE_JOY1 ? "AKTIF" : "PASIF");
  Serial.print("JOY2: "); Serial.println(USE_JOY2 ? "AKTIF" : "PASIF");
  Serial.print("JOY3: "); Serial.println(USE_JOY3 ? "AKTIF" : "PASIF");
  Serial.println();

  connectWiFi();
  connectServer();

  Serial.println("ESP32 hazir.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!client.connected()) {
    unsigned long now = millis();

    if (now - lastReconnectTry > 3000) {
      lastReconnectTry = now;
      connectServer();
    }
  }

  unsigned long now = millis();

  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;

    int x1 = readAxis(VRx1, USE_JOY1);
    int y1 = readAxis(VRy1, USE_JOY1);

    int x2 = readAxis(VRx2, USE_JOY2);
    int y2 = readAxis(VRy2, USE_JOY2);

    int x3 = readAxis(VRx3, USE_JOY3);
    int y3 = readAxis(VRy3, USE_JOY3);

    int b4  = digitalRead(btn4)  == LOW ? 1 : 0;
    int b18 = digitalRead(btn18) == LOW ? 1 : 0;
    int b19 = digitalRead(btn19) == LOW ? 1 : 0;
    int b21 = digitalRead(btn21) == LOW ? 1 : 0;
    int b27 = digitalRead(btn27) == LOW ? 1 : 0;

    char data[160];

    snprintf(data, sizeof(data),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             x1, y1, x2, y2, x3, y3, b4, b18, b19, b21, b27);

    if (client.connected()) {
      client.println(data);
      Serial.println(data);
    } else {
      Serial.println("Hata: Raspberry'ye bagli degil.");
    }
  }
}