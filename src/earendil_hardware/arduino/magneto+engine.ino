#include <Wire.h>
#include <math.h>

// =======================================================
// ARC26 Rover Arduino Mega Kodu
// Motorlar + Matek M9N-5883 QMC5883L Manyetometre
//
// Manyetometre hesap k?sm? kullan?c?n?n verdi?i kodla ayn? tutuldu:
// - Kalibrasyon de?erleri ayn?
// - HEADING_CORRECTION_CONSTANT ayn?
// - calculateRawHeading ayn?
// - calculateCorrectedHeading ayn?
//
// Arduino Mega I2C:
// SDA = 20
// SCL = 21
//
// ROS bridge için MAG format?:
// MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,XY,0.00,motor_mode,pwm
// =======================================================


// =======================================================
// MANYETOMETRE - SEN?N KODUNDAN AYNEN ALINAN KISIM
// =======================================================

#define QMC5883L_ADDR 0x0D

// Rover ustunde alinan yeni kalibrasyon degerleri
#define MAG_X_OFFSET -930.500000
#define MAG_Y_OFFSET -646.500000
#define MAG_Z_OFFSET 0.000000

#define MAG_X_SCALE 1.026731
#define MAG_Y_SCALE 0.974626
#define MAG_Z_SCALE 1.000000

// Ilk test icin eski duzeltme sabitini koruyoruz.
// Testten sonra gerekirse bunu degistirecegiz.
#define HEADING_CORRECTION_CONSTANT 243.0

void writeReg(byte reg, byte value) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readRawMag(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(QMC5883L_ADDR, 6);

  if (Wire.available() < 6) {
    return false;
  }

  byte x_lsb = Wire.read();
  byte x_msb = Wire.read();
  byte y_lsb = Wire.read();
  byte y_msb = Wire.read();
  byte z_lsb = Wire.read();
  byte z_msb = Wire.read();

  x = (int16_t)(x_msb << 8 | x_lsb);
  y = (int16_t)(y_msb << 8 | y_lsb);
  z = (int16_t)(z_msb << 8 | z_lsb);

  return true;
}

float normalizeAngle(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

float calculateRawHeading(float x, float y) {
  float rawHeading = atan2(y, x) * 180.0 / PI;
  return normalizeAngle(rawHeading);
}

float calculateCorrectedHeading(float rawHeading) {
  return normalizeAngle(HEADING_CORRECTION_CONSTANT - rawHeading);
}


// =======================================================
// MOTOR PINLERI - BTS7960 / Arduino Mega
// Kendi onceki sistemindeki pinleri ayni tuttum.
// =======================================================

const int L_RPWM = 5;
const int L_LPWM = 6;
const int L_REN  = 7;
const int L_LEN  = 12;

const int R_RPWM = 13;
const int R_LPWM = 10;
const int R_REN  = 11;
const int R_LEN  = 12;


// =======================================================
// AYARLAR
// =======================================================

#define SERIAL_BAUD 115200
#define TELEMETRY_INTERVAL_MS 100
#define MOTOR_WATCHDOG_MS 700


// =======================================================
// DURUM DEGISKENLERI
// =======================================================

String motorMode = "STOP";
int currentMotorPwm = 0;

bool telemetryEnabled = true;

unsigned long lastTelemetryMs = 0;
unsigned long lastMotorCommandMs = 0;


// =======================================================
// MOTOR FONKSIYONLARI
// =======================================================

void motorStop() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  motorMode = "STOP";
  currentMotorPwm = 0;
}

void motorForward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  motorMode = "FWD";
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorBackward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  motorMode = "BACK";
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);

  // Sol tank dönü?: sol motor geri, sa? motor ileri
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  motorMode = "LEFT";
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorRight(int pwm) {
  pwm = constrain(pwm, 0, 255);

  // Sa? tank dönü?: sol motor ileri, sa? motor geri
  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  motorMode = "RIGHT";
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void checkMotorWatchdog() {
  if (motorMode == "STOP") {
    return;
  }

  if (millis() - lastMotorCommandMs > MOTOR_WATCHDOG_MS) {
    motorStop();
    Serial.println("WARN,MOTOR_WATCHDOG_STOP");
  }
}


// =======================================================
// MANYETOMETRE TELEMETRY
// =======================================================

bool readMagAndPrint(bool forcePrint) {
  if (!telemetryEnabled && !forcePrint) {
    return false;
  }

  unsigned long now = millis();

  if (!forcePrint && now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return false;
  }

  lastTelemetryMs = now;

  int16_t rawX, rawY, rawZ;

  if (readRawMag(rawX, rawY, rawZ)) {
    float x = (rawX - MAG_X_OFFSET) * MAG_X_SCALE;
    float y = (rawY - MAG_Y_OFFSET) * MAG_Y_SCALE;
    float z = (rawZ - MAG_Z_OFFSET) * MAG_Z_SCALE;

    float rawHeading = calculateRawHeading(x, y);
    float correctedHeading = calculateCorrectedHeading(rawHeading);

    // ROS bridge'in okuyaca?? ana sat?r.
    // Format:
    // MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm
    Serial.print("MAG,");
    Serial.print(now);
    Serial.print(",");

    Serial.print(correctedHeading, 2);
    Serial.print(",");

    Serial.print(rawX);
    Serial.print(",");
    Serial.print(rawY);
    Serial.print(",");
    Serial.print(rawZ);
    Serial.print(",");

    Serial.print(x, 2);
    Serial.print(",");
    Serial.print(y, 2);
    Serial.print(",");
    Serial.print(z, 2);
    Serial.print(",");

    Serial.print("XY");
    Serial.print(",");

    // Eski -53.5 gibi bridge offsetlerinin karismamasi icin burada sabit 0.00 basiyoruz.
    // Heading zaten HEADING_CORRECTION_CONSTANT ile duzeltiliyor.
    Serial.print("0.00");
    Serial.print(",");

    Serial.print(motorMode);
    Serial.print(",");
    Serial.println(currentMotorPwm);

    return true;
  }
  else {
    Serial.println("QMC5883L okunamadi.");
    return false;
  }
}


// =======================================================
// KOMUTLAR
// =======================================================

int getPwmAfterLastColon(String cmd) {
  int idx = cmd.lastIndexOf(':');
  if (idx < 0) return 0;

  String pwmStr = cmd.substring(idx + 1);
  pwmStr.trim();

  int pwm = pwmStr.toInt();
  return constrain(pwm, 0, 255);
}

void printStatus() {
  Serial.print("STATUS,");
  Serial.print("MAG=MATEK_M9N_5883_QMC5883L");
  Serial.print(",ADDR=0x0D");
  Serial.print(",HEADING_CORRECTION_CONSTANT=");
  Serial.print(HEADING_CORRECTION_CONSTANT, 2);
  Serial.print(",TELEM=");
  Serial.print(telemetryEnabled ? "ON" : "OFF");
  Serial.print(",MOTOR=");
  Serial.print(motorMode);
  Serial.print(",PWM=");
  Serial.println(currentMotorPwm);
}

void printHelp() {
  Serial.println("HELP_BEGIN");
  Serial.println("MOTOR:STOP");
  Serial.println("MOTOR:FWD:<0-255>");
  Serial.println("MOTOR:BACK:<0-255>");
  Serial.println("MOTOR:LEFT:<0-255>");
  Serial.println("MOTOR:RIGHT:<0-255>");
  Serial.println("HEADING");
  Serial.println("STATUS");
  Serial.println("TELEM:ON");
  Serial.println("TELEM:OFF");
  Serial.println("PING");
  Serial.println("HELP_END");
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd == "HELP") {
    printHelp();
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  if (cmd == "HEADING") {
    readMagAndPrint(true);
    return;
  }

  if (cmd == "TELEM:ON") {
    telemetryEnabled = true;
    Serial.println("ACK,TELEM,ON");
    return;
  }

  if (cmd == "TELEM:OFF") {
    telemetryEnabled = false;
    Serial.println("ACK,TELEM,OFF");
    return;
  }

  // Bridge PLANE komutu gonderirse kabul ediyoruz ama manyetometre hesabini degistirmiyoruz.
  if (cmd == "PLANE:XY" || cmd == "PLANE:XZ" || cmd == "PLANE:YZ") {
    Serial.println("ACK,PLANE,XY_FIXED");
    return;
  }

  // Bridge OFFSET komutu gonderirse kabul ediyoruz ama heading'e eklemiyoruz.
  // Eski -53.5 offsetin tekrar karismamasi icin bilerek etkisiz.
  if (cmd.startsWith("OFFSET:")) {
    Serial.println("ACK,OFFSET,IGNORED_THIS_CODE_USES_HEADING_CORRECTION_CONSTANT");
    return;
  }

  if (cmd == "MOTOR:STOP") {
    motorStop();
    lastMotorCommandMs = millis();
    Serial.println("ACK,MOTOR,STOP");
    return;
  }

  if (cmd.startsWith("MOTOR:FWD:")) {
    int pwm = getPwmAfterLastColon(cmd);
    motorForward(pwm);
    Serial.print("ACK,MOTOR,FWD,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:BACK:")) {
    int pwm = getPwmAfterLastColon(cmd);
    motorBackward(pwm);
    Serial.print("ACK,MOTOR,BACK,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:LEFT:")) {
    int pwm = getPwmAfterLastColon(cmd);
    motorLeft(pwm);
    Serial.print("ACK,MOTOR,LEFT,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:RIGHT:")) {
    int pwm = getPwmAfterLastColon(cmd);
    motorRight(pwm);
    Serial.print("ACK,MOTOR,RIGHT,");
    Serial.println(pwm);
    return;
  }

  // Eski manuel komutlar
  if (cmd == "dur") {
    motorStop();
    Serial.println("ACK,MOTOR,STOP");
    return;
  }

  if (cmd == "ileri_yavas") {
    motorForward(80);
    Serial.println("ACK,MOTOR,FWD,80");
    return;
  }

  if (cmd == "ileri_hizli") {
    motorForward(200);
    Serial.println("ACK,MOTOR,FWD,200");
    return;
  }

  if (cmd == "geri_yavas") {
    motorBackward(80);
    Serial.println("ACK,MOTOR,BACK,80");
    return;
  }

  if (cmd == "geri_hizli") {
    motorBackward(200);
    Serial.println("ACK,MOTOR,BACK,200");
    return;
  }

  if (cmd == "sol_yavas") {
    motorLeft(80);
    Serial.println("ACK,MOTOR,LEFT,80");
    return;
  }

  if (cmd == "sol_hizli") {
    motorLeft(200);
    Serial.println("ACK,MOTOR,LEFT,200");
    return;
  }

  if (cmd == "sag_yavas") {
    motorRight(80);
    Serial.println("ACK,MOTOR,RIGHT,80");
    return;
  }

  if (cmd == "sag_hizli") {
    motorRight(200);
    Serial.println("ACK,MOTOR,RIGHT,200");
    return;
  }

  Serial.print("WARN,UNKNOWN_CMD,");
  Serial.println(cmd);
}

void handleSerial() {
  static String line = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processCommand(line);
        line = "";
      }
    }
    else {
      line += c;

      if (line.length() > 120) {
        line = "";
        Serial.println("WARN,SERIAL_LINE_TOO_LONG");
      }
    }
  }
}


// =======================================================
// SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();

  delay(500);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  motorStop();

  Serial.println("ARC26 Rover motor + Matek M9N-5883 heading kodu basladi.");
  Serial.println("Matek M9N-5883 rover ustu heading testi entegre edildi.");

  writeReg(0x0B, 0x01);
  writeReg(0x09, 0x1D);

  delay(200);

  printHelp();

  lastTelemetryMs = millis();
  lastMotorCommandMs = millis();
}

void loop() {
  handleSerial();
  checkMotorWatchdog();
  readMagAndPrint(false);
}