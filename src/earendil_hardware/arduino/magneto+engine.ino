#include <Wire.h>
#include <math.h>

// =======================================================
// SERIAL
// =======================================================
#define SERIAL_BAUD 115200

// =======================================================
// GY-271 ADRESLERİ
// =======================================================
#define QMC5883_ADDR 0x0D
#define HMC5883_ADDR 0x1E

// =======================================================
// GY-271 KALİBRASYON DEĞERLERİ
// =======================================================
#define MAG_X_OFFSET -1264.50
#define MAG_Y_OFFSET 311.00
#define MAG_Z_OFFSET -73.50

#define MAG_X_SCALE 0.81943
#define MAG_Y_SCALE 0.70439
#define MAG_Z_SCALE 2.78393

#define MAG_SAMPLE_COUNT 10
#define TELEMETRY_INTERVAL_MS 200

// 0 = XY, 1 = XZ, 2 = YZ
#define DEFAULT_HEADING_PLANE 0

// Rover burnuna göre pusula offset'i.
// Bunu ROS2 tarafında da düzeltebiliriz ama Arduino telemetry için burada da tutuyoruz.
#define DEFAULT_HEADING_OFFSET_DEG 0.0

// =======================================================
// MOTOR PİNLERİ - BTS7960
// =======================================================
const int L_RPWM = 5;
const int L_LPWM = 6;
const int L_REN  = 7;
const int L_LEN  = 8;

const int R_RPWM = 9;
const int R_LPWM = 10;
const int R_REN  = 11;
const int R_LEN  = 12;

// =======================================================
// GÜVENLİK
// =======================================================
// ROS2/Serial komutu kesilirse motoru durdurur.
#define MOTOR_WATCHDOG_MS 700

// =======================================================
// MAGNETOMETER STATE
// =======================================================
enum MagType {
  MAG_NONE,
  MAG_QMC5883L,
  MAG_HMC5883L
};

MagType magType = MAG_NONE;

uint8_t activeHeadingPlane = DEFAULT_HEADING_PLANE;
float headingOffsetDeg = DEFAULT_HEADING_OFFSET_DEG;

// =======================================================
// MOTOR STATE
// =======================================================
enum MotorMode {
  MOTOR_STOPPED,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_LEFT,
  MOTOR_RIGHT
};

MotorMode currentMotorMode = MOTOR_STOPPED;
int currentMotorPwm = 0;

unsigned long lastMotorCommandMs = 0;
unsigned long lastTelemetryMs = 0;

bool telemetryEnabled = true;

// =======================================================
// YARDIMCI FONKSİYONLAR
// =======================================================
float normalizeHeading(float h) {
  while (h >= 360.0) h -= 360.0;
  while (h < 0.0) h += 360.0;
  return h;
}

const char* planeName(uint8_t plane) {
  if (plane == 0) return "XY";
  if (plane == 1) return "XZ";
  return "YZ";
}

const char* motorModeName(MotorMode mode) {
  if (mode == MOTOR_FORWARD) return "FORWARD";
  if (mode == MOTOR_BACKWARD) return "BACKWARD";
  if (mode == MOTOR_LEFT) return "LEFT";
  if (mode == MOTOR_RIGHT) return "RIGHT";
  return "STOP";
}

// =======================================================
// I2C / GY-271
// =======================================================
bool i2cExists(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  Wire.requestFrom(addr, count);

  uint8_t i = 0;
  while (Wire.available() && i < count) {
    dest[i++] = Wire.read();
  }

  return i;
}

bool initMagnetometer() {
  if (i2cExists(QMC5883_ADDR)) {
    magType = MAG_QMC5883L;

    Serial.println("ACK,MAG,QMC5883L,0x0D");

    // QMC5883L: set/reset period
    write8(QMC5883_ADDR, 0x0B, 0x01);

    // QMC5883L control:
    // 0x1D = continuous mode, 200Hz, 8G, 512 OSR
    write8(QMC5883_ADDR, 0x09, 0x1D);

    return true;
  }

  if (i2cExists(HMC5883_ADDR)) {
    magType = MAG_HMC5883L;

    Serial.println("ACK,MAG,HMC5883L,0x1E");

    write8(HMC5883_ADDR, 0x00, 0x70);
    write8(HMC5883_ADDR, 0x01, 0x20);
    write8(HMC5883_ADDR, 0x02, 0x00);

    return true;
  }

  magType = MAG_NONE;
  Serial.println("ERR,MAG_NOT_FOUND");
  return false;
}

bool readRawMagnetometer(int16_t &rawX, int16_t &rawY, int16_t &rawZ) {
  if (magType == MAG_NONE) return false;

  if (magType == MAG_QMC5883L) {
    uint8_t b[6] = {0};

    if (readBytes(QMC5883_ADDR, 0x00, 6, b) < 6) {
      return false;
    }

    rawX = ((int16_t)b[1] << 8) | b[0];
    rawY = ((int16_t)b[3] << 8) | b[2];
    rawZ = ((int16_t)b[5] << 8) | b[4];

    return true;
  }

  if (magType == MAG_HMC5883L) {
    uint8_t b[6] = {0};

    if (readBytes(HMC5883_ADDR, 0x03, 6, b) < 6) {
      return false;
    }

    rawX = ((int16_t)b[0] << 8) | b[1];
    rawZ = ((int16_t)b[2] << 8) | b[3];
    rawY = ((int16_t)b[4] << 8) | b[5];

    return true;
  }

  return false;
}

bool readCalibratedMag(float &rawXAvg, float &rawYAvg, float &rawZAvg,
                       float &calX, float &calY, float &calZ) {
  long sx = 0;
  long sy = 0;
  long sz = 0;
  int validCount = 0;

  for (int i = 0; i < MAG_SAMPLE_COUNT; i++) {
    int16_t x, y, z;

    if (readRawMagnetometer(x, y, z)) {
      sx += x;
      sy += y;
      sz += z;
      validCount++;
    }

    delay(3);
  }

  if (validCount == 0) {
    return false;
  }

  rawXAvg = sx / (float)validCount;
  rawYAvg = sy / (float)validCount;
  rawZAvg = sz / (float)validCount;

  calX = (rawXAvg - MAG_X_OFFSET) * MAG_X_SCALE;
  calY = (rawYAvg - MAG_Y_OFFSET) * MAG_Y_SCALE;
  calZ = (rawZAvg - MAG_Z_OFFSET) * MAG_Z_SCALE;

  return true;
}

float calculateHeadingDeg(float calX, float calY, float calZ) {
  float heading = 0.0;

  if (activeHeadingPlane == 0) {
    heading = atan2(calY, calX) * 180.0 / PI;
  }
  else if (activeHeadingPlane == 1) {
    heading = atan2(calZ, calX) * 180.0 / PI;
  }
  else {
    heading = atan2(calZ, calY) * 180.0 / PI;
  }

  heading = normalizeHeading(heading + headingOffsetDeg);
  return heading;
}

// =======================================================
// MOTOR FONKSİYONLARI
// =======================================================
void motorStop() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  currentMotorMode = MOTOR_STOPPED;
  currentMotorPwm = 0;
}

void motorForward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  currentMotorMode = MOTOR_FORWARD;
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorBackward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  currentMotorMode = MOTOR_BACKWARD;
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorTurnLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);

  // Tank dönüşü sol:
  // Sol motor geri, sağ motor ileri.
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  currentMotorMode = MOTOR_LEFT;
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void motorTurnRight(int pwm) {
  pwm = constrain(pwm, 0, 255);

  // Tank dönüşü sağ:
  // Sol motor ileri, sağ motor geri.
  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  currentMotorMode = MOTOR_RIGHT;
  currentMotorPwm = pwm;
  lastMotorCommandMs = millis();
}

void checkMotorWatchdog() {
  if (currentMotorMode == MOTOR_STOPPED) {
    return;
  }

  if (millis() - lastMotorCommandMs > MOTOR_WATCHDOG_MS) {
    motorStop();
    Serial.println("WARN,MOTOR_WATCHDOG_STOP");
  }
}

// =======================================================
// TELEMETRY
// =======================================================
void publishMagTelemetry(bool forcePrint) {
  if (!telemetryEnabled && !forcePrint) {
    return;
  }

  unsigned long now = millis();

  if (!forcePrint && now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = now;

  float rawX, rawY, rawZ;
  float calX, calY, calZ;

  if (!readCalibratedMag(rawX, rawY, rawZ, calX, calY, calZ)) {
    Serial.println("ERR,MAG_READ_FAIL");
    return;
  }

  float heading = calculateHeadingDeg(calX, calY, calZ);

  // Format:
  // MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm
  Serial.print("MAG,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(heading, 2);
  Serial.print(",");
  Serial.print(rawX, 0);
  Serial.print(",");
  Serial.print(rawY, 0);
  Serial.print(",");
  Serial.print(rawZ, 0);
  Serial.print(",");
  Serial.print(calX, 2);
  Serial.print(",");
  Serial.print(calY, 2);
  Serial.print(",");
  Serial.print(calZ, 2);
  Serial.print(",");
  Serial.print(planeName(activeHeadingPlane));
  Serial.print(",");
  Serial.print(headingOffsetDeg, 2);
  Serial.print(",");
  Serial.print(motorModeName(currentMotorMode));
  Serial.print(",");
  Serial.print(currentMotorPwm);
  Serial.println();
}

void publishStatus() {
  Serial.print("STATUS,");
  Serial.print("MAG=");

  if (magType == MAG_QMC5883L) {
    Serial.print("QMC5883L");
  }
  else if (magType == MAG_HMC5883L) {
    Serial.print("HMC5883L");
  }
  else {
    Serial.print("NONE");
  }

  Serial.print(",PLANE=");
  Serial.print(planeName(activeHeadingPlane));

  Serial.print(",OFFSET=");
  Serial.print(headingOffsetDeg, 2);

  Serial.print(",MOTOR=");
  Serial.print(motorModeName(currentMotorMode));

  Serial.print(",PWM=");
  Serial.print(currentMotorPwm);

  Serial.print(",TELEM=");
  Serial.print(telemetryEnabled ? "ON" : "OFF");

  Serial.println();
}

// =======================================================
// SERIAL PROTOCOL
// =======================================================
int parsePwmFromCommand(String cmd, int lastColonIndex) {
  String pwmStr = cmd.substring(lastColonIndex + 1);
  pwmStr.trim();

  int pwm = pwmStr.toInt();
  return constrain(pwm, 0, 255);
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
  Serial.println("PLANE:XY");
  Serial.println("PLANE:XZ");
  Serial.println("PLANE:YZ");
  Serial.println("OFFSET:<deg>");
  Serial.println("PING");
  Serial.println("HELP_END");
}

void processSerialCommand(String cmd) {
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
    publishStatus();
    return;
  }

  if (cmd == "HEADING") {
    publishMagTelemetry(true);
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

  if (cmd == "PLANE:XY") {
    activeHeadingPlane = 0;
    Serial.println("ACK,PLANE,XY");
    return;
  }

  if (cmd == "PLANE:XZ") {
    activeHeadingPlane = 1;
    Serial.println("ACK,PLANE,XZ");
    return;
  }

  if (cmd == "PLANE:YZ") {
    activeHeadingPlane = 2;
    Serial.println("ACK,PLANE,YZ");
    return;
  }

  if (cmd.startsWith("OFFSET:")) {
    headingOffsetDeg = cmd.substring(7).toFloat();

    Serial.print("ACK,OFFSET,");
    Serial.println(headingOffsetDeg, 2);
    return;
  }

  // Yeni ROS2 serial protocol
  if (cmd == "MOTOR:STOP") {
    motorStop();
    Serial.println("ACK,MOTOR,STOP");
    return;
  }

  if (cmd.startsWith("MOTOR:FWD:")) {
    int pwm = parsePwmFromCommand(cmd, cmd.lastIndexOf(':'));
    motorForward(pwm);

    Serial.print("ACK,MOTOR,FWD,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:BACK:")) {
    int pwm = parsePwmFromCommand(cmd, cmd.lastIndexOf(':'));
    motorBackward(pwm);

    Serial.print("ACK,MOTOR,BACK,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:LEFT:")) {
    int pwm = parsePwmFromCommand(cmd, cmd.lastIndexOf(':'));
    motorTurnLeft(pwm);

    Serial.print("ACK,MOTOR,LEFT,");
    Serial.println(pwm);
    return;
  }

  if (cmd.startsWith("MOTOR:RIGHT:")) {
    int pwm = parsePwmFromCommand(cmd, cmd.lastIndexOf(':'));
    motorTurnRight(pwm);

    Serial.print("ACK,MOTOR,RIGHT,");
    Serial.println(pwm);
    return;
  }

  // Eski manuel komutlarla uyumluluk.
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
    motorTurnLeft(80);
    Serial.println("ACK,MOTOR,LEFT,80");
    return;
  }

  if (cmd == "sol_hizli") {
    motorTurnLeft(200);
    Serial.println("ACK,MOTOR,LEFT,200");
    return;
  }

  if (cmd == "sag_yavas") {
    motorTurnRight(80);
    Serial.println("ACK,MOTOR,RIGHT,80");
    return;
  }

  if (cmd == "sag_hizli") {
    motorTurnRight(200);
    Serial.println("ACK,MOTOR,RIGHT,200");
    return;
  }

  Serial.print("ERR,UNKNOWN_CMD,");
  Serial.println(cmd);
}

void handleSerialInput() {
  static String line = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processSerialCommand(line);
        line = "";
      }
    }
    else {
      line += c;

      if (line.length() > 120) {
        line = "";
        Serial.println("ERR,SERIAL_LINE_TOO_LONG");
      }
    }
  }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

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

  Wire.begin();          // Arduino Mega: SDA=20, SCL=21
  Wire.setClock(100000);

  Serial.println("BOOT,ARC26_ARDUINO_LOWLEVEL");
  Serial.println("BOOT,ROLE,MOTOR_AND_GY271_ONLY");
  Serial.println("BOOT,BAUD,115200");
  Serial.println("BOOT,I2C,MEGA_SDA20_SCL21");

  initMagnetometer();

  lastMotorCommandMs = millis();
  lastTelemetryMs = millis();

  printHelp();
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  handleSerialInput();
  checkMotorWatchdog();
  publishMagTelemetry(false);
}
