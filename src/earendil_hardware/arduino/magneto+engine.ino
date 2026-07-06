#include <Wire.h>
#include <Servo.h>
#include <math.h>

// =======================================================
// ARC'26 Rover Arduino Mega ROS2 Bridge Firmware
// =======================================================
// Telemetry:
// MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm
//
// Motor commands:
// MOTOR:STOP
// MOTOR:FWD:<pwm>
// MOTOR:BACK:<pwm>
// MOTOR:LEFT:<pwm>
// MOTOR:RIGHT:<pwm>
//
// Config commands:
// TELEM:ON
// TELEM:OFF
// PLANE:XY
// PLANE:XZ
// PLANE:YZ
// OFFSET:<deg>
// HEADING
// STATUS
// PING
// =======================================================


// =======================================================
// GY-271 ADRESLERI
// =======================================================
#define QMC5883_ADDR 0x0D
#define HMC5883_ADDR 0x1E

#define SERIAL_BAUD 115200

// 100 ms -> yaklaşık 10 Hz telemetry
#define TELEMETRY_INTERVAL_MS 100

// Magnetometer ortalama sayısı
#define SAMPLE_COUNT 8
#define SAMPLE_DELAY_MS 2

// Raspberry'den motor komutu kesilirse motor durur
#define MOTOR_WATCHDOG_MS 700


// =======================================================
// GY-271 KALIBRASYON DEGERLERI
// Daha önce rover üzerinde ayarlanan doğru değerler
// =======================================================
#define MAG_X_OFFSET -1264.50
#define MAG_Y_OFFSET 311.00
#define MAG_Z_OFFSET -73.50

#define MAG_X_SCALE 0.81943
#define MAG_Y_SCALE 0.70439
#define MAG_Z_SCALE 2.78393


// =======================================================
// HEADING AYARLARI
// =======================================================
// 0 = XY, 1 = XZ, 2 = YZ
#define HEADING_PLANE_DEFAULT 0

// Daha önce sahada ayarlanan offset
#define HEADING_OFFSET_DEFAULT -53.5

// 4 noktalı heading düzeltme aktif
#define HEADING_CORRECTION_ENABLED true


// =======================================================
// MOTOR SÜRÜCÜ PİNLERİ
// =======================================================
const int L_RPWM = 5;
const int L_LPWM = 6;
const int L_REN  = 7;
const int L_LEN  = 8;

const int R_RPWM = 9;
const int R_LPWM = 10;
const int R_REN  = 11;
const int R_LEN  = 12;

// Sondaj / 3. motor
const int M3_RPWM = 4;
const int M3_LPWM = 13;
const int M3_REN  = 45;

// Bizim son bağlantı notumuzda 46 idi.
// Eğer sende fiziksel olarak 52'ye bağlıysa sadece bu satırı 52 yap.
const int M3_LEN  = 46;


// =======================================================
// PWM AYARLARI
// =======================================================
#define PWM_YAVAS 80
#define PWM_HIZLI 200
const int DRILL_PWM = 200;


// =======================================================
// SERVO NESNELERI
// Eski manuel sistemdeki servo komutları korunmuştur.
// =======================================================
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

const int SERVO_PIN1 = 22;
const int SERVO_PIN2 = 24;
const int SERVO_PIN3 = 26;
const int SERVO_PIN4 = 28;
const int SERVO_PIN5 = 44;

const int SAFE_MIN         = 40;
const int SAFE_MAX         = 140;
const int SAFE_MAX1_SECOND = 45;

const int TUR_MS           = 400;
const int TUR_PWM_YUKARI   = 180;
const int TUR_PWM_ASAGI    = 0;
const int SONRAKI_YUK_PWM1 = 46;
const int SONRAKI_ASG_PWM1 = 134;
const int SONRAKI_YUK_PWM2 = 46;
const int SONRAKI_ASG_PWM2 = 134;
const int MAX_TUR          = 2;

int pwm1 = 45;
int pwm2 = 90;
int pwm3 = 90;
int pwm4 = 180;
int pwm5 = 90;

int turSayisi1 = 2;
int turSayisi2 = 2;

const int JOY_CENTER = 2048;
const int DEADZONE   = 400;
const int STEP       = 2;


// =======================================================
// MAGNETOMETER STATE
// =======================================================
enum MagType {
  MAG_NONE,
  MAG_QMC5883L,
  MAG_HMC5883L
};

MagType magType = MAG_NONE;


// =======================================================
// SYSTEM STATE
// =======================================================
uint8_t activeHeadingPlane = HEADING_PLANE_DEFAULT;
float headingOffsetDeg = HEADING_OFFSET_DEFAULT;

bool telemetryEnabled = true;

unsigned long lastTelemetryMs = 0;
unsigned long lastMotorCommandMs = 0;

String motorMode = "STOP";
int currentMotorPwm = 0;


// =======================================================
// GENEL YARDIMCI FONKSIYONLAR
// =======================================================
float normalizeHeading(float h) {
  while (h >= 360.0) h -= 360.0;
  while (h < 0.0) h += 360.0;
  return h;
}

float angleErrorDeg(float target, float current) {
  float e = target - current;

  while (e > 180.0) e -= 360.0;
  while (e < -180.0) e += 360.0;

  return e;
}

String planeName(uint8_t p) {
  if (p == 0) return "XY";
  if (p == 1) return "XZ";
  return "YZ";
}

String magName() {
  if (magType == MAG_QMC5883L) return "QMC5883L";
  if (magType == MAG_HMC5883L) return "HMC5883L";
  return "NONE";
}

int parsePwmValue(String s) {
  s.trim();
  int pwm = s.toInt();
  pwm = constrain(pwm, 0, 255);
  return pwm;
}

float linearMapFloat(float x, float inA, float inB, float outA, float outB) {
  return outA + (x - inA) * (outB - outA) / (inB - inA);
}


// =======================================================
// HEADING DÜZELTME TABLOSU
// =======================================================
// Motor kapalıyken ölçülen değerler:
//
// Gerçek 0°   -> Ölçülen 19°
// Gerçek 90°  -> Ölçülen 89°
// Gerçek 180° -> Ölçülen 203°
// Gerçek 270° -> Ölçülen 299°
//
// Bu fonksiyon ölçülen heading'i gerçek heading'e yaklaştırır.
// Bu düzeltme PLANE:XY ve OFFSET:-53.5 için geçerlidir.
// Sensör yeri değişirse bu tablo yeniden ölçülmelidir.
// =======================================================
float correctHeadingDeg(float h) {
  if (!HEADING_CORRECTION_ENABLED) {
    return normalizeHeading(h);
  }

  h = normalizeHeading(h);

  float x = h;

  // Wrap bölgesi:
  // measured 299 -> true 270
  // measured 19  -> true 360/0
  if (x < 19.0) {
    x += 360.0;
  }

  float corrected = 0.0;

  if (x >= 19.0 && x < 89.0) {
    corrected = linearMapFloat(x, 19.0, 89.0, 0.0, 90.0);
  }
  else if (x >= 89.0 && x < 203.0) {
    corrected = linearMapFloat(x, 89.0, 203.0, 90.0, 180.0);
  }
  else if (x >= 203.0 && x < 299.0) {
    corrected = linearMapFloat(x, 203.0, 299.0, 180.0, 270.0);
  }
  else {
    corrected = linearMapFloat(x, 299.0, 379.0, 270.0, 360.0);
  }

  return normalizeHeading(corrected);
}


// =======================================================
// I2C / GY-271 FONKSIYONLARI
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

    Serial.println("GY-271 bulundu: QMC5883L, adres 0x0D");

    // QMC5883L:
    // 0x0B: Set/Reset period
    // 0x09: OSR=512, RNG=8G, ODR=200Hz, continuous
    write8(QMC5883_ADDR, 0x0B, 0x01);
    write8(QMC5883_ADDR, 0x09, 0x1D);

    return true;
  }

  if (i2cExists(HMC5883_ADDR)) {
    magType = MAG_HMC5883L;

    Serial.println("GY-271 bulundu: HMC5883L, adres 0x1E");

    write8(HMC5883_ADDR, 0x00, 0x70);
    write8(HMC5883_ADDR, 0x01, 0x20);
    write8(HMC5883_ADDR, 0x02, 0x00);

    return true;
  }

  Serial.println("GY-271 bulunamadi.");
  magType = MAG_NONE;
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

bool readCalibrated(
  float &rawXAvg,
  float &rawYAvg,
  float &rawZAvg,
  float &calX,
  float &calY,
  float &calZ
) {
  long sx = 0;
  long sy = 0;
  long sz = 0;
  int count = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t x, y, z;

    if (readRawMagnetometer(x, y, z)) {
      sx += x;
      sy += y;
      sz += z;
      count++;
    }

    delay(SAMPLE_DELAY_MS);
  }

  if (count == 0) {
    return false;
  }

  rawXAvg = sx / (float)count;
  rawYAvg = sy / (float)count;
  rawZAvg = sz / (float)count;

  calX = (rawXAvg - MAG_X_OFFSET) * MAG_X_SCALE;
  calY = (rawYAvg - MAG_Y_OFFSET) * MAG_Y_SCALE;
  calZ = (rawZAvg - MAG_Z_OFFSET) * MAG_Z_SCALE;

  return true;
}

float calculateMeasuredHeadingFromCalibrated(float calX, float calY, float calZ) {
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

float calculateCorrectedHeadingFromCalibrated(float calX, float calY, float calZ) {
  float measuredHeading = calculateMeasuredHeadingFromCalibrated(calX, calY, calZ);
  return correctHeadingDeg(measuredHeading);
}


// =======================================================
// MOTOR FONKSIYONLARI
// =======================================================
void stopDrive() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  motorMode = "STOP";
  currentMotorPwm = 0;
}

void driveForward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  motorMode = "FWD";
  currentMotorPwm = pwm;
}

void driveBackward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  motorMode = "BACK";
  currentMotorPwm = pwm;
}

// Sağ tank dönüşü:
// Sol motor ileri, sağ motor geri.
void tankRight(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  motorMode = "RIGHT";
  currentMotorPwm = pwm;
}

// Sol tank dönüşü:
// Sol motor geri, sağ motor ileri.
void tankLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  motorMode = "LEFT";
  currentMotorPwm = pwm;
}

void updateMotorCommandTime() {
  lastMotorCommandMs = millis();
}

void sondajYukari() {
  analogWrite(M3_RPWM, DRILL_PWM);
  analogWrite(M3_LPWM, 0);
}

void sondajAsagi() {
  analogWrite(M3_RPWM, 0);
  analogWrite(M3_LPWM, DRILL_PWM);
}

void sondajDur() {
  analogWrite(M3_RPWM, 0);
  analogWrite(M3_LPWM, 0);
}

void checkMotorWatchdog() {
  if (motorMode == "STOP") {
    return;
  }

  unsigned long now = millis();

  if (now - lastMotorCommandMs > MOTOR_WATCHDOG_MS) {
    stopDrive();
    Serial.println("WARN,MOTOR_WATCHDOG_STOP");
  }
}


// =======================================================
// TELEMETRY
// =======================================================
void publishMagTelemetry() {
  if (!telemetryEnabled) {
    return;
  }

  unsigned long now = millis();

  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = now;

  float rawX, rawY, rawZ;
  float calX, calY, calZ;

  if (!readCalibrated(rawX, rawY, rawZ, calX, calY, calZ)) {
    Serial.println("MAG_ERROR,GY271_READ_FAILED");
    return;
  }

  float measuredHeading = calculateMeasuredHeadingFromCalibrated(calX, calY, calZ);
  float heading = correctHeadingDeg(measuredHeading);

  // ROS2 bridge bu satırdaki 3. alanı heading olarak kullanır.
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

  Serial.print(motorMode);
  Serial.print(",");
  Serial.print(currentMotorPwm);

  Serial.println();
}

void printHeadingDebug() {
  float rawX, rawY, rawZ;
  float calX, calY, calZ;

  if (!readCalibrated(rawX, rawY, rawZ, calX, calY, calZ)) {
    Serial.println("GY-271 okunamadi.");
    return;
  }

  float measuredHeading = calculateMeasuredHeadingFromCalibrated(calX, calY, calZ);
  float correctedHeading = correctHeadingDeg(measuredHeading);

  float hXY_measured = normalizeHeading(atan2(calY, calX) * 180.0 / PI + headingOffsetDeg);
  float hXZ_measured = normalizeHeading(atan2(calZ, calX) * 180.0 / PI + headingOffsetDeg);
  float hYZ_measured = normalizeHeading(atan2(calZ, calY) * 180.0 / PI + headingOffsetDeg);

  Serial.print("RAW X:");
  Serial.print(rawX, 0);
  Serial.print(" Y:");
  Serial.print(rawY, 0);
  Serial.print(" Z:");
  Serial.print(rawZ, 0);

  Serial.print(" || CAL X:");
  Serial.print(calX, 1);
  Serial.print(" Y:");
  Serial.print(calY, 1);
  Serial.print(" Z:");
  Serial.print(calZ, 1);

  Serial.print(" || H_XY_MEAS:");
  Serial.print(hXY_measured, 1);
  Serial.print(" H_XZ_MEAS:");
  Serial.print(hXZ_measured, 1);
  Serial.print(" H_YZ_MEAS:");
  Serial.print(hYZ_measured, 1);

  Serial.print(" || ACTIVE:");
  Serial.print(planeName(activeHeadingPlane));

  Serial.print(" MEAS:");
  Serial.print(measuredHeading, 1);

  Serial.print(" CORR:");
  Serial.print(correctedHeading, 1);

  Serial.print(" OFFSET:");
  Serial.print(headingOffsetDeg, 1);

  Serial.println();
}

void printStatus() {
  Serial.print("STATUS,");
  Serial.print("mag=");
  Serial.print(magName());

  Serial.print(",plane=");
  Serial.print(planeName(activeHeadingPlane));

  Serial.print(",offset=");
  Serial.print(headingOffsetDeg, 2);

  Serial.print(",heading_correction=");
  Serial.print(HEADING_CORRECTION_ENABLED ? "ON" : "OFF");

  Serial.print(",telemetry=");
  Serial.print(telemetryEnabled ? "ON" : "OFF");

  Serial.print(",motor=");
  Serial.print(motorMode);

  Serial.print(",pwm=");
  Serial.print(currentMotorPwm);

  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("===== ROS2 UYUMLU KOMUTLAR =====");
  Serial.println("TELEM:ON              -> MAG telemetry ac");
  Serial.println("TELEM:OFF             -> MAG telemetry kapat");
  Serial.println("MOTOR:STOP            -> Motorlari durdur");
  Serial.println("MOTOR:FWD:<pwm>       -> Ileri");
  Serial.println("MOTOR:BACK:<pwm>      -> Geri");
  Serial.println("MOTOR:LEFT:<pwm>      -> Sola tank donus");
  Serial.println("MOTOR:RIGHT:<pwm>     -> Saga tank donus");
  Serial.println("PLANE:XY              -> Heading duzlemi XY");
  Serial.println("PLANE:XZ              -> Heading duzlemi XZ");
  Serial.println("PLANE:YZ              -> Heading duzlemi YZ");
  Serial.println("OFFSET:<deg>          -> Heading offset gir");
  Serial.println("HEADING               -> Debug heading yazdir");
  Serial.println("STATUS                -> Sistem durumunu yazdir");
  Serial.println("PING                  -> PONG");
  Serial.println();
  Serial.println("Eski manuel komutlar da desteklenir:");
  Serial.println("ileri_hizli, ileri_yavas, geri_hizli, geri_yavas");
  Serial.println("sag_hizli, sag_yavas, sol_hizli, sol_yavas, dur");
  Serial.println("sondaj:yukari, sondaj:asagi, sondaj:dur");
  Serial.println("servo5:yukari, servo5:asagi");
  Serial.println("x2:<val>, x3:<val>, y2:<val>, y3:<val>");
  Serial.println("===============================");
  Serial.println();
}


// =======================================================
// SERIAL KOMUT PARSE
// =======================================================
bool handleMotorCommand(String veri) {
  if (veri == "MOTOR:STOP") {
    stopDrive();
    updateMotorCommandTime();
    Serial.println("ACK,MOTOR:STOP");
    return true;
  }

  if (veri.startsWith("MOTOR:FWD:")) {
    int pwm = parsePwmValue(veri.substring(10));
    driveForward(pwm);
    updateMotorCommandTime();
    Serial.print("ACK,MOTOR:FWD:");
    Serial.println(pwm);
    return true;
  }

  if (veri.startsWith("MOTOR:BACK:")) {
    int pwm = parsePwmValue(veri.substring(11));
    driveBackward(pwm);
    updateMotorCommandTime();
    Serial.print("ACK,MOTOR:BACK:");
    Serial.println(pwm);
    return true;
  }

  if (veri.startsWith("MOTOR:LEFT:")) {
    int pwm = parsePwmValue(veri.substring(11));
    tankLeft(pwm);
    updateMotorCommandTime();
    Serial.print("ACK,MOTOR:LEFT:");
    Serial.println(pwm);
    return true;
  }

  if (veri.startsWith("MOTOR:RIGHT:")) {
    int pwm = parsePwmValue(veri.substring(12));
    tankRight(pwm);
    updateMotorCommandTime();
    Serial.print("ACK,MOTOR:RIGHT:");
    Serial.println(pwm);
    return true;
  }

  return false;
}

bool handleConfigCommand(String veri) {
  if (veri == "PING") {
    Serial.println("PONG");
    return true;
  }

  if (veri == "HELP" || veri == "help") {
    printHelp();
    return true;
  }

  if (veri == "STATUS") {
    printStatus();
    return true;
  }

  if (veri == "HEADING") {
    printHeadingDebug();
    return true;
  }

  if (veri == "TELEM:ON") {
    telemetryEnabled = true;
    Serial.println("ACK,TELEM:ON");
    return true;
  }

  if (veri == "TELEM:OFF") {
    telemetryEnabled = false;
    Serial.println("ACK,TELEM:OFF");
    return true;
  }

  if (veri == "PLANE:XY") {
    activeHeadingPlane = 0;
    Serial.println("ACK,PLANE:XY");
    return true;
  }

  if (veri == "PLANE:XZ") {
    activeHeadingPlane = 1;
    Serial.println("ACK,PLANE:XZ");
    return true;
  }

  if (veri == "PLANE:YZ") {
    activeHeadingPlane = 2;
    Serial.println("ACK,PLANE:YZ");
    return true;
  }

  if (veri.startsWith("OFFSET:")) {
    headingOffsetDeg = veri.substring(7).toFloat();

    Serial.print("ACK,OFFSET:");
    Serial.println(headingOffsetDeg, 2);

    return true;
  }

  return false;
}

bool handleLegacyManualCommand(String veri) {
  // Eski manuel yön komutları
  if (veri == "ileri_hizli") {
    driveForward(PWM_HIZLI);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "ileri_yavas") {
    driveForward(PWM_YAVAS);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "geri_hizli") {
    driveBackward(PWM_HIZLI);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "geri_yavas") {
    driveBackward(PWM_YAVAS);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "sag_hizli") {
    tankRight(PWM_HIZLI);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "sag_yavas") {
    tankRight(PWM_YAVAS);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "sol_hizli") {
    tankLeft(PWM_HIZLI);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "sol_yavas") {
    tankLeft(PWM_YAVAS);
    updateMotorCommandTime();
    return true;
  }
  else if (veri == "dur") {
    stopDrive();
    updateMotorCommandTime();
    return true;
  }

  // Sondaj
  else if (veri == "sondaj:yukari") {
    sondajYukari();
    return true;
  }
  else if (veri == "sondaj:asagi") {
    sondajAsagi();
    return true;
  }
  else if (veri == "sondaj:dur") {
    sondajDur();
    return true;
  }

  // Servo5
  else if (veri == "servo5:yukari") {
    pwm5 += STEP;
    pwm5 = constrain(pwm5, 0, 180);
    servo5.write(pwm5);
    return true;
  }
  else if (veri == "servo5:asagi") {
    pwm5 -= STEP;
    pwm5 = constrain(pwm5, 0, 180);
    servo5.write(pwm5);
    return true;
  }

  // Normal servo3
  else if (veri.startsWith("y2:")) {
    int deger = veri.substring(3).toInt();
    int diff = deger - JOY_CENTER;

    if (abs(diff) > DEADZONE) {
      pwm3 += (diff > 0) ? STEP : -STEP;
      pwm3 = constrain(pwm3, 0, 180);
      servo3.write(pwm3);
    }

    return true;
  }

  // Normal servo4
  else if (veri.startsWith("y3:")) {
    int deger = veri.substring(3).toInt();
    int diff = deger - JOY_CENTER;

    if (abs(diff) > DEADZONE) {
      pwm4 += (diff > 0) ? STEP : -STEP;
      pwm4 = constrain(pwm4, 0, 180);
      servo4.write(pwm4);
    }

    return true;
  }

  // Çok turlu servo1
  else if (veri.startsWith("x3:")) {
    int deger = veri.substring(3).toInt();
    int diff = deger - JOY_CENTER;

    if (abs(diff) > DEADZONE) {
      if (diff > 0) {
        if (pwm1 >= ((turSayisi1 == MAX_TUR) ? SAFE_MAX1_SECOND : SAFE_MAX)) {
          if (turSayisi1 < MAX_TUR) {
            servo1.write(TUR_PWM_YUKARI);
            delay(TUR_MS);
            pwm1 = SONRAKI_YUK_PWM1;
            turSayisi1++;
          }
        }
        else {
          pwm1 += STEP;
        }
      }
      else {
        if (pwm1 <= SAFE_MIN) {
          if (turSayisi1 > 1) {
            servo1.write(TUR_PWM_ASAGI);
            delay(TUR_MS);
            pwm1 = SONRAKI_ASG_PWM1;
            turSayisi1--;
          }
        }
        else {
          pwm1 -= STEP;
        }
      }

      int safeMax1 = (turSayisi1 == MAX_TUR) ? SAFE_MAX1_SECOND : SAFE_MAX;
      pwm1 = constrain(pwm1, SAFE_MIN, safeMax1);
      servo1.write(pwm1);
    }

    return true;
  }

  // Çok turlu servo2
  else if (veri.startsWith("x2:")) {
    int deger = veri.substring(3).toInt();
    int diff = deger - JOY_CENTER;

    if (abs(diff) > DEADZONE) {
      if (diff > 0) {
        if (pwm2 >= SAFE_MAX) {
          if (turSayisi2 < MAX_TUR) {
            servo2.write(TUR_PWM_YUKARI);
            delay(TUR_MS);
            pwm2 = SONRAKI_YUK_PWM2;
            turSayisi2++;
          }
        }
        else {
          pwm2 += STEP;
        }
      }
      else {
        if (pwm2 <= SAFE_MIN) {
          if (turSayisi2 > 1) {
            servo2.write(TUR_PWM_ASAGI);
            delay(TUR_MS);
            pwm2 = SONRAKI_ASG_PWM2;
            turSayisi2--;
          }
        }
        else {
          pwm2 -= STEP;
        }
      }

      pwm2 = constrain(pwm2, SAFE_MIN, SAFE_MAX);
      servo2.write(pwm2);
    }

    return true;
  }

  return false;
}

void processSerialLine(String veri) {
  veri.trim();

  if (veri.length() == 0) {
    return;
  }

  if (handleMotorCommand(veri)) {
    return;
  }

  if (handleConfigCommand(veri)) {
    return;
  }

  if (handleLegacyManualCommand(veri)) {
    return;
  }

  Serial.print("WARN,UNKNOWN_COMMAND:");
  Serial.println(veri);
}

// Serial'i bloklamadan okuyoruz.
void handleSerialInput() {
  static String line = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processSerialLine(line);
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
// SETUP
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin();          // Arduino Mega: SDA=20, SCL=21
  Wire.setClock(100000);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);
  pinMode(M3_REN, OUTPUT);
  pinMode(M3_LEN, OUTPUT);

  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  digitalWrite(M3_REN, HIGH);
  digitalWrite(M3_LEN, HIGH);

  stopDrive();
  sondajDur();

  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);
  servo4.attach(SERVO_PIN4);
  servo5.attach(SERVO_PIN5);

  servo1.write(pwm1);
  servo2.write(pwm2);
  servo3.write(pwm3);
  servo4.write(pwm4);
  servo5.write(pwm5);

  Serial.println();
  Serial.println("ARC'26 Rover ROS2 Arduino Firmware Basladi.");
  Serial.println("Arduino Mega I2C: SDA=20, SCL=21");
  Serial.println("Serial baud: 115200");

  initMagnetometer();

  printHelp();

  lastMotorCommandMs = millis();
  lastTelemetryMs = millis();
}


// =======================================================
// LOOP
// =======================================================
void loop() {
  handleSerialInput();
  checkMotorWatchdog();
  publishMagTelemetry();
}