#include <Wire.h>
#include <math.h>

// =======================================================
// I2C ADRESLERİ
// =======================================================
#define MPU_ADDR_1 0x68
#define MPU_ADDR_2 0x69
#define QMC5883P_ADDR 0x2C

#define SERIAL_BAUD 115200
// ROS2 için 300ms çok yavaş kalır (3 Hz). 50ms (20 Hz) idealdir.
#define LOOP_DELAY_MS 50 

// =======================================================
// GY-271 / QMC5883P AYARLARI
// =======================================================
#define MAG_SAMPLE_COUNT 10
#define MAG_X_OFFSET 0.0
#define MAG_Y_OFFSET 0.0
#define MAG_Z_OFFSET 0.0
#define MAG_X_SCALE 1.0
#define MAG_Y_SCALE 1.0
#define MAG_Z_SCALE 1.0
#define HEADING_OFFSET_DEG 0.0
#define INVERT_HEADING false

// =======================================================
// GLOBAL DURUMLAR
// =======================================================
uint8_t mpuAddr = 0;
bool mpuAvailable = false;
bool magAvailable = false;

unsigned long lastMPURetryMs = 0;
unsigned long lastMagRetryMs = 0;
#define RETRY_INTERVAL_MS 2000

// =======================================================
// I2C YARDIMCI FONKSİYONLAR
// =======================================================
bool i2cExists(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool read8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(addr, (uint8_t)1);
  if (received != 1) return false;
  if (!Wire.available()) return false;
  val = Wire.read();
  return true;
}

bool readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(addr, count);
  if (received != count) return false;
  for (uint8_t i = 0; i < count; i++) {
    if (!Wire.available()) return false;
    dest[i] = Wire.read();
  }
  return true;
}

float normalizeAngle(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

// =======================================================
// GY-91 / MPU BAŞLATMA
// =======================================================
bool initMPU() {
  if (i2cExists(MPU_ADDR_1)) mpuAddr = MPU_ADDR_1;
  else if (i2cExists(MPU_ADDR_2)) mpuAddr = MPU_ADDR_2;
  else { mpuAddr = 0; return false; }

  uint8_t whoami = 0;
  read8(mpuAddr, 0x75, whoami);

  write8(mpuAddr, 0x6B, 0x00); delay(100); 
  write8(mpuAddr, 0x6B, 0x01); delay(100); 
  write8(mpuAddr, 0x1A, 0x03);             
  write8(mpuAddr, 0x1B, 0x00);             
  write8(mpuAddr, 0x1C, 0x00);             
  write8(mpuAddr, 0x1D, 0x03);             

  return true;
}

// =======================================================
// GY-91 IMU OKUMA
// =======================================================
bool readMPU(float &ax_ms2, float &ay_ms2, float &az_ms2,
             float &gx_rads, float &gy_rads, float &gz_rads) {
  if (mpuAddr == 0) return false;
  uint8_t b[14];
  if (!readBytes(mpuAddr, 0x3B, 14, b)) return false;

  int16_t rawAx = ((int16_t)b[0] << 8) | b[1];
  int16_t rawAy = ((int16_t)b[2] << 8) | b[3];
  int16_t rawAz = ((int16_t)b[4] << 8) | b[5];

  int16_t rawGx = ((int16_t)b[8] << 8) | b[9];
  int16_t rawGy = ((int16_t)b[10] << 8) | b[11];
  int16_t rawGz = ((int16_t)b[12] << 8) | b[13];

  // ROS2 Standartlarına Çevirme (m/s^2 ve rad/s)
  ax_ms2 = (rawAx / 16384.0) * 9.80665;
  ay_ms2 = (rawAy / 16384.0) * 9.80665;
  az_ms2 = (rawAz / 16384.0) * 9.80665;

  gx_rads = (rawGx / 131.0) * (PI / 180.0);
  gy_rads = (rawGy / 131.0) * (PI / 180.0);
  gz_rads = (rawGz / 131.0) * (PI / 180.0);

  return true;
}

// =======================================================
// QMC5883P / 0x2C PUSULA BAŞLATMA
// =======================================================
bool initQMC5883P() {
  if (!i2cExists(QMC5883P_ADDR)) return false;
  write8(QMC5883P_ADDR, 0x29, 0x06); delay(10);
  write8(QMC5883P_ADDR, 0x0B, 0x08); delay(10);
  write8(QMC5883P_ADDR, 0x0A, 0xCD); delay(100);
  return true;
}

// =======================================================
// QMC5883P HAM VERİ OKUMA
// =======================================================
bool readRawQMC5883P(int16_t &rawX, int16_t &rawY, int16_t &rawZ) {
  uint8_t b[6];
  if (!readBytes(QMC5883P_ADDR, 0x01, 6, b)) return false;
  rawX = ((int16_t)b[1] << 8) | b[0];
  rawY = ((int16_t)b[3] << 8) | b[2];
  rawZ = ((int16_t)b[5] << 8) | b[4];
  return true;
}

// =======================================================
// QMC5883P HEADING OKUMA
// =======================================================
bool readMagHeading(float &headingDeg) {
  long sumX = 0, sumY = 0, sumZ = 0;
  int validCount = 0;

  for (int i = 0; i < MAG_SAMPLE_COUNT; i++) {
    int16_t rawX, rawY, rawZ;
    if (readRawQMC5883P(rawX, rawY, rawZ)) {
      sumX += rawX; sumY += rawY; sumZ += rawZ;
      validCount++;
    }
    delay(2); // Toplama esnasında minik gecikme
  }

  if (validCount == 0) return false;

  float rawXAvg = sumX / (float)validCount;
  float rawYAvg = sumY / (float)validCount;
  float rawZAvg = sumZ / (float)validCount;

  float calX = (rawXAvg - MAG_X_OFFSET) * MAG_X_SCALE;
  float calY = (rawYAvg - MAG_Y_OFFSET) * MAG_Y_SCALE;

  headingDeg = atan2(calY, calX) * 180.0 / PI;
  headingDeg = normalizeAngle(headingDeg);

#if INVERT_HEADING
  headingDeg = 360.0 - headingDeg;
  headingDeg = normalizeAngle(headingDeg);
#endif

  headingDeg += HEADING_OFFSET_DEG;
  headingDeg = normalizeAngle(headingDeg);

  return true;
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  // Bu WARN ve ERR prefixleri hardware_bridge.py tarafından algılanıp ROS Loglarına aktarılır.
  Serial.println("WARN,Arduino basliyor. I2C taramasi ve Init yapiliyor...");

  mpuAvailable = initMPU();
  if (!mpuAvailable) Serial.println("ERR,GY-91 bulunamadi!");

  magAvailable = initQMC5883P();
  if (!magAvailable) Serial.println("ERR,QMC5883P bulunamadi!");
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  float ax, ay, az, gx, gy, gz;
  float heading;

  // =====================================================
  // GY-91 IMU (hardware_bridge.py -> _parse_imu fonksiyonu ile okunur)
  // =====================================================
  if (!mpuAvailable) {
    if (millis() - lastMPURetryMs > RETRY_INTERVAL_MS) {
      lastMPURetryMs = millis();
      mpuAvailable = initMPU();
    }
  }

  if (mpuAvailable) {
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      // Beklenen format: IMU,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
      Serial.print("IMU,");
      Serial.print(gx, 4); Serial.print(",");
      Serial.print(gy, 4); Serial.print(",");
      Serial.print(gz, 4); Serial.print(",");
      Serial.print(ax, 4); Serial.print(",");
      Serial.print(ay, 4); Serial.print(",");
      Serial.println(az, 4);
    } else {
      mpuAvailable = false;
      mpuAddr = 0;
    }
  }

  // =====================================================
  // GY-271 0x2C PUSULA (hardware_bridge.py -> _parse_mag fonksiyonu ile okunur)
  // =====================================================
  if (!magAvailable) {
    if (millis() - lastMagRetryMs > RETRY_INTERVAL_MS) {
      lastMagRetryMs = millis();
      magAvailable = initQMC5883P();
    }
  }

  if (magAvailable) {
    if (readMagHeading(heading)) {
      // Beklenen format: MAG,time_ms,heading_deg
      Serial.print("MAG,");
      Serial.print(millis());
      Serial.print(",");
      Serial.println(heading, 2);
    } else {
      magAvailable = false;
    }
  }

  delay(LOOP_DELAY_MS);
}

