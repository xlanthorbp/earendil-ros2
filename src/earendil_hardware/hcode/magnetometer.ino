#include <Wire.h>
#include <math.h>

#define QMC5883_ADDR 0x0D
#define HMC5883_ADDR 0x1E

#define SERIAL_BAUD 115200
#define LOOP_DELAY_MS 300
#define SAMPLE_COUNT 20

// Mevcut kalibrasyon değerlerin.
// Şimdilik eksen testi için kullanıyoruz.
#define MAG_X_OFFSET -356.50
#define MAG_Y_OFFSET -134.00
#define MAG_Z_OFFSET 55.00

#define MAG_X_SCALE 1.05697
#define MAG_Y_SCALE 0.93964
#define MAG_Z_SCALE 1.05844

enum MagType {
  MAG_NONE,
  MAG_QMC5883L,
  MAG_HMC5883L
};

MagType magType = MAG_NONE;

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

void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(addr, count);

  uint8_t i = 0;
  while (Wire.available() && i < count) {
    dest[i++] = Wire.read();
  }
}

float normalizeHeading(float h) {
  while (h >= 360.0) h -= 360.0;
  while (h < 0.0) h += 360.0;
  return h;
}

bool initMagnetometer() {
  if (i2cExists(QMC5883_ADDR)) {
    magType = MAG_QMC5883L;

    Serial.println("GY-271 bulundu: QMC5883L, adres 0x0D");

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
    uint8_t b[6];
    readBytes(QMC5883_ADDR, 0x00, 6, b);

    rawX = ((int16_t)b[1] << 8) | b[0];
    rawY = ((int16_t)b[3] << 8) | b[2];
    rawZ = ((int16_t)b[5] << 8) | b[4];

    return true;
  }

  if (magType == MAG_HMC5883L) {
    uint8_t b[6];
    readBytes(HMC5883_ADDR, 0x03, 6, b);

    rawX = ((int16_t)b[0] << 8) | b[1];
    rawZ = ((int16_t)b[2] << 8) | b[3];
    rawY = ((int16_t)b[4] << 8) | b[5];

    return true;
  }

  return false;
}

bool readCalibrated(float &rawXAvg, float &rawYAvg, float &rawZAvg,
                    float &calX, float &calY, float &calZ) {
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

    delay(5);
  }

  if (count == 0) return false;

  rawXAvg = sx / (float)count;
  rawYAvg = sy / (float)count;
  rawZAvg = sz / (float)count;

  calX = (rawXAvg - MAG_X_OFFSET) * MAG_X_SCALE;
  calY = (rawYAvg - MAG_Y_OFFSET) * MAG_Y_SCALE;
  calZ = (rawZAvg - MAG_Z_OFFSET) * MAG_Z_SCALE;

  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin();          // Mega: SDA=20, SCL=21
  Wire.setClock(100000);

  Serial.println();
  Serial.println("GY-271 eksen testi basladi.");
  Serial.println("Arduino Mega I2C: SDA=20, SCL=21");
  Serial.println();

  initMagnetometer();

  Serial.println();
  Serial.println("Roveri sabit tut, sonra 90 derece cevir.");
  Serial.println("Hangi heading yaklasik 90 derece degisiyor ona bak.");
  Serial.println("------------------------------------------------------");
}

void loop() {
  float rawX, rawY, rawZ;
  float calX, calY, calZ;

  if (readCalibrated(rawX, rawY, rawZ, calX, calY, calZ)) {
    float headingXY = normalizeHeading(atan2(calY, calX) * 180.0 / PI);

    // ROS-parseable format: MAG,heading
    // Pi tarafindaki mag_heading_node.py bu formati okur.
    Serial.print("MAG,");
    Serial.println(headingXY, 1);

    // Debug: Detayli ciktiyi gormek istersen yukaridaki 2 satiri
    // yorum satirina al ve asagidakileri ac.
    // float headingXZ = normalizeHeading(atan2(calZ, calX) * 180.0 / PI);
    // float headingYZ = normalizeHeading(atan2(calZ, calY) * 180.0 / PI);
    // Serial.print("RAW X: "); Serial.print(rawX, 0);
    // Serial.print(" | RAW Y: "); Serial.print(rawY, 0);
    // Serial.print(" | RAW Z: "); Serial.print(rawZ, 0);
    // Serial.print(" || Heading XY: "); Serial.print(headingXY, 1);
    // Serial.print(" | Heading XZ: "); Serial.print(headingXZ, 1);
    // Serial.print(" | Heading YZ: "); Serial.println(headingYZ, 1);
  } 
  else {
    Serial.println("MAG,ERR");
  }

  delay(LOOP_DELAY_MS);
}