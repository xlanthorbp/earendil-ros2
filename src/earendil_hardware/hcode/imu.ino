#include <Wire.h>
#include <math.h>

// =======================================================
// I2C ADRESLERİ
// =======================================================
#define MPU_ADDR_1 0x68
#define MPU_ADDR_2 0x69

#define BMP_ADDR_1 0x76
#define BMP_ADDR_2 0x77

#define QMC5883_ADDR 0x0D   // Çoğu yeni GY-271 modülü
#define HMC5883_ADDR 0x1E   // Eski/orijinal GY-271 modülü

// =======================================================
// AYARLAR
// =======================================================
#define SERIAL_BAUD 115200
#define LOOP_DELAY_MS 500

// Deniz seviyesi basıncı. İrtifa yaklaşık hesabı için.
// Bulunduğun yere göre değişebilir.
#define SEA_LEVEL_HPA 1013.25

// Manyetometre kalibrasyon offsetleri.
// Şimdilik 0 bırak. Sonra kalibrasyon yaparsan buraya yazarsın.
float MAG_X_OFFSET = 0;
float MAG_Y_OFFSET = 0;
float MAG_Z_OFFSET = 0;

// =======================================================
// GLOBAL DEĞİŞKENLER
// =======================================================
uint8_t mpuAddr = 0;
uint8_t bmpAddr = 0;

enum MagType {
  MAG_NONE,
  MAG_QMC5883L,
  MAG_HMC5883L
};

MagType magType = MAG_NONE;

// BMP280 kalibrasyon değişkenleri
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

// =======================================================
// I2C YARDIMCI FONKSİYONLAR
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

uint8_t read8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
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

uint16_t readU16LE(uint8_t addr, uint8_t reg) {
  uint8_t b[2];
  readBytes(addr, reg, 2, b);
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

int16_t readS16LE(uint8_t addr, uint8_t reg) {
  return (int16_t)readU16LE(addr, reg);
}

// =======================================================
// I2C TARAMA
// =======================================================
void scanI2C() {
  Serial.println("I2C taraniyor...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cExists(addr)) {
      Serial.print("Cihaz bulundu: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  Serial.println();
}

// =======================================================
// MPU9250 / MPU6050 BAŞLATMA
// =======================================================
bool initMPU() {
  if (i2cExists(MPU_ADDR_1)) {
    mpuAddr = MPU_ADDR_1;
  } else if (i2cExists(MPU_ADDR_2)) {
    mpuAddr = MPU_ADDR_2;
  } else {
    Serial.println("MPU bulunamadi. GY-91 MPU kismi yok.");
    return false;
  }

  uint8_t whoami = read8(mpuAddr, 0x75);

  Serial.print("MPU bulundu. Adres: 0x");
  Serial.print(mpuAddr, HEX);
  Serial.print(" WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  // Uyandır
  write8(mpuAddr, 0x6B, 0x00);
  delay(100);

  // DLPF ayarı
  write8(mpuAddr, 0x1A, 0x03);

  // Gyro ±250 dps
  write8(mpuAddr, 0x1B, 0x00);

  // Accel ±2g
  write8(mpuAddr, 0x1C, 0x00);

  // Accel DLPF
  write8(mpuAddr, 0x1D, 0x03);

  return true;
}

bool readMPU(float &ax_g, float &ay_g, float &az_g,
             float &gx_dps, float &gy_dps, float &gz_dps,
             float &imu_temp_c) {
  if (mpuAddr == 0) return false;

  uint8_t b[14];
  readBytes(mpuAddr, 0x3B, 14, b);

  int16_t ax = ((int16_t)b[0] << 8) | b[1];
  int16_t ay = ((int16_t)b[2] << 8) | b[3];
  int16_t az = ((int16_t)b[4] << 8) | b[5];

  int16_t tempRaw = ((int16_t)b[6] << 8) | b[7];

  int16_t gx = ((int16_t)b[8] << 8) | b[9];
  int16_t gy = ((int16_t)b[10] << 8) | b[11];
  int16_t gz = ((int16_t)b[12] << 8) | b[13];

  ax_g = ax / 16384.0;
  ay_g = ay / 16384.0;
  az_g = az / 16384.0;

  gx_dps = gx / 131.0;
  gy_dps = gy / 131.0;
  gz_dps = gz / 131.0;

  // MPU9250 sıcaklık formülü
  imu_temp_c = tempRaw / 333.87 + 21.0;

  return true;
}

// =======================================================
// BMP280 BAŞLATMA
// =======================================================
bool initBMP280() {
  if (i2cExists(BMP_ADDR_1)) {
    bmpAddr = BMP_ADDR_1;
  } else if (i2cExists(BMP_ADDR_2)) {
    bmpAddr = BMP_ADDR_2;
  } else {
    Serial.println("BMP280 bulunamadi. GY-91 barometre kismi yok.");
    return false;
  }

  uint8_t id = read8(bmpAddr, 0xD0);

  Serial.print("BMP bulundu. Adres: 0x");
  Serial.print(bmpAddr, HEX);
  Serial.print(" ID: 0x");
  Serial.println(id, HEX);

  if (id != 0x58 && id != 0x60) {
    Serial.println("Uyari: Bu sensor BMP280/BME280 gibi gorunmuyor olabilir.");
  }

  // Kalibrasyon verilerini oku
  dig_T1 = readU16LE(bmpAddr, 0x88);
  dig_T2 = readS16LE(bmpAddr, 0x8A);
  dig_T3 = readS16LE(bmpAddr, 0x8C);

  dig_P1 = readU16LE(bmpAddr, 0x8E);
  dig_P2 = readS16LE(bmpAddr, 0x90);
  dig_P3 = readS16LE(bmpAddr, 0x92);
  dig_P4 = readS16LE(bmpAddr, 0x94);
  dig_P5 = readS16LE(bmpAddr, 0x96);
  dig_P6 = readS16LE(bmpAddr, 0x98);
  dig_P7 = readS16LE(bmpAddr, 0x9A);
  dig_P8 = readS16LE(bmpAddr, 0x9C);
  dig_P9 = readS16LE(bmpAddr, 0x9E);

  // Normal mode, temp x1, pressure x1
  write8(bmpAddr, 0xF4, 0x27);

  // Standby 1000 ms, filtre kapalı
  write8(bmpAddr, 0xF5, 0xA0);

  return true;
}

int32_t compensateTemperatureBMP280(int32_t adc_T) {
  int32_t var1, var2, T;

  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
          ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  return T; // 0.01 derece C
}

uint32_t compensatePressureBMP280(int32_t adc_P) {
  int64_t var1, var2, p;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) {
    return 0;
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

  return (uint32_t)p; // Pa * 256
}

bool readBMP280(float &bmp_temp_c, float &pressure_hpa, float &altitude_m) {
  if (bmpAddr == 0) return false;

  uint8_t b[6];
  readBytes(bmpAddr, 0xF7, 6, b);

  int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

  int32_t temp100 = compensateTemperatureBMP280(adc_T);
  uint32_t press256 = compensatePressureBMP280(adc_P);

  bmp_temp_c = temp100 / 100.0;
  float pressure_pa = press256 / 256.0;
  pressure_hpa = pressure_pa / 100.0;

  altitude_m = 44330.0 * (1.0 - pow(pressure_hpa / SEA_LEVEL_HPA, 0.1903));

  return true;
}

// =======================================================
// GY-271 MANYETOMETRE BAŞLATMA
// =======================================================
bool initMagnetometer() {
  if (i2cExists(QMC5883_ADDR)) {
    magType = MAG_QMC5883L;

    Serial.println("GY-271 manyetometre bulundu: QMC5883L, adres 0x0D");

    // QMC5883L init
    write8(QMC5883_ADDR, 0x0B, 0x01); // Set/reset period
    write8(QMC5883_ADDR, 0x09, 0x1D); // Continuous, 200Hz, 8G, OSR ayari

    return true;
  }

  if (i2cExists(HMC5883_ADDR)) {
    magType = MAG_HMC5883L;

    Serial.println("GY-271 manyetometre bulundu: HMC5883L, adres 0x1E");

    // HMC5883L init
    write8(HMC5883_ADDR, 0x00, 0x70); // 8-average, 15Hz
    write8(HMC5883_ADDR, 0x01, 0x20); // Gain
    write8(HMC5883_ADDR, 0x02, 0x00); // Continuous mode

    return true;
  }

  Serial.println("GY-271 manyetometre bulunamadi. 0x0D veya 0x1E yok.");
  magType = MAG_NONE;
  return false;
}

bool readMagnetometer(float &mx, float &my, float &mz, float &heading_deg) {
  if (magType == MAG_NONE) return false;

  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;

  if (magType == MAG_QMC5883L) {
    uint8_t b[6];
    readBytes(QMC5883_ADDR, 0x00, 6, b);

    // QMC5883L little-endian: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    rawX = ((int16_t)b[1] << 8) | b[0];
    rawY = ((int16_t)b[3] << 8) | b[2];
    rawZ = ((int16_t)b[5] << 8) | b[4];
  }

  else if (magType == MAG_HMC5883L) {
    uint8_t b[6];
    readBytes(HMC5883_ADDR, 0x03, 6, b);

    // HMC5883L big-endian: X, Z, Y
    rawX = ((int16_t)b[0] << 8) | b[1];
    rawZ = ((int16_t)b[2] << 8) | b[3];
    rawY = ((int16_t)b[4] << 8) | b[5];
  }

  mx = rawX - MAG_X_OFFSET;
  my = rawY - MAG_Y_OFFSET;
  mz = rawZ - MAG_Z_OFFSET;

  heading_deg = atan2(my, mx) * 180.0 / PI;

  if (heading_deg < 0) {
    heading_deg += 360.0;
  }

  return true;
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin();          // Mega'da SDA=20, SCL=21
  Wire.setClock(100000); // 100 kHz daha stabil

  Serial.println();
  Serial.println("GY-91 + GY-271 Arduino Mega veri okuma basladi.");
  Serial.println("Mega I2C: SDA=20, SCL=21");
  Serial.println();

  scanI2C();

  initMPU();
  initBMP280();
  initMagnetometer();

  Serial.println();
  Serial.println("Okuma basliyor...");
  Serial.println("----------------------------------------------------");
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float imuTemp;

  float bmpTemp;
  float pressure;
  float altitude;

  float mx, my, mz;
  float heading;

  Serial.println();

  // ================= MPU =================
  if (readMPU(ax, ay, az, gx, gy, gz, imuTemp)) {
    Serial.println("GY-91 / MPU verileri:");

    Serial.print("Ivme     X: ");
    Serial.print(ax, 3);
    Serial.print(" g | Y: ");
    Serial.print(ay, 3);
    Serial.print(" g | Z: ");
    Serial.print(az, 3);
    Serial.println(" g");

    Serial.print("Gyro     X: ");
    Serial.print(gx, 2);
    Serial.print(" dps | Y: ");
    Serial.print(gy, 2);
    Serial.print(" dps | Z: ");
    Serial.print(gz, 2);
    Serial.println(" dps");

    Serial.print("IMU Sicaklik: ");
    Serial.print(imuTemp, 2);
    Serial.println(" C");
  } else {
    Serial.println("MPU verisi okunamadi.");
  }

  // ================= BMP280 =================
  if (readBMP280(bmpTemp, pressure, altitude)) {
    Serial.println();

    Serial.println("GY-91 / BMP280 verileri:");

    Serial.print("BMP Sicaklik: ");
    Serial.print(bmpTemp, 2);
    Serial.println(" C");

    Serial.print("Basinc: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    Serial.print("Yaklasik Irtifa: ");
    Serial.print(altitude, 2);
    Serial.println(" m");
  } else {
    Serial.println("BMP280 verisi okunamadi.");
  }

  // ================= GY-271 =================
  if (readMagnetometer(mx, my, mz, heading)) {
    Serial.println();

    Serial.println("GY-271 / Manyetometre verileri:");

    Serial.print("Manyetometre X: ");
    Serial.print(mx, 0);
    Serial.print(" | Y: ");
    Serial.print(my, 0);
    Serial.print(" | Z: ");
    Serial.println(mz, 0);

    Serial.print("Ham pusula acisi: ");
    Serial.print(heading, 1);
    Serial.println(" derece");
  } else {
    Serial.println("GY-271 manyetometre verisi okunamadi.");
  }

  Serial.println("----------------------------------------------------");

  delay(LOOP_DELAY_MS);
}