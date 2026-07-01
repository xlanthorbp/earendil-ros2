#include <Servo.h>
#include <Wire.h>
#include <math.h>

  // === Motor Sürücü Pinleri ===
  const int L_RPWM = 5,  L_LPWM = 6,  L_REN = 7,  L_LEN = 8;
  const int R_RPWM = 9,  R_LPWM = 10, R_REN = 11, R_LEN = 12;
  const int M3_RPWM = 4, M3_LPWM = 13, M3_REN = 45, M3_LEN = 52;

  // === PWM Kademeleri ===
  #define PWM_YAVAS 80
  #define PWM_HIZLI 200
  const int DRILL_PWM = 200;

  // === Servo Nesneleri ===
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

  // === Servo1/2 çok turlu ayarlari ===
  const int SAFE_MIN             = 40;
  const int SAFE_MAX             = 140;
  const int SAFE_MAX1_SECOND     = 45;

  const int TUR_MS               = 400;
  const int TUR_PWM_YUKARI       = 180;
  const int TUR_PWM_ASAGI        = 0;
  const int SONRAKI_YUK_PWM1     = 46;
  const int SONRAKI_ASG_PWM1     = 134;
  const int SONRAKI_YUK_PWM2     = 46;
  const int SONRAKI_ASG_PWM2     = 134;
  const int MAX_TUR              = 2;

  // === Baslangic ===
  int pwm1 = 45, pwm2 = 90;
  int turSayisi1 = 2, turSayisi2 = 2;
  int pwm3 = 90, pwm4 = 180;
  int pwm5 = 90;

  // === Esikler ===
  const int JOY_CENTER = 2048;
  const int DEADZONE   = 400;
  const int STEP       = 2;

  // ===================================================================
  // IMU SECTION (GY-91 MPU + GY-271 Magnetometer)
  // ===================================================================
  #define MPU_ADDR 0x68
  #define QMC5883_ADDR 0x0D
  #define HMC5883_ADDR 0x1E

  #define IMU_SEND_INTERVAL_MS 50   // Send IMU data every 50ms (20 Hz)

  enum MagType { MAG_NONE, MAG_QMC5883L, MAG_HMC5883L };
  MagType magType = MAG_NONE;
  bool mpuReady = false;

  // Mag calibration offsets (update after calibration)
  float MAG_X_OFFSET = 0;
  float MAG_Y_OFFSET = 0;

  unsigned long lastImuSendTime = 0;

  // --- I2C helpers ---
  bool i2cExists(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

  void imuWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  void imuReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, count);
    uint8_t i = 0;
    while (Wire.available() && i < count) {
      dest[i++] = Wire.read();
    }
  }

  // --- MPU init ---
  void initMPU() {
    if (!i2cExists(MPU_ADDR)) { return; }
    imuWrite8(MPU_ADDR, 0x6B, 0x00); // Wake up
    delay(100);
    imuWrite8(MPU_ADDR, 0x1A, 0x03); // DLPF
    imuWrite8(MPU_ADDR, 0x1B, 0x00); // Gyro ±250 dps
    imuWrite8(MPU_ADDR, 0x1C, 0x00); // Accel ±2g
    mpuReady = true;
  }

  // --- Magnetometer init ---
  void initMag() {
    if (i2cExists(QMC5883_ADDR)) {
      magType = MAG_QMC5883L;
      imuWrite8(QMC5883_ADDR, 0x0B, 0x01);
      imuWrite8(QMC5883_ADDR, 0x09, 0x1D);
    } else if (i2cExists(HMC5883_ADDR)) {
      magType = MAG_HMC5883L;
      imuWrite8(HMC5883_ADDR, 0x00, 0x70);
      imuWrite8(HMC5883_ADDR, 0x01, 0x20);
      imuWrite8(HMC5883_ADDR, 0x02, 0x00);
    }
  }

  // --- Read and send IMU data ---
  // Format: IMU,heading,ax,ay,az,gx,gy,gz\n
  void sendImuData() {
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float heading = -1;

    // Read MPU (accel + gyro)
    if (mpuReady) {
      uint8_t b[14];
      imuReadBytes(MPU_ADDR, 0x3B, 14, b);
      int16_t rax = ((int16_t)b[0] << 8) | b[1];
      int16_t ray = ((int16_t)b[2] << 8) | b[3];
      int16_t raz = ((int16_t)b[4] << 8) | b[5];
      int16_t rgx = ((int16_t)b[8] << 8) | b[9];
      int16_t rgy = ((int16_t)b[10] << 8) | b[11];
      int16_t rgz = ((int16_t)b[12] << 8) | b[13];
      ax = rax / 16384.0;
      ay = ray / 16384.0;
      az = raz / 16384.0;
      gx = rgx / 131.0;
      gy = rgy / 131.0;
      gz = rgz / 131.0;
    }

    // Read Magnetometer (heading)
    if (magType == MAG_QMC5883L) {
      uint8_t b[6];
      imuReadBytes(QMC5883_ADDR, 0x00, 6, b);
      float mx = (float)(((int16_t)b[1] << 8) | b[0]) - MAG_X_OFFSET;
      float my = (float)(((int16_t)b[3] << 8) | b[2]) - MAG_Y_OFFSET;
      heading = atan2(my, mx) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
    } else if (magType == MAG_HMC5883L) {
      uint8_t b[6];
      imuReadBytes(HMC5883_ADDR, 0x03, 6, b);
      float mx = (float)(((int16_t)b[0] << 8) | b[1]) - MAG_X_OFFSET;
      float my = (float)(((int16_t)b[4] << 8) | b[5]) - MAG_Y_OFFSET;
      heading = atan2(my, mx) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
    }

    // Send as CSV: IMU,heading,ax,ay,az,gx,gy,gz
    Serial.print("IMU,");
    Serial.print(heading, 2);
    Serial.print(",");
    Serial.print(ax, 4); Serial.print(",");
    Serial.print(ay, 4); Serial.print(",");
    Serial.print(az, 4); Serial.print(",");
    Serial.print(gx, 2); Serial.print(",");
    Serial.print(gy, 2); Serial.print(",");
    Serial.print(gz, 2);
    Serial.println();
  }

  // ===================================================================
  // MOTOR FUNCTIONS
  // ===================================================================
  void dur() {
    analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
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

  // ===================================================================
  // SETUP
  // ===================================================================
  void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
    pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
    pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
    pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
    pinMode(M3_RPWM, OUTPUT); pinMode(M3_LPWM, OUTPUT);
    pinMode(M3_REN, OUTPUT);  pinMode(M3_LEN, OUTPUT);
    digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
    digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
    digitalWrite(M3_REN, HIGH); digitalWrite(M3_LEN, HIGH);

    // Servos
    servo1.attach(SERVO_PIN1);
    servo2.attach(SERVO_PIN2);
    servo3.attach(SERVO_PIN3);
    servo4.attach(SERVO_PIN4);
    servo5.attach(SERVO_PIN5);

    servo1.write(pwm3);
    servo2.write(pwm2);
    servo3.write(pwm1);
    servo4.write(pwm4);
    servo5.write(pwm5);

    // IMU sensors (I2C)
    Wire.begin();          // Mega: SDA=20, SCL=21
    Wire.setClock(100000);
    initMPU();
    initMag();
  }

  // ===================================================================
  // LOOP
  // ===================================================================
  void loop() {

    // --- Send IMU data at 20 Hz ---
    unsigned long now = millis();
    if (now - lastImuSendTime >= IMU_SEND_INTERVAL_MS) {
      sendImuData();
      lastImuSendTime = now;
    }

    // --- Process incoming motor/servo commands from Pi ---
    if (Serial.available()) {
      String veri = Serial.readStringUntil('\n');
      veri.trim();

      // === Yön komutlari ===
      if (veri == "ileri_hizli") {
        analogWrite(L_RPWM, PWM_HIZLI); analogWrite(L_LPWM, 0);
        analogWrite(R_RPWM, PWM_HIZLI); analogWrite(R_LPWM, 0);
      }
      else if (veri == "ileri_yavas") {
        analogWrite(L_RPWM, PWM_YAVAS); analogWrite(L_LPWM, 0);
        analogWrite(R_RPWM, PWM_YAVAS); analogWrite(R_LPWM, 0);
      }
      else if (veri == "geri_hizli") {
        analogWrite(L_RPWM, 0); analogWrite(L_LPWM, PWM_HIZLI);
        analogWrite(R_RPWM, 0); analogWrite(R_LPWM, PWM_HIZLI);
      }
      else if (veri == "geri_yavas") {
        analogWrite(L_RPWM, 0); analogWrite(L_LPWM, PWM_YAVAS);
        analogWrite(R_RPWM, 0); analogWrite(R_LPWM, PWM_YAVAS);
      }
      else if (veri == "sag_hizli") {
        analogWrite(L_RPWM, PWM_HIZLI); analogWrite(L_LPWM, 0);
        analogWrite(R_RPWM, 0);         analogWrite(R_LPWM, PWM_HIZLI);
      }
      else if (veri == "sag_yavas") {
        analogWrite(L_RPWM, PWM_YAVAS); analogWrite(L_LPWM, 0);
        analogWrite(R_RPWM, 0);         analogWrite(R_LPWM, PWM_YAVAS);
      }
      else if (veri == "sol_hizli") {
        analogWrite(L_RPWM, 0);         analogWrite(L_LPWM, PWM_HIZLI);
        analogWrite(R_RPWM, PWM_HIZLI); analogWrite(R_LPWM, 0);
      }
      else if (veri == "sol_yavas") {
        analogWrite(L_RPWM, 0);         analogWrite(L_LPWM, PWM_YAVAS);
        analogWrite(R_RPWM, PWM_YAVAS); analogWrite(R_LPWM, 0);
      }
      else if (veri == "dur") dur();

      // === Sondaj komutlari ===
      else if (veri == "sondaj:yukari") sondajYukari();
      else if (veri == "sondaj:asagi") sondajAsagi();
      else if (veri == "sondaj:dur") sondajDur();

      // === Servo5 buton kontrolü ===
      else if (veri == "servo5:yukari") {
        pwm5 += STEP;
        pwm5 = constrain(pwm5, 0, 180);
        servo5.write(pwm5);
      }
      else if (veri == "servo5:asagi") {
        pwm5 -= STEP;
        pwm5 = constrain(pwm5, 0, 180);
        servo5.write(pwm5);
      }

      // === Normal servo (x2, y2, y3, x3) ===
      else if (veri.startsWith("y2:")) {
        int deger = veri.substring(3).toInt();
        int diff = deger - JOY_CENTER;
        if (abs(diff) > DEADZONE) {
          pwm3 += (diff > 0) ? STEP : -STEP;
          pwm3 = constrain(pwm3, 0, 180);
          servo3.write(pwm3);
        }
      }

      else if (veri.startsWith("y3:")) {
        int deger = veri.substring(3).toInt();
        int diff = deger - JOY_CENTER;
        if (abs(diff) > DEADZONE) {
          pwm4 += (diff > 0) ? STEP : -STEP;
          pwm4 = constrain(pwm4, 0, 180);
          servo4.write(pwm4);
        }
      }

      // === Çok turlu servo1 ===
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
            } else pwm1 += STEP;
          } else {
            if (pwm1 <= SAFE_MIN) {
              if (turSayisi1 > 1) {
                servo1.write(TUR_PWM_ASAGI);
                delay(TUR_MS);
                pwm1 = SONRAKI_ASG_PWM1;
                turSayisi1--;
              }
            } else pwm1 -= STEP;
          }
          int safeMax1 = (turSayisi1 == MAX_TUR) ? SAFE_MAX1_SECOND : SAFE_MAX;
          pwm1 = constrain(pwm1, SAFE_MIN, safeMax1);
          servo1.write(pwm1);
        }
      }

      // === Çok turlu servo2 ===
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
            } else pwm2 += STEP;
          } else {
            if (pwm2 <= SAFE_MIN) {
              if (turSayisi2 > 1) {
                servo2.write(TUR_PWM_ASAGI);
                delay(TUR_MS);
                pwm2 = SONRAKI_ASG_PWM2;
                turSayisi2--;
              }
            } else pwm2 -= STEP;
          }
          pwm2 = constrain(pwm2, SAFE_MIN, SAFE_MAX);
          servo2.write(pwm2);
        }
      }
    }
  }