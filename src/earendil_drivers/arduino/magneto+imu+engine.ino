#include <Wire.h>
#include <Servo.h>
#include <math.h>

// =======================================================
// SENSOR ADRESLERI VE AYARLARI
// =======================================================
#define MPU_ADDR_1 0x68
#define MPU_ADDR_2 0x69
#define QMC5883P_ADDR 0x2C

#define SERIAL_BAUD 115200

// ROS2 Telemetry frekansi
#define TELEMETRY_INTERVAL_MS 50
#define MOTOR_WATCHDOG_MS 700

// Pusula kalibrasyon
#define MAG_SAMPLE_COUNT 10
#define MAG_X_OFFSET -1264.50
#define MAG_Y_OFFSET 311.00
#define MAG_Z_OFFSET -73.50
#define MAG_X_SCALE 0.81943
#define MAG_Y_SCALE 0.70439
#define MAG_Z_SCALE 2.78393
#define HEADING_OFFSET_DEG 0.0
#define INVERT_HEADING false

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

const int M3_RPWM = 4;
const int M3_LPWM = 13;
const int M3_REN  = 45;
const int M3_LEN  = 52;

#define PWM_YAVAS 80
#define PWM_HIZLI 200
const int DRILL_PWM = 200;

// =======================================================
// SERVO NESNELERİ
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

int pwm1 = 45;
int pwm2 = 90;
int pwm3 = 90;
int pwm4 = 180;
int pwm5 = 90;
const int STEP = 2;

// =======================================================
// GLOBAL DURUMLAR
// =======================================================
uint8_t mpuAddr = 0;
bool mpuAvailable = false;
bool magAvailable = false;

unsigned long lastMPURetryMs = 0;
unsigned long lastMagRetryMs = 0;
#define RETRY_INTERVAL_MS 2000

unsigned long lastTelemetryMs = 0;
unsigned long lastMotorCommandMs = 0;

String motorMode = "STOP";
int currentMotorPwm = 0;
bool telemetryEnabled = true;

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
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom(addr,(uint8_t)1)!=1 || !Wire.available()) return false; 
  val=Wire.read(); 
  return true;
}
bool readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr); 
  Wire.write(reg); 
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom(addr,count)!=count) return false;
  for(uint8_t i=0; i<count; i++) { 
    if(!Wire.available()) return false; 
    dest[i]=Wire.read(); 
  } 
  return true;
}
float normalizeAngle(float angle) { 
  while(angle>=360.0) angle-=360.0; 
  while(angle<0.0) angle+=360.0; 
  return angle; 
}

// =======================================================
// GY-91 / MPU FONKSİYONLARI
// =======================================================
bool initMPU() {
  if (i2cExists(MPU_ADDR_1)) mpuAddr = MPU_ADDR_1; 
  else if (i2cExists(MPU_ADDR_2)) mpuAddr = MPU_ADDR_2; 
  else return false;

  // Wake up
  write8(mpuAddr, 0x6B, 0x00); delay(100);
  // Clock
  write8(mpuAddr, 0x6B, 0x01); delay(100);
  // DLPF
  write8(mpuAddr, 0x1A, 0x03); 
  // Gyro ±250 dps
  write8(mpuAddr, 0x1B, 0x00); 
  // Accel ±2g
  write8(mpuAddr, 0x1C, 0x00); 
  write8(mpuAddr, 0x1D, 0x03);
  
  return true;
}

bool readMPU(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps, float &temp_c) {
  if(mpuAddr==0) return false;
  uint8_t b[14]; 
  if(!readBytes(mpuAddr, 0x3B, 14, b)) return false;
  
  int16_t rawAx=((int16_t)b[0]<<8)|b[1];
  int16_t rawAy=((int16_t)b[2]<<8)|b[3];
  int16_t rawAz=((int16_t)b[4]<<8)|b[5];
  int16_t rawTemp=((int16_t)b[6]<<8)|b[7];
  int16_t rawGx=((int16_t)b[8]<<8)|b[9];
  int16_t rawGy=((int16_t)b[10]<<8)|b[11];
  int16_t rawGz=((int16_t)b[12]<<8)|b[13];
  
  ax_g = rawAx/16384.0; ay_g = rawAy/16384.0; az_g = rawAz/16384.0;
  gx_dps = rawGx/131.0; gy_dps = rawGy/131.0; gz_dps = rawGz/131.0;
  temp_c = rawTemp/333.87+21.0;
  return true;
}

// =======================================================
// QMC5883P FONKSİYONLARI (0x2C)
// =======================================================
bool initQMC5883P() {
  if (!i2cExists(QMC5883P_ADDR)) return false;
  write8(QMC5883P_ADDR, 0x29, 0x06); delay(10);
  write8(QMC5883P_ADDR, 0x0B, 0x08); delay(10);
  write8(QMC5883P_ADDR, 0x0A, 0xCD); delay(100);
  return true;
}

bool readRawQMC5883P(int16_t &rawX, int16_t &rawY, int16_t &rawZ) {
  uint8_t b[6]; 
  if(!readBytes(QMC5883P_ADDR, 0x01, 6, b)) return false;
  rawX=((int16_t)b[1]<<8)|b[0]; 
  rawY=((int16_t)b[3]<<8)|b[2]; 
  rawZ=((int16_t)b[5]<<8)|b[4]; 
  return true;
}

bool readMagHeading(float &headingDeg, float &rawXAvg, float &rawYAvg, float &rawZAvg, float &calX, float &calY, float &calZ) {
  long sumX=0, sumY=0, sumZ=0; 
  int validCount=0;
  
  for(int i=0; i<MAG_SAMPLE_COUNT; i++) {
    int16_t rx, ry, rz; 
    if(readRawQMC5883P(rx, ry, rz)) { 
      sumX+=rx; sumY+=ry; sumZ+=rz; validCount++; 
    } 
    delay(2);
  }
  
  if(validCount==0) return false;
  
  rawXAvg = sumX/(float)validCount; 
  rawYAvg = sumY/(float)validCount; 
  rawZAvg = sumZ/(float)validCount;
  
  calX = (rawXAvg-MAG_X_OFFSET)*MAG_X_SCALE; 
  calY = (rawYAvg-MAG_Y_OFFSET)*MAG_Y_SCALE; 
  calZ = (rawZAvg-MAG_Z_OFFSET)*MAG_Z_SCALE;
  
  headingDeg = atan2(calY, calX)*180.0/PI;
  headingDeg = normalizeAngle(headingDeg);
  
  if(INVERT_HEADING) headingDeg = normalizeAngle(360.0 - headingDeg);
  headingDeg = normalizeAngle(headingDeg + HEADING_OFFSET_DEG);
  return true;
}

// =======================================================
// MOTOR VE HAREKET FONKSİYONLARI
// =======================================================
void updateMotorCommandTime() { lastMotorCommandMs = millis(); }

void stopDrive() {
  analogWrite(L_RPWM,0); analogWrite(L_LPWM,0); analogWrite(R_RPWM,0); analogWrite(R_LPWM,0);
  motorMode="STOP"; currentMotorPwm=0;
}
void driveForward(int pwm) {
  pwm=constrain(pwm,0,255); analogWrite(L_RPWM,pwm); analogWrite(L_LPWM,0); analogWrite(R_RPWM,pwm); analogWrite(R_LPWM,0);
  motorMode="FWD"; currentMotorPwm=pwm;
}
void driveBackward(int pwm) {
  pwm=constrain(pwm,0,255); analogWrite(L_RPWM,0); analogWrite(L_LPWM,pwm); analogWrite(R_RPWM,0); analogWrite(R_LPWM,pwm);
  motorMode="BACK"; currentMotorPwm=pwm;
}
void tankRight(int pwm) {
  pwm=constrain(pwm,0,255); analogWrite(L_RPWM,pwm); analogWrite(L_LPWM,0); analogWrite(R_RPWM,0); analogWrite(R_LPWM,pwm);
  motorMode="RIGHT"; currentMotorPwm=pwm;
}
void tankLeft(int pwm) {
  pwm=constrain(pwm,0,255); analogWrite(L_RPWM,0); analogWrite(L_LPWM,pwm); analogWrite(R_RPWM,pwm); analogWrite(R_LPWM,0);
  motorMode="LEFT"; currentMotorPwm=pwm;
}
void sondajYukari() { analogWrite(M3_RPWM,DRILL_PWM); analogWrite(M3_LPWM,0); }
void sondajAsagi() { analogWrite(M3_RPWM,0); analogWrite(M3_LPWM,DRILL_PWM); }
void sondajDur() { analogWrite(M3_RPWM,0); analogWrite(M3_LPWM,0); }

void checkMotorWatchdog() {
  if (motorMode != "STOP" && millis() - lastMotorCommandMs > MOTOR_WATCHDOG_MS) stopDrive();
}

// =======================================================
// SERİ HABERLEŞME PARSER
// =======================================================
int parsePwmValue(String s) { s.trim(); return constrain(s.toInt(), 0, 255); }

void processSerialLine(String veri) {
  veri.trim(); 
  if(veri.length()==0) return;
  
  if(veri == "MOTOR:STOP") { stopDrive(); updateMotorCommandTime(); Serial.println("ACK,MOTOR:STOP"); return; }
  if(veri.startsWith("MOTOR:FWD:")) { driveForward(parsePwmValue(veri.substring(10))); updateMotorCommandTime(); return; }
  if(veri.startsWith("MOTOR:BACK:")) { driveBackward(parsePwmValue(veri.substring(11))); updateMotorCommandTime(); return; }
  if(veri.startsWith("MOTOR:LEFT:")) { tankLeft(parsePwmValue(veri.substring(11))); updateMotorCommandTime(); return; }
  if(veri.startsWith("MOTOR:RIGHT:")) { tankRight(parsePwmValue(veri.substring(12))); updateMotorCommandTime(); return; }
  
  if(veri == "TELEM:ON") { telemetryEnabled = true; return; }
  if(veri == "TELEM:OFF") { telemetryEnabled = false; return; }

  if(veri == "ileri_hizli") { driveForward(PWM_HIZLI); updateMotorCommandTime(); return; }
  if(veri == "ileri_yavas") { driveForward(PWM_YAVAS); updateMotorCommandTime(); return; }
  if(veri == "geri_hizli") { driveBackward(PWM_HIZLI); updateMotorCommandTime(); return; }
  if(veri == "geri_yavas") { driveBackward(PWM_YAVAS); updateMotorCommandTime(); return; }
  if(veri == "sag_hizli") { tankRight(PWM_HIZLI); updateMotorCommandTime(); return; }
  if(veri == "sag_yavas") { tankRight(PWM_YAVAS); updateMotorCommandTime(); return; }
  if(veri == "sol_hizli") { tankLeft(PWM_HIZLI); updateMotorCommandTime(); return; }
  if(veri == "sol_yavas") { tankLeft(PWM_YAVAS); updateMotorCommandTime(); return; }
  if(veri == "dur") { stopDrive(); updateMotorCommandTime(); return; }
  
  if(veri == "sondaj:yukari") { sondajYukari(); return; }
  if(veri == "sondaj:asagi") { sondajAsagi(); return; }
  if(veri == "sondaj:dur") { sondajDur(); return; }

  if(veri == "servo5:yukari") { pwm5 = constrain(pwm5+STEP,0,180); servo5.write(pwm5); return; }
  if(veri == "servo5:asagi") { pwm5 = constrain(pwm5-STEP,0,180); servo5.write(pwm5); return; }
}

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
      if (line.length() > 120) line = ""; 
    }
  }
}

// =======================================================
// TELEMETRİ YAYINI (ROS2 formatı)
// =======================================================
void publishMagTelemetry() {
  if (!telemetryEnabled) return;
  unsigned long now = millis();
  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = now;

  // IMU YAYINI
  if (!mpuAvailable && now - lastMPURetryMs > RETRY_INTERVAL_MS) { 
    lastMPURetryMs = now; 
    mpuAvailable = initMPU(); 
  }
  
  if (mpuAvailable) {
    float ax, ay, az, gx, gy, gz, t;
    if (readMPU(ax, ay, az, gx, gy, gz, t)) {
      // hardware_bridge.py beklenen IMU birimleri:
      // gx,gy,gz: rad/s  --> gx_dps * PI / 180.0
      // ax,ay,az: m/s^2  --> ax_g * 9.80665
      float rad_gx = gx * PI / 180.0;
      float rad_gy = gy * PI / 180.0;
      float rad_gz = gz * PI / 180.0;
      
      float ms2_ax = ax * 9.80665;
      float ms2_ay = ay * 9.80665;
      float ms2_az = az * 9.80665;

      Serial.print("IMU,");
      Serial.print(rad_gx, 4); Serial.print(","); 
      Serial.print(rad_gy, 4); Serial.print(","); 
      Serial.print(rad_gz, 4); Serial.print(",");
      Serial.print(ms2_ax, 4); Serial.print(","); 
      Serial.print(ms2_ay, 4); Serial.print(","); 
      Serial.println(ms2_az, 4);
    } else {
      mpuAvailable = false;
    }
  }

  // MAG YAYINI
  if (!magAvailable && now - lastMagRetryMs > RETRY_INTERVAL_MS) { 
    lastMagRetryMs = now; 
    magAvailable = initQMC5883P(); 
  }
  
  if (magAvailable) {
    float h, rx, ry, rz, cx, cy, cz;
    if (readMagHeading(h, rx, ry, rz, cx, cy, cz)) {
      // MAG,time_ms,heading,rx,ry,rz,cx,cy,cz,plane,offset,motor,pwm
      Serial.print("MAG,"); 
      Serial.print(now); Serial.print(","); 
      Serial.print(h, 2); Serial.print(",");
      Serial.print(rx, 0); Serial.print(","); 
      Serial.print(ry, 0); Serial.print(","); 
      Serial.print(rz, 0); Serial.print(",");
      Serial.print(cx, 2); Serial.print(","); 
      Serial.print(cy, 2); Serial.print(","); 
      Serial.print(cz, 2); Serial.print(",");
      Serial.print("XY,0.0,"); 
      Serial.print(motorMode); Serial.print(","); 
      Serial.println(currentMotorPwm);
    } else {
      magAvailable = false;
    }
  }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Wire.begin();
  Wire.setClock(100000);

  // Motor ve Sondaj Pinleri
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);
  pinMode(M3_RPWM, OUTPUT); pinMode(M3_LPWM, OUTPUT); pinMode(M3_REN, OUTPUT); pinMode(M3_LEN, OUTPUT);
  
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
  digitalWrite(M3_REN, HIGH); digitalWrite(M3_LEN, HIGH);
  stopDrive(); sondajDur();

  // Servolar
  servo1.attach(SERVO_PIN1); servo2.attach(SERVO_PIN2); servo3.attach(SERVO_PIN3); 
  servo4.attach(SERVO_PIN4); servo5.attach(SERVO_PIN5);
  servo1.write(pwm1); servo2.write(pwm2); servo3.write(pwm3); 
  servo4.write(pwm4); servo5.write(pwm5);

  Serial.println("WARN,Arduino Motor + IMU (0x2C QMC5883P) ROS2 Bridge Basladi");

  mpuAvailable = initMPU();
  if(!mpuAvailable) Serial.println("ERR,GY-91 IMU bulunamadi.");
  
  magAvailable = initQMC5883P();
  if(!magAvailable) Serial.println("ERR,QMC5883P 0x2C pusula bulunamadi.");

  lastMotorCommandMs = millis();
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  handleSerialInput();
  checkMotorWatchdog();
  publishMagTelemetry();
}