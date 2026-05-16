#include <Servo.h>

  // === Motor Sürücü Pinleri ===
  const int L_RPWM = 5,  L_LPWM = 6,  L_REN = 7,  L_LEN = 8;
  const int R_RPWM = 9,  R_LPWM = 10, R_REN = 11, R_LEN = 12;
  const int M3_RPWM = 4, M3_LPWM = 13, M3_REN = 45, M3_LEN = 52;

  // === PWM Kademeleri ===
  #define PWM_YAVAS 80
  #define PWM_HIZLI 200
  const int DRILL_PWM = 200;

  // === Servo Nesneleri ===
  Servo servo1; // Çok turlu (X3 ? pin 26)
  Servo servo2; // Çok turlu (Y3 ? pin 24)
  Servo servo3; // Normal joystick (X2 ? pin 32)
  Servo servo4; // Normal joystick (Y2 ? pin 28)
  Servo servo5; // Buton ile kontrol edilen servo (pin 34)

  const int SERVO_PIN1 = 22;
  const int SERVO_PIN2 = 24;
  const int SERVO_PIN3 = 26;
  const int SERVO_PIN4 = 28;
  const int SERVO_PIN5 = 44;

  // === Servo1/2 çok turlu ayarlar? ===
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

  // === Ba?lang?ç ===
  int pwm1 = 45, pwm2 = 90;
  int turSayisi1 = 2, turSayisi2 = 2;
  int pwm3 = 90, pwm4 = 180;
  int pwm5 = 90;

  // === E?ikler ===
  const int JOY_CENTER = 2048;
  const int DEADZONE   = 400;
  const int STEP       = 2;

  void setup() {
    Serial.begin(115200);

    pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
    pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
    pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
    pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
    pinMode(M3_RPWM, OUTPUT); pinMode(M3_LPWM, OUTPUT);
    pinMode(M3_REN, OUTPUT);  pinMode(M3_LEN, OUTPUT);
    digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
    digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
    digitalWrite(M3_REN, HIGH); digitalWrite(M3_LEN, HIGH);

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
  }

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

  void loop() {
    if (Serial.available()) {
      String veri = Serial.readStringUntil('\n');
      veri.trim();

      // === Yön komutlar? ===
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

      // === Sondaj komutlar? ===
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