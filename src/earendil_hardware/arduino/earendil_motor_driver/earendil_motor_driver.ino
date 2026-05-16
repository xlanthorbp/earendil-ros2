/*
 * Earendil Bot - Simple Arduino Mega Motor Driver (No Encoders)
 * 
 * Protocol:
 * "m <left_pwm> <right_pwm>\n" -> Sets raw PWM speeds (-255 to 255).
 */

// ==========================================
// PIN CONFIGURATIONS
// ==========================================
// BTS7960 Left Motor
const int L_EN = 8;
const int R_EN = 9;
const int L_PWM1 = 10; // Forward PWM
const int L_PWM2 = 11; // Reverse PWM

// BTS7960 Right Motor
const int R_EN2 = 6;
const int L_EN2 = 7;
const int R_PWM1 = 4; // Forward PWM
const int R_PWM2 = 5; // Reverse PWM

// Serial Parsing
String inputString = "";
boolean stringComplete = false;
unsigned long last_cmd_time = 0;

void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  // Motor Pins Setup
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT);
  pinMode(L_PWM1, OUTPUT); pinMode(L_PWM2, OUTPUT);
  
  pinMode(L_EN2, OUTPUT); pinMode(R_EN2, OUTPUT);
  pinMode(R_PWM1, OUTPUT); pinMode(R_PWM2, OUTPUT);

  // Enable all BTS7960 half-bridges
  digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN2, HIGH); digitalWrite(R_EN2, HIGH);
}

void loop() {
  if (stringComplete) {
    parseSerialCommand();
    inputString = "";
    stringComplete = false;
  }

  // Failsafe: Stop motors if no command received for 1 second
  if (millis() - last_cmd_time > 1000) {
    setMotorPWM(0, 0);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseSerialCommand() {
  if (inputString.startsWith("m")) {
    int firstSpace = inputString.indexOf(' ');
    int secondSpace = inputString.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String leftStr = inputString.substring(firstSpace + 1, secondSpace);
      String rightStr = inputString.substring(secondSpace + 1);
      
      int target_left_pwm = leftStr.toInt();
      int target_right_pwm = rightStr.toInt();
      
      setMotorPWM(target_left_pwm, target_right_pwm);
      last_cmd_time = millis();
    }
  }
}

void setMotorPWM(int pwm_l, int pwm_r) {
  pwm_l = constrain(pwm_l, -255, 255);
  pwm_r = constrain(pwm_r, -255, 255);

  // Left Motor
  if (pwm_l > 0) {
    analogWrite(L_PWM1, pwm_l);
    analogWrite(L_PWM2, 0);
  } else if (pwm_l < 0) {
    analogWrite(L_PWM1, 0);
    analogWrite(L_PWM2, -pwm_l);
  } else {
    analogWrite(L_PWM1, 0);
    analogWrite(L_PWM2, 0);
  }

  // Right Motor
  if (pwm_r > 0) {
    analogWrite(R_PWM1, pwm_r);
    analogWrite(R_PWM2, 0);
  } else if (pwm_r < 0) {
    analogWrite(R_PWM1, 0);
    analogWrite(R_PWM2, -pwm_r);
  } else {
    analogWrite(R_PWM1, 0);
    analogWrite(R_PWM2, 0);
  }
}
