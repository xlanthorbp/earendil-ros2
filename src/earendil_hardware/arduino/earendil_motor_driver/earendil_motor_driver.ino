/*
 * Earendil Bot - Arduino Mega Motor Driver (BTS7960)
 * 
 * Protocol:
 * "m <left_rads> <right_rads>\n" -> Sets target speeds.
 * "e\n" -> Arduino responds with "E <left_ticks> <right_ticks>\n"
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

// Encoders (Mega has interrupts on 2, 3, 18, 19, 20, 21)
const int ENC_L_A = 2;
const int ENC_L_B = 3;
const int ENC_R_A = 18;
const int ENC_R_B = 19;

// ==========================================
// VARIABLES
// ==========================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// PID Variables
float target_left_rads = 0.0;
float target_right_rads = 0.0;

long prev_left_ticks = 0;
long prev_right_ticks = 0;

// You MUST tune these PID values for your specific motors!
float Kp = 15.0; 
float Ki = 0.5;
float Kd = 0.1;

float err_sum_l = 0;
float err_last_l = 0;
float err_sum_r = 0;
float err_last_r = 0;

unsigned long last_pid_time = 0;
const int PID_INTERVAL_MS = 20; // 50Hz control loop

const float TICKS_PER_REV = 341.0; // Update this to your real encoder resolution

// Serial Parsing
String inputString = "";
boolean stringComplete = false;
unsigned long last_cmd_time = 0;

// ==========================================
// SETUP
// ==========================================
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

  // Encoder Pins
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // 1. Process incoming Serial data
  if (stringComplete) {
    parseSerialCommand();
    inputString = "";
    stringComplete = false;
  }

  // 2. Failsafe: Stop motors if no command received for 1 second
  if (millis() - last_cmd_time > 1000) {
    target_left_rads = 0.0;
    target_right_rads = 0.0;
  }

  // 3. Run PID Control Loop at 50Hz
  if (millis() - last_pid_time >= PID_INTERVAL_MS) {
    runPID();
    last_pid_time = millis();
  }
}

// ==========================================
// SERIAL EVENT & PARSING
// ==========================================
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
  if (inputString.startsWith("e")) {
    // Send encoder data back to Raspberry Pi
    Serial.print("E ");
    Serial.print(left_ticks);
    Serial.print(" ");
    Serial.println(right_ticks);
  } 
  else if (inputString.startsWith("m")) {
    // Parse target speeds: "m <left_rads> <right_rads>"
    int firstSpace = inputString.indexOf(' ');
    int secondSpace = inputString.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String leftStr = inputString.substring(firstSpace + 1, secondSpace);
      String rightStr = inputString.substring(secondSpace + 1);
      
      target_left_rads = leftStr.toFloat();
      target_right_rads = rightStr.toFloat();
      last_cmd_time = millis();
    }
  }
}

// ==========================================
// PID CONTROL
// ==========================================
void runPID() {
  // Calculate current velocity in rad/s
  long curr_left_ticks = left_ticks;
  long curr_right_ticks = right_ticks;
  
  float left_rads = ((curr_left_ticks - prev_left_ticks) / TICKS_PER_REV) * 2.0 * PI * (1000.0 / PID_INTERVAL_MS);
  float right_rads = ((curr_right_ticks - prev_right_ticks) / TICKS_PER_REV) * 2.0 * PI * (1000.0 / PID_INTERVAL_MS);
  
  prev_left_ticks = curr_left_ticks;
  prev_right_ticks = curr_right_ticks;

  // Left PID
  float err_l = target_left_rads - left_rads;
  err_sum_l += err_l;
  float d_err_l = err_l - err_last_l;
  err_last_l = err_l;
  float left_pwm = (Kp * err_l) + (Ki * err_sum_l) + (Kd * d_err_l);

  // Right PID
  float err_r = target_right_rads - right_rads;
  err_sum_r += err_r;
  float d_err_r = err_r - err_last_r;
  err_last_r = err_r;
  float right_pwm = (Kp * err_r) + (Ki * err_sum_r) + (Kd * d_err_r);

  // Stop integral windup when targeting 0
  if (target_left_rads == 0) { left_pwm = 0; err_sum_l = 0; }
  if (target_right_rads == 0) { right_pwm = 0; err_sum_r = 0; }

  setMotorPWM(left_pwm, right_pwm);
}

// ==========================================
// MOTOR DRIVER ABSTRACTION
// ==========================================
void setMotorPWM(float left, float right) {
  // Constrain to 8-bit PWM limits (-255 to 255)
  int pwm_l = constrain((int)left, -255, 255);
  int pwm_r = constrain((int)right, -255, 255);

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

// ==========================================
// INTERRUPT SERVICE ROUTINES (Encoders)
// ==========================================
void leftEncoderISR() {
  if (digitalRead(ENC_L_B) == HIGH) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void rightEncoderISR() {
  if (digitalRead(ENC_R_B) == HIGH) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}
