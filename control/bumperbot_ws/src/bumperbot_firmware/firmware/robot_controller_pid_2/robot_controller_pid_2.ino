#include <util/atomic.h> //  For the ATOMIC_BLOCK macro
#include "PID.h"

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11 // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B
#define L298N_in2 13 // Dir Motor A
#define L298N_in1 12 // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4

// Encoders
unsigned int left_encoder_counter = 0;
unsigned int right_encoder_counter = 0;

unsigned int l_c = 0;
unsigned int r_c = 0;

String left_wheel_sign = "p"; // 'p' = positive, 'n' = negative
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_left_wheel_cmd = false;
bool is_right_wheel_cmd = false;
bool is_left_wheel_forward = true;
bool is_right_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
float left_wheel_cmd_vel = 0.0;     // rad/s
float right_wheel_cmd_vel = 0.0;    // rad/s
// Input - Measurement
float left_wheel_meas_vel = 0.0;    // rad/s
float right_wheel_meas_vel = 0.0;   // rad/s

// Output - Command
float left_wheel_cmd = 0.0;         // 0-255
float right_wheel_cmd = 0.0;        // 0-255

unsigned int max_pwm = 255;

long prevT = 0;

//initialiying current time
long currT = 0;

float deltaT;

// Tuning
float Kp_l = 10.0;
float Ki_l = 0.1;
float Kd_l = 0.3;
float Kp_r = 12.0;
float Ki_r = 0.9;
float Kd_r = 0.1;

PID_Custom leftMotorPID(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l);
PID_Custom rightMotorPID(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r);

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);

  // Init encoders
  pinMode(left_encoder_phaseB, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  // Reset PID controllers
  leftMotorPID.reset();
  rightMotorPID.reset();

  // Set the mode to automatic
  leftMotorPID.setMode(true);
  // Set the mode to automatic
  rightMotorPID.setMode(true);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available()) {
    char chr = Serial.read();
    // Left Wheel Motor
    if (chr == 'l') {
      is_left_wheel_cmd = true;
      is_right_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Right Wheel Motor
    else if (chr == 'r') {
      is_left_wheel_cmd = false;
      is_right_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if (chr == 'p') {
      if (is_left_wheel_cmd && !is_left_wheel_forward) {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_left_wheel_forward = true;
      } else if (is_right_wheel_cmd && !is_right_wheel_forward) {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_right_wheel_forward = true;
      }
    }
    // Negative direction
    else if (chr == 'n') {
      if (is_left_wheel_cmd && is_left_wheel_forward) {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_left_wheel_forward = false;
      } else if (is_right_wheel_cmd && is_right_wheel_forward) {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_right_wheel_forward = false;
      }
    }
    // Separator
    else if (chr == ',') {
      if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
      } else if (is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else {
      if (value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      l_c = left_encoder_counter;
      r_c = right_encoder_counter; 
    }
    deltaT = current_millis - last_millis;
    // Approximate velocities based on encoder ticks
    left_wheel_meas_vel = (10 * l_c * (60.0 / 385.0)) * 0.10472;
    right_wheel_meas_vel = (10 * r_c * (60.0 / 385.0)) * 0.10472;

    leftMotorPID.compute(deltaT);
    rightMotorPID.compute(deltaT);

    // Ignore commands smaller than inertia
    if(left_wheel_cmd_vel == 0.0)
    {
      // Reset PID controllers
      leftMotorPID.reset();
      left_wheel_cmd = 0.0;
    }
    if(right_wheel_cmd_vel == 0.0)
    {
      rightMotorPID.reset();
      right_wheel_cmd = 0.0;
    }

    String encoder_read = "l" + left_wheel_sign + String(left_wheel_meas_vel) + ",r" + right_wheel_sign + String(right_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    left_encoder_counter = 0;
    right_encoder_counter = 0;

    analogWrite(L298N_enA, left_wheel_cmd);
    analogWrite(L298N_enB, right_wheel_cmd);
  }
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "p";
  } else {
    left_wheel_sign = "n";
  }
  left_encoder_counter++;
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = "n";
  } else {
    right_wheel_sign = "p";
  }
  right_encoder_counter++;
}
