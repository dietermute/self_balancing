#include "Wire.h"

// MPU6050 interfacing code based on Dejan's article on How to Mechatronics

const int MPU_ADDR = 0x68; // I2C address of MPU6050

// vars for state estimation
float a_x, a_y, a_z;
float gyro_x, gyro_y, gyro_z;
float temp; // IMU temperature
float a_pitch;
float theta_target = -1.25; // angle where robot is balanced
float theta_est = theta_target; // angle estimate, assume starting upright
float read_time_curr = 0;
float read_time_prev = 0;
float dt_read;
float alpha = 0.001; // alpha for complementary filter

// vars for calibrating IMU
// offsets can either be hardcoded in or determined via calibration routine
float gyro_x_offset = 4.405;
float gyro_y_offset = -1.60;
float gyro_z_offset = -0.71;
float accel_x_offset = -0.045;
float accel_y_offset = -0.01;
float accel_z_offset = -0.03; 
int c = 0;
int calib_iter = 2000;
float accel_x_sum, accel_y_sum, accel_z_sum;
float accel_x_mean, accel_y_mean, accel_z_mean;
float gyro_x_sum, gyro_y_sum, gyro_z_sum;
float gyro_x_mean, gyro_y_mean, gyro_z_mean;

// vars for motor control
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 11;
int enA = 9;
int enB = 10;
int min_power = 0;

bool needs_kick = true;
int kick_delay = 3; // how long to send max power pulse

// vars for P(I)D control

// these coefficients must be tuned to your specific robot
float k_p = 15;
//float k_i = 0;
float k_d = 0.055;

float error;
float error_prev = 0;
float derivative = 0;
//float integral = 0;
float pd_time_curr = 0;
float pd_time_prev = 0;
float dt_pd;
int control = 0;
float thresh = 0.75;

// can run a calibration at startup each time by holding the robot perfectly upright, requires a few seconds before it can start moving
void calibrate() {
  while (c < calib_iter) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    a_x = (Wire.read()<<8 | Wire.read()) / 16384.0;
    a_y = (Wire.read()<<8 | Wire.read()) / 16384.0;
    a_z = (Wire.read()<<8 | Wire.read()) / 16384.0; 

    accel_x_sum += a_x;
    accel_y_sum += a_y;
    accel_z_sum += a_z;

    c ++;
  }
  accel_x_mean = accel_x_sum / calib_iter;
  accel_y_mean = accel_y_sum / calib_iter;
  accel_z_mean = accel_z_sum / calib_iter;
  
  accel_x_offset = 0 - accel_x_mean;
  accel_y_offset = 0 - accel_y_mean;
  accel_z_offset = 1 - accel_z_mean;
  
  c = 0;
  while (c < calib_iter) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gyro_x = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyro_y = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyro_z = (Wire.read() << 8 | Wire.read()) / 131.0;

    gyro_x_sum = gyro_x_sum + gyro_x;
    gyro_y_sum = gyro_y_sum + gyro_y;
    gyro_z_sum = gyro_z_sum + gyro_z;

    c++;
  }
  gyro_x_mean = gyro_x_sum / calib_iter;
  gyro_y_mean = gyro_y_sum / calib_iter;
  gyro_z_mean = gyro_z_sum / calib_iter;
  
  gyro_x_offset = 0 - gyro_x_mean;
  gyro_y_offset = 0 - gyro_y_mean;
  gyro_z_offset = 0 - gyro_z_mean;
}

float compFilter(float prev_angle, float gyro_rate, float delta_t, float accel_angle) {
  return (1-alpha) * (prev_angle + gyro_rate * delta_t) + (alpha) * a_pitch; 
}

void goForward(int power) {
  if (needs_kick == true) {
    // "kick" the motor into starting
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    delay(kick_delay);
  }
  analogWrite(9, power);
  analogWrite(10, power);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void goBackward(int power) {
  if (needs_kick == true) {
    // "kick" the motor into starting
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    delay(kick_delay);
  }
  analogWrite(9, power);
  analogWrite(10, power);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void stop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void read_IMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  a_x = ((Wire.read()<<8 | Wire.read()) / 16384.0) + accel_x_offset;
  a_y = ((Wire.read()<<8 | Wire.read()) / 16384.0) + accel_y_offset;
  a_z = ((Wire.read()<<8 | Wire.read()) / 16384.0) + accel_z_offset;
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = ((Wire.read() << 8 | Wire.read()) / 131.0) + gyro_x_offset;
  gyro_y = ((Wire.read() << 8 | Wire.read()) / 131.0) + gyro_y_offset;
  gyro_z = ((Wire.read() << 8 | Wire.read()) / 131.0) + gyro_z_offset;
}

void setup() {
  // set up IMU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  //calibrate();

  // set up motors
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void loop() {
  // STATE ESTIMATION
  read_IMU();
  a_pitch = atan2(a_y, a_z) * 180/PI; // tilt angle according to accelerometer
  read_time_curr = millis();
  dt_read = (read_time_curr - read_time_prev) / 1000;
  read_time_prev = read_time_curr;
  theta_est = compFilter(theta_est, gyro_x, dt_read, a_pitch);

  // PD CONTROL
  pd_time_curr = millis();
  dt_pd = (pd_time_curr - pd_time_prev)/1000;
  pd_time_prev = pd_time_curr;
  
  error = theta_target - theta_est;
  //integral = integral + error * dt_pd;
  derivative = (error - error_prev) / dt_pd;
  error_prev = error;

  // if using full PID, add k_i * integral
  control = int(k_p * error + k_d * derivative);
  control = constrain(control, -255, 255); // constrain to PWM range
  
  // move motors when robot is beyond "upright" threshold
  // can try and give the motors a "kickstart" if they struggle to move at low PWM
  // can also try only "kicking" whenever the motors start moving from stopped
  if (abs(error) > thresh) {
    if (control >= 0) {
      goBackward(control);
    }
    else if (control < 0) {
      goForward(abs(control));
    }
    //needs_kick = false;
  }
  else {
    stop();
    //needs_kick = true;
  }
  delay(5);
}
