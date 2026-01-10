#include <SPI.h>
#include <AS5047P.h>
#include "DShot.h"

// ---------------- Motors ----------------
DShot motor_yaw(&Serial1, DShotType::DShot300); // pin 1
DShot motor1(&Serial2, DShotType::DShot300);    // pin 8
DShot motor2(&Serial6, DShotType::DShot300);    // pin 24
DShot motor3(&Serial7, DShotType::DShot300);    // pin 29

// ---------------- Encoder ----------------
AS5047P as5047p(10, 10000000);

// ---------------- RC input pins ----------------
const int ch1Pin = 15; // throttle
const int ch2Pin = 16; // roll
const int ch3Pin = 17; // pitch
const int ch4Pin = 18; // yaw

unsigned long channel_1_pwm = 1000;
unsigned long channel_2_pwm = 1500;
unsigned long channel_3_pwm = 1500;
unsigned long channel_4_pwm = 1000;

const unsigned long channel_1_fs = 1000;
const unsigned long channel_2_fs = 1500;
const unsigned long channel_3_fs = 1500;
const unsigned long channel_4_fs = 1000;

// ---------------- Desired states ----------------
float thro_des, roll_des, pitch_des, yaw_des;
float encoder_angle_deg;
float encoder_angle_rad;

// ---------------- Parameters ----------------
const float MAX_THRUST = 0.9f;   // limit thrust
const float CYCLIC_GAIN = 0.4f;  // roll/pitch authority

void setup() {
  Serial.begin(115200);
  delay(200);

  radioSetup();

  if (!as5047p.initSPI()) {
    Serial.println("AS5047P init failed");
    while (true) delay(1000);
  }


  //setZeroAtCurrentPosition();


  // ESC init
  for (int i = 0; i < 4000; i++) {
    motor_yaw.sendCommand(0, false);
    motor1.sendCommand(0, false);
    motor2.sendCommand(0, false);
    motor3.sendCommand(0, false);
    delayMicroseconds(1000);
  }



}

void loop() {
  getCommands();
  failSafe();
  getDesState();

  encoder_angle_deg = as5047p.readAngleDegree(true);

  float angle_ccw_positive = 360.0f - encoder_angle_deg;
  if (angle_ccw_positive >= 360.0f) angle_ccw_positive -= 360.0f;

  encoder_angle_rad = angle_ccw_positive * DEG_TO_RAD;

  // Motor arm angles (radians)
  const float m1_ref = 0.0f * DEG_TO_RAD;
  const float m2_ref = 120.0f * DEG_TO_RAD;
  const float m3_ref = 240.0f * DEG_TO_RAD;

  //Cyclic mixing
  float m1 = thro_des
           - CYCLIC_GAIN * roll_des  * sin(m1_ref - encoder_angle_rad)
           - CYCLIC_GAIN * pitch_des * cos(m1_ref - encoder_angle_rad);

  float m2 = thro_des
           - CYCLIC_GAIN * roll_des  * sin(m2_ref - encoder_angle_rad)
           - CYCLIC_GAIN * pitch_des * cos(m2_ref - encoder_angle_rad);

  float m3 = thro_des
           - CYCLIC_GAIN * roll_des  * sin(m3_ref - encoder_angle_rad)
           - CYCLIC_GAIN * pitch_des * cos(m3_ref - encoder_angle_rad);

  // Clamp
  m1 = constrain(m1, 0.0f, MAX_THRUST);
  m2 = constrain(m2, 0.0f, MAX_THRUST);
  m3 = constrain(m3, 0.0f, MAX_THRUST);


  // Convert to DShot (0–1999)
  uint16_t d1 = (uint16_t)(m1 * 1999.0f);
  uint16_t d2 = (uint16_t)(m2 * 1999.0f);
  uint16_t d3 = (uint16_t)(m3 * 1999.0f);

  uint16_t dy = (uint16_t)channel_4_pwm;

  //Throttle cut
  if (thro_des < 0.05f) {
    d1 = d2 = d3 = dy = 0;
  }

  motor1.sendThrottle(d1, false);
  motor2.sendThrottle(d2, false);
  motor3.sendThrottle(d3, false);
  motor_yaw.sendThrottle(dy, false);

  // Debug print
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    // Serial.print("Enc: "); Serial.print(angle_ccw_positive);
    Serial.print(" M1: "); Serial.print(d1);
    Serial.print(" M2: "); Serial.print(d2);
    Serial.print(" M3: "); Serial.print(d3);
    Serial.print(" yaw_motor: "); Serial.println(dy);

  }

}

// ---------------- Helpers ----------------
void failSafe() {
  if (channel_1_pwm < 800 || channel_1_pwm > 2200 ||
      channel_2_pwm < 800 || channel_2_pwm > 2200 ||
      channel_3_pwm < 800 || channel_3_pwm > 2200 ||
      channel_4_pwm < 800 || channel_4_pwm > 2200) {

    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
  }
  // Serial.println("failsafe active");
}

void getDesState() {
  thro_des  = constrain((channel_1_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
  roll_des  = constrain((channel_2_pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
  pitch_des = constrain((channel_3_pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
  yaw_des   = constrain((channel_4_pwm - 1500.0f) / 500.0f, 0.0f, 1.0f);
}

bool setZeroAtCurrentPosition()
{
  AS5047P_Types::ERROR_t err;

  // Read current angle as 14-bit value (0..16383).
  // Many libraries expose readAngleRaw(); if yours doesn’t, see note below.
  uint16_t angle14 = as5047p.readAngleRaw(true) & 0x3FFF;

  AS5047P_Types::ZPOSM_t zposm;
  AS5047P_Types::ZPOSL_t zposl;

  // Split 14-bit angle into MSB/LSB parts as required by the sensor.
  zposl.data.raw = (angle14 & 0x003F);           // bits [5:0]
  zposm.data.raw = ((angle14 >> 6) & 0x00FF);    // bits [13:6]

  // Write MSB then LSB (either order typically works, but this is common practice).
  bool ok = true;
  ok &= as5047p.write_ZPOSM(&zposm, &err, true, true);
  ok &= as5047p.write_ZPOSL(&zposl, &err, true, true);

  if (!ok) {
    Serial.print("ZPOS write failed. Error: ");
    Serial.println(err.toArduinoString());
  }
  return ok;
}

