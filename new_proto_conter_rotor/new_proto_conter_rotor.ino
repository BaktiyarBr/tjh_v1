#include <Arduino.h>
#include <SPI.h>
#include <AS5047P.h>

#include "src/ESCPID/ESCCMD.h"

// ---------------- Motors ----------------
static const uint8_t N_ESC = 6;  // must be >= 5 to access motor #5 (index 4)

// Only using motors #2, #3, #5
static const uint8_t M2_IDX = 1; // output #2 -> pin 8
static const uint8_t M3_IDX = 2; // output #3 -> pin 24
static const uint8_t M5_IDX = 4; // output #5 -> pin 23

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

// indices for motors 2,3,5
static const uint8_t M2 = 1;
static const uint8_t M3 = 2;
static const uint8_t M5 = 4;
float angle_ccw_positive;

// ---------------- Desired states ----------------
float thro_des, roll_des, pitch_des, yaw_des;
float encoder_angle_deg;
float encoder_angle_rad;

// ---------------- Parameters ----------------
const float MAX_THRUST   = 0.9f;  // limit thrust
const float CYCLIC_GAIN  = 0.4f;  // roll/pitch authority
const float THRO_CUT     = 0.02f; // below this -> stop motors

// ---------------- Timing / loop ----------------
float dt;
unsigned long current_time = 0, prev_time = 0;



// ---------- External functions you already have ----------
void radioSetup();
void getCommands();
// ---------------------------------------------------------

float encoder_zero_offset = 177.49;

void setup() {
  Serial.begin(115200);
  delay(200);

  radioSetup();

  if (!as5047p.initSPI()) {
    Serial.println("AS5047P init failed");
    while (true) delay(1000);
  }

  ESCCMD_init(N_ESC);
  Serial.println(ESCCMD_arm_all());
  Serial.println(ESCCMD_start_timer());

  // force STOP frames for 2 seconds
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    ESCCMD_stop(M2);
    ESCCMD_stop(M3);
    ESCCMD_stop(M5);
    ESCCMD_tic();
  }

  ESCCMD_start_timer();

  // Stop all outputs (then we only command 2/3/5 in the loop)
  for (uint8_t i = 0; i < N_ESC; i++) ESCCMD_stop(i);

  // Initialize timing so first dt is sane
  current_time = micros();
  prev_time = current_time;

  Serial.println("Setup complete. Controlling motors 2,3,5 (idx 1,2,4).");

  
  // calibrate_encoder();
}


void loop() {
  ESCCMD_tic();
  getCommands();
  failSafe();
  getDesState();

  encoder_angle_deg = as5047p.readAngleDegree(true);

  // Convert to CCW-positive angle first
  angle_ccw_positive = 360.0f - encoder_angle_deg;
  if (angle_ccw_positive >= 360.0f) 
    angle_ccw_positive -= 360.0f;
  angle_ccw_positive -= encoder_zero_offset;
  if (angle_ccw_positive < 0.0f) 
    angle_ccw_positive += 360.0f;
  else if (angle_ccw_positive >= 360.0f) 
    angle_ccw_positive -= 360.0f;

  encoder_angle_rad = angle_ccw_positive * DEG_TO_RAD;

  // Motor arm angles (radians)
  const float m1_ref = 0.0f   * DEG_TO_RAD;
  const float m2_ref = 120.0f * DEG_TO_RAD;
  const float m3_ref = 240.0f * DEG_TO_RAD;
  
  float gyro_phase_offset_deg = 90.0f;   // "90 degrees prior" for CCW-increasing angle
  float gyro_phase_offset_rad = gyro_phase_offset_deg * DEG_TO_RAD;

  // Precompute shifted angles (optional, but clearer)
  float a1 = wrap_2pi((m1_ref + encoder_angle_rad) + gyro_phase_offset_rad);
  float a2 = wrap_2pi((m2_ref + encoder_angle_rad) + gyro_phase_offset_rad);
  float a3 = wrap_2pi((m3_ref + encoder_angle_rad) + gyro_phase_offset_rad);

  // Cyclic mixing (with precession phase offset)
  float m1 = thro_des
          + CYCLIC_GAIN * roll_des  * sinf(a1)
          - CYCLIC_GAIN * pitch_des * cosf(a1);

  float m2 = thro_des
          + CYCLIC_GAIN * roll_des  * sinf(a2)
          - CYCLIC_GAIN * pitch_des * cosf(a2);

  float m3 = thro_des
          + CYCLIC_GAIN * roll_des  * sinf(a3)
          - CYCLIC_GAIN * pitch_des * cosf(a3);
          

  // Clamp thrust commands
  m1 = constrain(m1, 0.0f, MAX_THRUST);
  m2 = constrain(m2, 0.0f, MAX_THRUST);
  m3 = constrain(m3, 0.0f, MAX_THRUST);

  // Convert to ESCCMD throttle scale 0..1999
  // Map your (0..MAX_THRUST) range into (0..1999*MAX_THRUST)
  uint16_t d1 = clamp_0_1999(m1 * 1999.0f);
  uint16_t d2 = clamp_0_1999(m2 * 1999.0f);
  uint16_t d3 = clamp_0_1999(m3 * 1999.0f);

  if (thro_des < 0.02f) {
    ESCCMD_throttle(M2, 0);
    ESCCMD_throttle(M5, 0);
    ESCCMD_throttle(M3, 0);
  } else {
    ESCCMD_throttle(M2, d1 + 48);
    ESCCMD_throttle(M5, d2 + 48);
    ESCCMD_throttle(M3, d3 + 48);
  }

  // // Debug print
  // // Serial Plotter-friendly output: one line = one sample, numbers only.
  static uint32_t lastPrint = 0;
  // static bool headerPrinted = false;

  // if (!headerPrinted) {
  //   // Arduino IDE Serial Plotter can show this as legend in newer IDEs; harmless otherwise.
  //   Serial.println("m1_deg m2_deg m3_deg d_m2 d_m3 d_m5");
  //   headerPrinted = true;
  // }

  // if (millis() - lastPrint >= 20) {
  //   lastPrint += 20; // keeps a steadier interval than setting to millis()

  //   Serial.print(a1 * RAD_TO_DEG); Serial.print(' ');
  //   Serial.print(a2 * RAD_TO_DEG); Serial.print(' ');
  //   Serial.print(a3 * RAD_TO_DEG); Serial.print(' ');

  //   Serial.print(d1); Serial.print(' ');
  //   Serial.print(d2); Serial.print(' ');
  //   Serial.println(d3);
  // }

  if (millis() - lastPrint > 20) {
    lastPrint = millis();
    Serial.print("Enc: m1 pos "); Serial.print(a1 * RAD_TO_DEG);
    Serial.print("  Enc: m2 pos "); Serial.print(a2 * RAD_TO_DEG);
    Serial.print("  Enc: m3 pos "); Serial.print(a3 * RAD_TO_DEG);

    Serial.print("  d(m2): "); Serial.print(d1);
    Serial.print("  d(m3): "); Serial.print(d2);
    Serial.print("  d(m5): "); Serial.println(d3);
  }

}

void calibrate_encoder() {
  for (int i = 0; i < 100; i++){
    as5047p.readAngleDegree(true);
    delay(1);
  }
  float sum = 0;
  int num_samples = 100;
   for (int i = 0; i < num_samples; i++){
    sum += 360.0f - as5047p.readAngleDegree(true);
    delay(1);
  }
  // for (int i = 0; i< 300; i++){
  //   Serial.printf("%3d , %f \n", i, 360.0f - as5047p.readAngleDegree(true));
  // }
  Serial.print("encoder zero callibration value = ");  Serial.println(sum / num_samples);
  delay(5000);
}

static inline uint16_t clamp_0_1999(float x)
{
  if (x < 0.0f) x = 0.0f;
  if (x > 1999.0f) x = 1999.0f;
  return (uint16_t)(x + 0.5f);
}

static inline float wrap_pi(float x)
{
  x = fmodf(x + PI, TWO_PI);
  if (x < 0.0f) x += TWO_PI;
  return x - PI;
}

static inline float wrap_2pi(float x)
{
  // returns angle in [0, 2*pi)
  x = fmodf(x, TWO_PI);
  if (x < 0.0f) x += TWO_PI;
  return x;
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
}

void getDesState() {
  thro_des  = constrain((channel_1_pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
  roll_des  = constrain((channel_2_pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
  pitch_des = constrain((channel_3_pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
  yaw_des   = constrain((channel_4_pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
}

void loopRate(int freq) {
  const float invFreqUs = (1.0f / freq) * 1000000.0f;
  unsigned long checker = micros();

  // Keep sending DShot frames while we wait
  while (invFreqUs > (checker - current_time)) {
    ESCCMD_tic();
    checker = micros();
  }
}

