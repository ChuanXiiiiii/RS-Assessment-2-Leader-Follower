/*
 * 机器人 2: Follower (Bump-only 双 PID + 前馈 + 滤波)
 * 目标:
 * 1. 使用转向 PID 保持与 Leader 对齐。
 * 2. 使用距离 PID + 前馈速度，实现平滑的定距跟随。
 * 3. 只前进，不后退；信号丢失时停车。
 */

#include <Pololu3piPlus32U4.h>
#include <Arduino.h>

// =========================================================================
// PID_c 类
// =========================================================================
class PID_c {
private:
    float _last_error;
    float _i_sum;
    unsigned long _ms_last_t;
    float _p_gain;
    float _i_gain;
    float _d_gain;
    float _min_output;
    float _max_output;

public:
    PID_c() {}

    void initialise(float p, float i, float d, float min_out, float max_out) {
        _p_gain     = p;
        _i_gain     = i;
        _d_gain     = d;
        _min_output = min_out;
        _max_output = max_out;
        reset();
    }

    void reset() {
        _i_sum      = 0.0f;
        _last_error = 0.0f;
        _ms_last_t  = millis();
    }

    float update(float demand, float measurement) {
        unsigned long ms_now_t = millis();
        unsigned long ms_dt    = ms_now_t - _ms_last_t;
        _ms_last_t             = ms_now_t;

        float error = demand - measurement;

        if (ms_dt == 0) {
            return _p_gain * error;
        }

        float float_dt = (float)ms_dt / 1000.0f;

        // P
        float p_term = _p_gain * error;

        // I
        _i_sum += error * float_dt;
        if (_i_sum > _max_output) {
            _i_sum = _max_output;
        } else if (_i_sum < _min_output) {
            _i_sum = _min_output;
        }
        float i_term = _i_gain * _i_sum;

        // D
        float diff_error = (error - _last_error) / float_dt;
        float d_term     = _d_gain * diff_error;
        _last_error      = error;

        float feedback = p_term + i_term + d_term;

        if (feedback > _max_output) {
            feedback = _max_output;
        } else if (feedback < _min_output) {
            feedback = _min_output;
        }
        return feedback;
    }
};
// =========================================================================
// 结束 PID_c 类
// =========================================================================


// ----------------------------------------
// 传感器引脚 (碰撞传感器)
// ----------------------------------------
const uint8_t LEFT_BUMP_PIN  = A6;
const uint8_t RIGHT_BUMP_PIN = 5;
const uint8_t EMIT_PIN       = 11;

const uint16_t TIMEOUT = 6000;
uint16_t sensorValues[2];

// ----------------------------------------
// 校准变量
// ----------------------------------------
uint16_t left_min  = TIMEOUT;
uint16_t left_max  = 0;
uint16_t right_min = TIMEOUT;
uint16_t right_max = 0;
float left_range   = 0;
float right_range  = 0;

// ----------------------------------------
// 控制参数 & 常量
// ----------------------------------------

// 目标强度（对应目标距离，可以按你实验调）
const float TARGET_STRENGTH = 0.53f;
// 信号丢失阈值
const float STOP_THRESHOLD  = 0.10f;

// 电机速度限制
const int MAX_SPEED         = 80;
const int MIN_SPEED         = 0;
// 默认前进速度（类似巡航速度，Leader 是 (40,40)，这里给 35 左右）
const int BASE_FEEDFORWARD  = 35;

// 最大转向调整量
const int MAX_TURN          = 60;

// 转向死区，避免轻微噪声引起抖动
const float DIRECTION_DEADBAND = 0.03f;

// 距离 PID：输出仅作为对前馈速度的修正（[-20, 20]）
const float DISTANCE_KP = 100.0f;
const float DISTANCE_KI = 0.0f;
const float DISTANCE_KD = 10.0f;

// 转向 PID（稍微凶一点）
const float TURNING_KP = 80.0f;
const float TURNING_KI = 0.0f;
const float TURNING_KD = 5.0f;

// 一阶滤波用的状态变量
float strength_filtered = 0.0f;

// ----------------------------------------
// Pololu 对象
// ----------------------------------------
Pololu3piPlus32U4::Buzzer  buzzer;
Pololu3piPlus32U4::ButtonA buttonA;
Pololu3piPlus32U4::Motors  motors;

PID_c distance_pid; // 前进速度修正
PID_c turning_pid;  // 左右差速控制

// 前向声明
void runCalibration();
void readBumpersDigital(uint16_t *values);

// =========================================================================
// Setup: 初始化
// =========================================================================
void setup()
{
  Serial.begin(115200);
  pinMode(EMIT_PIN, INPUT);

  runCalibration();

  // 距离 PID：输出在 [-20, 20]，只做前馈的修正
  distance_pid.initialise(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD,
                          -20.0f, 20.0f);

  // 转向 PID：输出 [-MAX_TURN, MAX_TURN]
  turning_pid.initialise(TURNING_KP, TURNING_KI, TURNING_KD,
                         (float)(-MAX_TURN), (float)MAX_TURN);

  // 滤波初值设成 0 或者也可以直接读一次当前强度
  strength_filtered = 0.0f;
}

// =========================================================================
// Loop: 运行双 PID 控制
// =========================================================================
void loop()
{
  // 1. 读取 bump 并归一化
  readBumpersDigital(sensorValues);
  uint16_t left_raw  = sensorValues[0];
  uint16_t right_raw = sensorValues[1];

  float left_norm  = (float)(left_max  - left_raw)  / left_range;
  float right_norm = (float)(right_max - right_raw) / right_range;
  left_norm  = constrain(left_norm,  0.0f, 1.0f);
  right_norm = constrain(right_norm, 0.0f, 1.0f);

  // 2. 距离强度 & 方向误差
  float avg_strength = (left_norm + right_norm) / 2.0f;
  float direction    = left_norm - right_norm; // >0 说明左边更近，Leader 偏左

  // 小方向死区
  if (fabs(direction) < DIRECTION_DEADBAND) {
    direction = 0.0f;
  }

  // 3. 信号是否丢失
  if (avg_strength < STOP_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    distance_pid.reset();
    turning_pid.reset();
    Serial.println("STOPPED (Signal Lost)");
  }
  else
  {
    // 3.1 一阶低通滤波，平滑距离曲线
    const float ALPHA = 0.3f; // 当前测量占的权重
    strength_filtered = ALPHA * avg_strength + (1.0f - ALPHA) * strength_filtered;

    // 3.2 距离 PID：输出对前馈速度的修正量
    float dist_error = TARGET_STRENGTH - strength_filtered;
    float speed_correction = distance_pid.update(TARGET_STRENGTH, strength_filtered);

    // 基础速度 = 前馈 + PID 修正
    float base_speed = (float)BASE_FEEDFORWARD + speed_correction;

    // 方向偏差较大时减速转弯
    if (fabs(direction) > 0.20f) {
      base_speed *= 0.5f; // 先慢慢转过去
    }

    // 限幅 & 只前进
    if (base_speed < 0)         base_speed = 0;
    if (base_speed > MAX_SPEED) base_speed = MAX_SPEED;

    // 3.3 转向 PID
    // demand = direction, measurement = 0，让输出朝 0 收敛
    float turn_adjustment = turning_pid.update(direction, 0.0f);

    int leftSpeed  = (int)(base_speed - turn_adjustment);
    int rightSpeed = (int)(base_speed + turn_adjustment);

    // 不允许反转
    if (leftSpeed < 0)   leftSpeed = 0;
    if (rightSpeed < 0)  rightSpeed = 0;
    if (leftSpeed > MAX_SPEED)   leftSpeed = MAX_SPEED;
    if (rightSpeed > MAX_SPEED)  rightSpeed = MAX_SPEED;

    motors.setSpeeds(leftSpeed, rightSpeed);

    // 调试输出
    Serial.print("Target: ");   Serial.print(TARGET_STRENGTH);
    Serial.print("\tAvg_Str: "); Serial.print(avg_strength);
    Serial.print("\tFilt: ");    Serial.print(strength_filtered);
    Serial.print("\tDir: ");     Serial.print(direction);
    Serial.print("\tBase: ");    Serial.print(base_speed);
    Serial.print("\tCorr: ");    Serial.print(speed_correction);
    Serial.print("\tTurn: ");    Serial.print(turn_adjustment);
    Serial.print("\tL: ");       Serial.print(leftSpeed);
    Serial.print("\tR: ");       Serial.println(rightSpeed);
  }

  delay(20);
}

// =========================================================================
// 校准
// =========================================================================
void runCalibration()
{
  Serial.println("Press Button A to start calibration.");
  Serial.println("Step 1: MAX (no signal), Step 2: MIN (strong signal).");
  buzzer.play("L16 c");
  buttonA.waitForPress();

  // 1. MAX
  Serial.println("Step 1: Calibrating MAX (No Signal)");
  Serial.println("Remove Emitter robot. Keep area clear.");
  buzzer.play("L16 c");
  delay(1000);

  unsigned long startTime = millis();
  while (millis() - startTime < 2000)
  {
    readBumpersDigital(sensorValues);
    if (sensorValues[0] > left_max)  { left_max  = sensorValues[0]; }
    if (sensorValues[1] > right_max) { right_max = sensorValues[1]; }
  }

  // 2. MIN
  Serial.println("Step 2: Calibrating MIN (Strong Signal)");
  Serial.println("Place Emitter robot VERY CLOSE and move across sensors.");
  buzzer.play("L16 g");
  delay(1000);

  startTime = millis();
  while (millis() - startTime < 4000)
  {
    readBumpersDigital(sensorValues);
    if (sensorValues[0] < left_min)  { left_min  = sensorValues[0]; }
    if (sensorValues[1] < right_min) { right_min = sensorValues[1]; }
  }

  // 3. 范围
  left_range  = (float)(left_max  - left_min);
  right_range = (float)(right_max - right_min);
  if (left_range == 0)  { left_range  = 1; }
  if (right_range == 0) { right_range = 1; }

  Serial.println("Calibration finished!");
  Serial.print("Left min/max: ");
  Serial.print(left_min);  Serial.print("/");
  Serial.println(left_max);
  Serial.print("Right min/max: ");
  Serial.print(right_min); Serial.print("/");
  Serial.println(right_max);

  buzzer.play("L16 c g c");
  delay(1000);
}

// =========================================================================
// RC 方式读取 bump 传感器
// =========================================================================
void readBumpersDigital(uint16_t *values)
{
  // 充电
  pinMode(LEFT_BUMP_PIN, OUTPUT);
  pinMode(RIGHT_BUMP_PIN, OUTPUT);
  digitalWrite(LEFT_BUMP_PIN, HIGH);
  digitalWrite(RIGHT_BUMP_PIN, HIGH);
  delayMicroseconds(10);

  // 放电计时
  pinMode(LEFT_BUMP_PIN, INPUT);
  pinMode(RIGHT_BUMP_PIN, INPUT);

  unsigned long startTime = micros();
  bool leftDone  = false;
  bool rightDone = false;

  while (micros() - startTime < TIMEOUT)
  {
    if (!leftDone && digitalRead(LEFT_BUMP_PIN) == LOW) {
      values[0] = micros() - startTime;
      leftDone  = true;
    }
    if (!rightDone && digitalRead(RIGHT_BUMP_PIN) == LOW) {
      values[1] = micros() - startTime;
      rightDone = true;
    }
    if (leftDone && rightDone) {
      break;
    }
  }

  if (!leftDone)  { values[0] = TIMEOUT; }
  if (!rightDone) { values[1] = TIMEOUT; }
}
