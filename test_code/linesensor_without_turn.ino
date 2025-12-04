/*
 * Robot 2: Follower (line sensors as IR receiver)
 * - Use 5 line sensors (passive mode) to read IR from Leader.
 * - RC timeout + ADC combined to estimate IR strength vs distance.
 * - PID #1: keep fused average strength (distance).
 * - (Turning PID 可以后面再打开做方向控制)
 */

#include <Arduino.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

// =========================================================================
// Simple PID class
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

    void initialise(float p, float i, float d, float min_out, float max_out)
    {
        _p_gain      = p;
        _i_gain      = i;
        _d_gain      = d;
        _min_output  = min_out;
        _max_output  = max_out;
        reset();
    }

    void reset()
    {
        _i_sum      = 0.0f;
        _last_error = 0.0f;
        _ms_last_t  = millis();
    }

    float update(float demand, float measurement)
    {
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
        if (_i_sum > _max_output) _i_sum = _max_output;
        if (_i_sum < _min_output) _i_sum = _min_output;
        float i_term = _i_gain * _i_sum;

        // D
        float diff_error = (error - _last_error) / float_dt;
        float d_term     = _d_gain * diff_error;
        _last_error      = error;

        float feedback = p_term + i_term + d_term;

        if (feedback > _max_output) feedback = _max_output;
        if (feedback < _min_output) feedback = _min_output;

        return feedback;
    }
};

// ----------------------------------------
// Line sensor data (RC timeout readings)
// ----------------------------------------
const uint8_t NUM_SENSORS = 5;
uint16_t lineSensorValues[NUM_SENSORS];

uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];

// ----------------------------------------
// ADC data (only for sensors with ADC pins)
// DN2 -> A0, DN3 -> A2, DN4 -> A3, DN5 -> A4
// DN1 在 D12 上没有 ADC，这里忽略
// ----------------------------------------
const uint8_t ADC_COUNT = 4;
const uint8_t adcPins[ADC_COUNT] = { A0, A2, A3, A4 };
uint16_t adcValues[ADC_COUNT];
uint16_t adcMin[ADC_COUNT];
uint16_t adcMax[ADC_COUNT];

// ----------------------------------------
// Motion / control constants
// ----------------------------------------

// 手动设置 RC timeout (微秒)
const uint16_t RC_TIMEOUT_US = 6000;

// Target average strength after normalization (0.0 ~ 1.0)
const float TARGET_STRENGTH = 0.50f;

// If fused signal is weaker than this, assume Leader lost
const float STOP_THRESHOLD = 0.10f;

// 权重切换阈值：RC 强/弱的判断
const float RC_STRONG = 0.20f;  // RC >= 0.20 → 近距离，信任 RC
const float RC_WEAK   = 0.05f;  // RC <= 0.05 → 远距离，信任 ADC

// Motor speed limits
const int MAX_SPEED = 50;   // limit to 50
const int MIN_SPEED = 0;

// Max turn adjustment (暂时不用)
const int MAX_TURN = 50;

// ----------------------------------------
// PID parameters
// ----------------------------------------
const float DISTANCE_KP = 800.0f;
const float DISTANCE_KI = 0.0f;
const float DISTANCE_KD = 10.0f;

const float TURNING_KP = 100.0f;
const float TURNING_KI = 0.0f;
const float TURNING_KD = 5.0f;

// ----------------------------------------
// Pololu objects
// ----------------------------------------
Buzzer      buzzer;
ButtonA     buttonA;
Motors      motors;
LineSensors lineSensors;

PID_c distance_pid;
PID_c turning_pid;

// Forward declaration
void runCalibration();

// =========================================================================
// Setup
// =========================================================================
void setup()
{
    Serial.begin(115200);

    // ★ 在这里统一设置 timeout，之后所有 read() 都用 6000us
    runCalibration();

    distance_pid.initialise(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD,
                            (float)MIN_SPEED, (float)MAX_SPEED);

    turning_pid.initialise(TURNING_KP, TURNING_KI, TURNING_KD,
                           (float)(-MAX_TURN), (float)MAX_TURN);
}

// =========================================================================
// Loop
// =========================================================================
void loop()
{
    // 1. RC timeout 读法（被动模式，不点亮自己 IR 灯）
    // ★ 这里改成 2 参数版本：timeout 已经由 setTimeout() 设置好了
    lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);

    // 2. RC 归一化
    float total_strength_rc = 0.0f;
    float left_side_sum_rc  = 0.0f;
    float right_side_sum_rc = 0.0f;

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        float range = (float)(sensorMax[i] - sensorMin[i]);
        if (range <= 0.0f) range = 1.0f;

        // (Max - raw) / range:
        // raw 小 = 更亮 (IR 强) → norm 接近 1
        float norm = (float)(sensorMax[i] - lineSensorValues[i]) / range;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        total_strength_rc += norm;

        // sensor 0,1 -> left
        if (i <= 1) left_side_sum_rc += norm;
        // sensor 3,4 -> right
        if (i >= 3) right_side_sum_rc += norm;
        // sensor 2 -> center, 不参与方向差
    }

    float avg_strength_rc = total_strength_rc / (float)NUM_SENSORS;

    // 左减右：>0 表示 Leader 在左边一点，<0 表示偏右
    float direction_rc = (left_side_sum_rc / 2.0f) - (right_side_sum_rc / 2.0f);

    // 3. ADC 读法 & 归一化（只做整体强度，用于远距离）
    float total_strength_adc = 0.0f;

    for (uint8_t j = 0; j < ADC_COUNT; j++)
    {
        uint16_t raw = analogRead(adcPins[j]);
        adcValues[j] = raw;

        float range_adc = (float)(adcMax[j] - adcMin[j]);
        if (range_adc <= 0.0f) range_adc = 1.0f;

        // 这里假设：Leader 近时 raw 变小 → 跟 RC 一样用 (max - raw)/range
        float norm_adc = (float)(adcMax[j] - raw) / range_adc;
        if (norm_adc < 0.0f) norm_adc = 0.0f;
        if (norm_adc > 1.0f) norm_adc = 1.0f;

        total_strength_adc += norm_adc;
    }

    float avg_strength_adc = total_strength_adc / (float)ADC_COUNT;

    // 4. 结合 RC + ADC：近距离信任 RC，远距离信任 ADC
    float w = 0; // 权重：0~1，越大越信 RC

    //if (avg_strength_rc >= RC_STRONG) {
    //    w = 1.0f;   // RC 很强：用 RC 为主
    //} else if (avg_strength_rc <= RC_WEAK) {
    //    w = 0.0f;   // RC 太弱：用 ADC 为主
    //} else {
    //    // 中间区域线性过渡
    //    w = (avg_strength_rc - RC_WEAK) / (RC_STRONG - RC_WEAK);
    //}

    float fused_strength = w * avg_strength_rc + (1.0f - w) * avg_strength_adc;

    // 5. 控制逻辑
    if (fused_strength < STOP_THRESHOLD)
    {
        // 信号太弱：认定 Leader 丢失
        motors.setSpeeds(0, 0);
        distance_pid.reset();
        turning_pid.reset();

        Serial.println("Signal lost");
    }
    else
    {
        // 距离 PID：用融合后的强度
        float base_speed = distance_pid.update(TARGET_STRENGTH, fused_strength);

        // 将来要做方向 PID 的时候，用 direction_rc：
        // float turn_adjustment = turning_pid.update(0.0f, direction_rc);
        //
        // int leftSpeed  = (int)(base_speed - turn_adjustment);
        // int rightSpeed = (int)(base_speed + turn_adjustment);

        // 目前只看距离效果：左右速度相同
        int leftSpeed  = (int)(base_speed);
        int rightSpeed = (int)(base_speed);

        if (leftSpeed > MAX_SPEED)  leftSpeed  = MAX_SPEED;
        if (leftSpeed < -MAX_SPEED) leftSpeed  = -MAX_SPEED;
        if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
        if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;

        motors.setSpeeds(leftSpeed, rightSpeed);

        // Debug 输出：方便 Serial Plotter 看三条曲线
        Serial.print("RC:");
        Serial.print(avg_strength_rc);
        Serial.print("\tADC:");
        Serial.print(avg_strength_adc);
        Serial.print("\tW:");
        Serial.print(w);
        Serial.print("\tFused:");
        Serial.print(fused_strength);
        Serial.print("\tDirRC:");
        Serial.print(direction_rc);
        Serial.print("\tL:");
        Serial.print(leftSpeed);
        Serial.print("\tR:");
        Serial.println(rightSpeed);
    }

    delay(10);
}

// =========================================================================
// Calibration for 5 sensors (RC) + ADC min/max
// =========================================================================
void runCalibration()
{
    
    Serial.println("Press Button A to start calibration.");
    Serial.println("Step 1: MAX (ambient / dark), Step 2: MIN (Leader close).");
    buzzer.play("L16 c");
    buttonA.waitForPress();
    delay(500);

    // Init RC min/max
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensorMin[i] = 2500;  // bigger than any expected reading
        sensorMax[i] = 0;
    }

    // Init ADC min/max
    for (uint8_t j = 0; j < ADC_COUNT; j++)
    {
        adcMin[j] = 1023;
        adcMax[j] = 0;
    }

    // --- Step 1: MAX (no Leader signal / ambient light) ---
    Serial.println("Calibrating RC MAX & ADC MAX (no Leader)...");
    buzzer.play("L16 c");
    unsigned long startTime = millis();

    while (millis() - startTime < 2000)
    {
        // RC 读数（★ 已经统一 timeout，无需再传 RC_TIMEOUT_US）
        lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (lineSensorValues[i] > sensorMax[i])
            {
                sensorMax[i] = lineSensorValues[i];
            }
        }

        // ADC 读数
        for (uint8_t j = 0; j < ADC_COUNT; j++)
        {
            uint16_t raw = analogRead(adcPins[j]);
            if (raw > adcMax[j]) adcMax[j] = raw;
        }
    }

    // --- Step 2: MIN (Leader close, strong IR) ---
    Serial.println("Calibrating RC MIN & ADC MIN (Leader close)...");
    Serial.println("Place Leader close and move across sensors.");
    buzzer.play("L16 g");

    startTime = millis();
    while (millis() - startTime < 5000) // 5 seconds to move Leader around
    {
        // RC 读数
        lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (lineSensorValues[i] < sensorMin[i])
            {
                sensorMin[i] = lineSensorValues[i];
            }
        }

        // ADC 读数
        for (uint8_t j = 0; j < ADC_COUNT; j++)
        {
            uint16_t raw = analogRead(adcPins[j]);
            if (raw < adcMin[j]) adcMin[j] = raw;
        }
    }

    Serial.println("Calibration done.");
    buzzer.play("L16 c g c");

    // 打印 RC 校准结果
    Serial.print("RC sensorMin/sensorMax: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorMin[i]);
        Serial.print("/");
        Serial.print(sensorMax[i]);
        Serial.print("  ");
    }
    Serial.println();

    // 打印 ADC 校准结果
    Serial.print("ADC adcMin/adcMax: ");
    for (uint8_t j = 0; j < ADC_COUNT; j++)
    {
        Serial.print(adcMin[j]);
        Serial.print("/");
        Serial.print(adcMax[j]);
        Serial.print("  ");
    }
    Serial.println();

    delay(1000);
}
