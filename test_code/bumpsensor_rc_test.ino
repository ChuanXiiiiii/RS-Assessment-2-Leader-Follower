// Pololu 3pi+ 32U4 Follower - Bump Sensor RC Timing Data Logger
// 目标: 测量Bump Sensor RC计时原始读数与距离的关系。
// 仅使用Bump Sensor，不控制电机。

#include <Pololu3piPlus32U4.h>
#include <Arduino.h> 

using namespace Pololu3piPlus32U4;

// ----------------------------------------
// Bump Sensor Configuration
// ----------------------------------------
const uint8_t LEFT_BUMP_PIN = A6; 
const uint8_t RIGHT_BUMP_PIN = 5;
const uint16_t TIMEOUT = 6000; 
uint16_t sensorValues[2]; // Index 0: Left, Index 1: Right

// Calibration variables
uint16_t left_min = TIMEOUT;
uint16_t left_max = 0;
uint16_t right_min = TIMEOUT;
uint16_t right_max = 0;

// ----------------------------------------
// Pololu Objects
// ----------------------------------------
Buzzer buzzer;
ButtonA buttonA;

// Forward declarations
void readBumpersDigital(uint16_t *values);
void runCalibration();

// =========================================================================
// Setup: 初始化
// =========================================================================
void setup()
{
    Serial.begin(115200);

    Serial.println("--- Bump Sensor (RC) Data Logger Ready ---");
    runCalibration();
    Serial.println("准备开始采集数据。请手动将Follower放置在所需距离。");
    // 打印表头，方便复制到表格中
    Serial.println("距离 (mm)\tRaw Left (us)\tRaw Right (us)\tMin Left\tMax Left\tMin Right\tMax Right");
}

// =========================================================================
// Loop: 读取并打印传感器数据
// =========================================================================
void loop()
{
    readBumpersDigital(sensorValues);
    
    // 打印左右传感器的原始读数和校准值
    Serial.print("0\t"); // 距离 (mm) 占位符，需手动填写
    Serial.print(sensorValues[0]);
    Serial.print("\t");
    Serial.print(sensorValues[1]);
    Serial.print("\t");
    Serial.print(left_min);
    Serial.print("\t");
    Serial.print(left_max);
    Serial.print("\t");
    Serial.print(right_min);
    Serial.print("\t");
    Serial.println(right_max);
    
    delay(50); 
}


// =========================================================================
// Calibration for Bump Sensors (RC)
// =========================================================================
void runCalibration()
{
    Serial.println("按键 A 开始 Bump Sensor (RC) 校准。");
    buzzer.play("L16 c");
    buttonA.waitForPress(); 

    // 1. 校准 MAX (无信号)
    Serial.println("Step 1: 校准 MAX (无信号)... 移开 Leader 机器人。");
    buzzer.play("L16 c");
    delay(1000); 

    unsigned long startTime = millis();
    while (millis() - startTime < 2000)
    {
        readBumpersDigital(sensorValues);
        if (sensorValues[0] > left_max) { left_max = sensorValues[0]; }
        if (sensorValues[1] > right_max) { right_max = sensorValues[1]; }
    }
    
    // 2. 校准 MIN (强信号)
    Serial.println("Step 2: 校准 MIN (强信号)... 将 Leader 机器人贴近。");
    buzzer.play("L16 g");
    delay(1000); 

    startTime = millis();
    while (millis() - startTime < 4000)
    {
        readBumpersDigital(sensorValues);
        if (sensorValues[0] < left_min) { left_min = sensorValues[0]; }
        if (sensorValues[1] < right_min) { right_min = sensorValues[1]; }
    }

    Serial.println("校准完成。");
    buzzer.play("L16 c g c"); 
    delay(1000);
}

// =========================================================================
// Bump Sensor Read Function (RC Timing)
// =========================================================================
void readBumpersDigital(uint16_t *values)
{
    // 充电 (设置为 OUTPUT HIGH)
    pinMode(LEFT_BUMP_PIN, OUTPUT);
    pinMode(RIGHT_BUMP_PIN, OUTPUT);
    digitalWrite(LEFT_BUMP_PIN, HIGH);
    digitalWrite(RIGHT_BUMP_PIN, HIGH);
    delayMicroseconds(10);
    
    // 开始计时放电 (设置为 INPUT)
    pinMode(LEFT_BUMP_PIN, INPUT);
    pinMode(RIGHT_BUMP_PIN, INPUT);
    unsigned long startTime = micros();
    bool leftDone = false;
    bool rightDone = false;
    
    // 测量放电时间直到 LOW
    while (micros() - startTime < TIMEOUT)
    {
        if (!leftDone) {
            if (digitalRead(LEFT_BUMP_PIN) == LOW) {
                values[0] = micros() - startTime; leftDone = true;
            }
        }
        if (!rightDone) {
            if (digitalRead(RIGHT_BUMP_PIN) == LOW) {
                values[1] = micros() - startTime; rightDone = true;
            }
        }
        if (leftDone && rightDone) { break; }
    }
    // 如果超时，使用 TIMEOUT 值
    if (!leftDone) { values[0] = TIMEOUT; }
    if (!rightDone) { values[1] = TIMEOUT; }
}