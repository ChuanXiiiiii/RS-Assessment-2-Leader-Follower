// Pololu 3pi+ 32U4 Follower - Line Sensor RC Timing Data Logger
// 目标: 测量Line Sensor RC计时原始读数与距离的关系。
// 仅使用Line Sensor，不使用Bump Sensor，不控制电机。

#include <Pololu3piPlus32U4.h>
#include <Arduino.h> 

using namespace Pololu3piPlus32U4;

// ----------------------------------------
// Line Sensor Configuration
// ----------------------------------------
const uint8_t NUM_SENSORS = 5;
const uint8_t CENTER_SENSOR_INDEX = 2; // S2 是中央传感器 (Index 2)
uint16_t lineSensorValues[NUM_SENSORS];
const uint16_t RC_TIMEOUT_US = 6000;

// 校准变量 (Max = 无信号, Min = 强信号)
uint16_t sensorMin[NUM_SENSORS] = {RC_TIMEOUT_US, RC_TIMEOUT_US, RC_TIMEOUT_US, RC_TIMEOUT_US, RC_TIMEOUT_US};
uint16_t sensorMax[NUM_SENSORS] = {0, 0, 0, 0, 0};

// ----------------------------------------
// Pololu Objects
// ----------------------------------------
Buzzer buzzer;
ButtonA buttonA;
LineSensors lineSensors;

// Forward declaration
void runCalibration();

// =========================================================================
// Setup: 初始化
// =========================================================================
void setup()
{
    Serial.begin(115200);
    // 为 Line Sensor 设置 RC 计时超时时间
    lineSensors.setTimeout(RC_TIMEOUT_US); 

    Serial.println("--- Line Sensor (RC) Data Logger Ready ---");
    runCalibration();
    Serial.println("准备开始采集数据。请手动将Follower放置在所需距离。");
    // 打印表头，方便复制到表格中
    Serial.println("距离 (mm)\tRaw S2 (us)\tMin S2\tMax S2");
}

// =========================================================================
// Loop: 读取并打印传感器数据
// =========================================================================
void loop()
{
    // 在被动模式下读取 Line Sensor (IR发射器关闭)
    lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);
    
    uint16_t raw_s2 = lineSensorValues[CENTER_SENSOR_INDEX];
    
    // 打印中央传感器 (S2) 的原始读数和校准值
    Serial.print("0\t"); // 距离 (mm) 占位符，需手动填写
    Serial.print(raw_s2);
    Serial.print("\t");
    Serial.print(sensorMin[CENTER_SENSOR_INDEX]);
    Serial.print("\t");
    Serial.println(sensorMax[CENTER_SENSOR_INDEX]);
    
    delay(50); 
}


// =========================================================================
// Calibration for Line Sensors (RC)
// =========================================================================
void runCalibration()
{
    Serial.println("按键 A 开始 Line Sensor (RC) 校准。");
    buzzer.play("L16 c");
    buttonA.waitForPress();
    delay(500);

    // --- Step 1: MAX (无信号 / 环境光) ---
    Serial.println("校准 MAX (无信号)... 移开 Leader 机器人。");
    buzzer.play("L16 c");
    unsigned long startTime = millis();

    while (millis() - startTime < 2000)
    {
        lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (lineSensorValues[i] > sensorMax[i])
            {
                sensorMax[i] = lineSensorValues[i];
            }
        }
    }

    // --- Step 2: MIN (Leader 极近, 强 IR) ---
    Serial.println("校准 MIN (强信号)... 将 Leader 机器人贴近。");
    buzzer.play("L16 g");
    delay(1000); 

    startTime = millis();
    while (millis() - startTime < 4000) 
    {
        lineSensors.read(lineSensorValues, LineSensorsReadMode::Off);
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (lineSensorValues[i] < sensorMin[i])
            {
                sensorMin[i] = lineSensorValues[i];
            }
        }
    }

    Serial.println("校准完成。");
    buzzer.play("L16 c g c");
    delay(1000);
}