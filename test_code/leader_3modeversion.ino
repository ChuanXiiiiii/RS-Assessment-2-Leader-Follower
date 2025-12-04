/*
 * 机器人 1: Leader (多模式运动 - 按钮切换版)
 * * 目标: 
 * * 1. 作为 Leader, 立即发射 3.9kHz 红外信号。
 * * 2. 程序启动后，默认暂停（STOP 状态）。
 * * 3. 按 A 键: 进入 匀速前进 模式 (FORWARD_CONSTANT)。
 * * 4. 按 B 键: 进入 变速前进 模式 (FORWARD_VARIABLE)。
 * * 5. 按 C 键: 进入 转圈 模式 (TURN_CIRCLE)。
 */

// 包含我们需要的 Pololu 库
#include <Pololu3piPlus32U4.h>

// ----------------------------------------
// 发射器引脚
// ----------------------------------------
const uint8_t EMIT_PIN = 11;

// ----------------------------------------
// 运动模式枚举
// ----------------------------------------
enum MovementState {
    STOP,               // 停止状态 (默认或按键切换)
    FORWARD_CONSTANT,   // 匀速前进 (对应 A 键)
    FORWARD_VARIABLE,   // 变速前进 (对应 B 键)
    TURN_CIRCLE         // 转圈 (对应 C 键)
};

// 当前运动状态 (初始化为停止)
MovementState currentState = STOP;

// ----------------------------------------
// 运动控制常量
// ----------------------------------------
// 匀速前进 (FORWARD_CONSTANT)
const int CONST_LEFT_SPEED = -50;
const int CONST_RIGHT_SPEED = -50;

// 转圈 (TURN_CIRCLE) - 较大的速度差实现绕圈或原地转弯
const int CIRCLE_LEFT_SPEED = -60;
const int CIRCLE_RIGHT_SPEED = -30; // 较慢，实现向右转

// 变速前进 (FORWARD_VARIABLE) - 用于定义速度范围
const int VAR_MIN_SPEED = -40;
const int VAR_MAX_SPEED = -65;
const int VAR_SPEED_STEP = 1; // 每次循环改变的速度步长

// 变速前进的当前速度变量
int variableSpeed = VAR_MIN_SPEED;
// 变速前进的方向 (1 为加速, -1 为减速)
int speedDirection = 1;


// ----------------------------------------
// Pololu 库对象
// ----------------------------------------
Pololu3piPlus32U4::Motors motors;
Pololu3piPlus32U4::ButtonA buttonA;
Pololu3piPlus32U4::ButtonB buttonB; // <-- 新增
Pololu3piPlus32U4::ButtonC buttonC; // <-- 新增
Pololu3piPlus32U4::Buzzer buzzer;

// =========================================================================
// 4. 函数: 切换状态并播放提示音
// =========================================================================
void switchState(MovementState newState, const char* message, const char* tune) {
    if (currentState != newState) {
        currentState = newState;
        Serial.print("State Switched to: ");
        Serial.println(message);
        motors.setSpeeds(0, 0); // 切换状态时先停一下
        buzzer.play(tune);
    }
}

// =========================================================================
// 5. Setup: 初始化
// =========================================================================
void setup()
{
    // 波特率设置，用于串口监视器查看状态
    Serial.begin(115200);
    Serial.println("Robot Controller Initializing...");

    // --- 1. 设置 3.9kHz 红外发射器 (v3) ---
    // (这一步会立即执行)
    pinMode(EMIT_PIN, OUTPUT);
    // 启动 50% 占空比的 PWM
    analogWrite(EMIT_PIN, 128); 
    // 修改 Timer0 预分频器为 8 (3.9kHz)
    TCCR0B = (TCCR0B & 0b11111000) | 0x02;
    Serial.println("Leader: 3.9kHz IR Emitter ON.");

    // --- 2. 初始提示 ---
    Serial.println("Leader: Press A (Constant), B (Variable), or C (Circle) to start.");
    buzzer.play("L8 cdefgab>c"); // 播放一段提示音
    
    // Setup 函数结束，进入 loop()
}

// =========================================================================
// 6. Loop: 持续检查按键和执行当前状态的动作
// =========================================================================
void loop()
{
    // --- 1. 按键检测 (优先级高，应在 loop 开始时检查) ---
    // 按 A 开始匀速前进
    if (buttonA.isPressed()) {
        switchState(FORWARD_CONSTANT, "FORWARD_CONSTANT (A)", "L16 g");
    }
    // 按 B 开始变速前进
    else if (buttonB.isPressed()) {
        switchState(FORWARD_VARIABLE, "FORWARD_VARIABLE (B)", "L16 c>g");
    }
    // 按 C 开始转圈
    else if (buttonC.isPressed()) {
        switchState(TURN_CIRCLE, "TURN_CIRCLE (C)", "L16 c<g");
    }
    // 如果没有按键按下，保持当前状态的运行

    // --- 2. 状态执行 ---
    switch (currentState) {
        case STOP:
            // 停止模式: 两个电机速度都设置为 0
            motors.setSpeeds(0, 0);
            // 可以在这里加一个小的延迟，降低 CPU 负载
            // delay(10); 
            break;

        case FORWARD_CONSTANT:
            // 匀速前进模式: 设置固定的速度
            motors.setSpeeds(CONST_LEFT_SPEED, CONST_RIGHT_SPEED);
            break;

        case FORWARD_VARIABLE:
            // 变速前进模式: 持续改变速度
            motors.setSpeeds(variableSpeed, variableSpeed);
            
            // 更新速度值
            variableSpeed += speedDirection * VAR_SPEED_STEP;
            
            // 检查是否达到边界，并反转方向
            if (variableSpeed >= VAR_MAX_SPEED) {
                variableSpeed = VAR_MAX_SPEED;
                speedDirection = -1; // 达到最大值，开始减速
                // buzzer.play("L32 c"); // 可以添加小的提示音
            } else if (variableSpeed <= VAR_MIN_SPEED) {
                variableSpeed = VAR_MIN_SPEED;
                speedDirection = 1;  // 达到最小值，开始加速
                // buzzer.play("L32 g"); // 可以添加小的提示音
            }
            
            // 变速模式需要一个小的延迟来控制速度变化频率
            delay(10); 
            break;

        case TURN_CIRCLE:
            // 转圈模式: 设置较大的速度差
            motors.setSpeeds(CIRCLE_LEFT_SPEED, CIRCLE_RIGHT_SPEED);
            break;
    }
}