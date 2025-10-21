/*
 * ESP32-S3 闭环PID & 多圈累计 (V6)
 * * 适用于: 连续旋转舵机 + 单圈不限位传感器
 * * 功能:
 * 1. IO1/IO42 为反馈电位器供电.
 * 2. 传感器 (GPIO 2) 角度 "解算" (Unwrapping):
 * - 检测 ADC 从 4095->0 (或 0->4095) 的突变.
 * - 累计 "圈数" (totalLaps).
 * - PID Input = 累计的总角度 (e.g., 720.5 度).
 * 3. 按键 (GPIO 0):
 * - 短按: 目标角度 (targetAngle) 增加 45 度.
 * - 长按: 扫描模式 (targetAngle 持续增加/减少).
 * 4. PID:
 * - Setpoint = targetAngle (累计的目标角度).
 * - Output = 速度指令 (-500 到 +500).
 * 5. SBUS 命令 = 1500 (停止) + PID 输出 (速度).
 */

#include <HardwareSerial.h>      // 用于 SBUS
#include <Adafruit_NeoPixel.h>   // 用于 LED
#include <QuickPID.h>            // 使用 QuickPID 库
#include <math.h>                // 【新】用于 fmod (LED 显示)

// --- 引脚定义 ---
#define SERVO_FB_PIN 2           // (ADC1_CH1) 舵机反馈(实际)
#define BUTTON_PIN 0             // GPIO 0 (BOOT 键)
#define LED_PIN 48               // 板载 LED
#define SBUS_TX_PIN 14           // SBUS 输出
#define SERVO_FB_VCC_PIN 1       // (IO1) 舵机反馈电位器的 "VCC"
#define SERVO_FB_GND_PIN 42      // (IO42) 舵机反馈电位器的 "GND"

// --- 板载 LED (NeoPixel) 配置 ---
#define LED_COUNT 1
#define MAX_BRIGHTNESS 64
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS 配置 ---
#define SBUS_BAUD 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_INVERTED true
HardwareSerial& sbusSerial = Serial1;
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
byte sbusFrame[SBUS_FRAME_LEN];
uint16_t sbusChannels[16];

// --- 【V6】角度解算 (Unwrapping) 配置 ---
#define ADC_RANGE_MIN 0
#define ADC_RANGE_MAX 4095
#define ADC_RANGE_HALF (ADC_RANGE_MAX - ADC_RANGE_MIN) / 2
#define ANGLE_PER_LAP 360.0 // 传感器转一圈 = 360 度

// --- 【V6】全局角度变量 ---
int lastFeedbackRaw = 0;       // 上一帧的 ADC 读数
int totalLaps = 0;             // 累计的圈数
float totalAngleAccumulated = 0.0; // 【PID Input】累计的实际总角度
float targetAngle = 0.0;       // 【PID Setpoint】累计的目标总角度

// --- QuickPID 控制器变量 ---
float Setpoint = 0; // (将被 targetAngle 填充)
float Input = 0;    // (将被 totalAngleAccumulated 填充)
float Output = 0;   // PID 输出 (速度: -500 到 +500)

// 【!! V6 警告: 必须重新调参 !!】
float Kp = 1.0, Ki = 0.0, Kd = 0.0; 

QuickPID myPID(&Input, &Output, &Setpoint);

// --- 【V6】按键配置 ---
#define TARGET_STEP_INCREMENT 45.0 // 短按一次 = 增加 45 度

// --- 按键状态机 ---
bool scanMode = false;
uint32_t buttonTimer = 0;
bool buttonActive = false;
bool longPressActive = false;

// --- 扫描模式 ---
float scanDirection = 1.0;
unsigned long lastScanTime = 0;

// --- 时间控制 ---
unsigned long lastFrameTime = 0;
const long frameInterval = 20; // 50Hz

//================================================================
//   主程序
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n==========================================");
  Serial.println("ESP32-S3 闭环PID & 多圈累计 (V6)");
  Serial.println("==========================================");

  // --- 为传感器供电 (IO1/42) ---
  pinMode(SERVO_FB_VCC_PIN, OUTPUT);
  digitalWrite(SERVO_FB_VCC_PIN, HIGH); // IO1 输出 3.3V
  pinMode(SERVO_FB_GND_PIN, OUTPUT);
  digitalWrite(SERVO_FB_GND_PIN, LOW);  // IO42 输出 0V (GND)
  Serial.println("已设置 IO1(HIGH) 和 IO42(LOW) 为反馈传感器供电。");

  // 初始化按键 (GPIO 0)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // --- 初始化 LED ---
  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.setPixelColor(0, 50, 0, 50); // 紫色启动
  strip.show();
  delay(500);

  // --- 初始化 SBUS 串口 ---
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, SBUS_INVERTED);
  Serial.println("SBUS 串口已在 IO14 上初始化。");

  // --- 初始化所有通道到中间值 ---
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
  
  // --- 【V6】初始化角度解算 ---
  // 必须在 PID 启动前读取一次
  delay(100); // 等待 ADC 稳定
  lastFeedbackRaw = analogRead(SERVO_FB_PIN);
  // 假设启动时在 0 圈
  totalAngleAccumulated = map(lastFeedbackRaw, ADC_RANGE_MIN, ADC_RANGE_MAX, -ANGLE_PER_LAP/2.0, ANGLE_PER_LAP/2.0);
  targetAngle = 0; // 启动时目标为 0 度
  Serial.printf("传感器初始化: ADC=%d, 启动角度=%.1f\n", lastFeedbackRaw, totalAngleAccumulated);

  // --- 初始化 QuickPID ---
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-500, 500);     // 速度输出
  myPID.SetSampleTimeUs(20000);         // 20ms 采样
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.Initialize();

  Serial.println("QuickPID 控制器已启动 (多圈累计模式)。");
  Serial.println("短按BOOT: 目标增加 45 度 | 长按(1s): 扫描");
}

void loop() {
  // 1. 检查按键输入 (更新 targetAngle)
  handleButton();

  // 2. 如果在扫描模式, 更新 (更新 targetAngle)
  if (scanMode) {
    updateAutoScan();
  }

  // 3. 保持 50Hz (20ms) 的更新频率
  unsigned long currentTime = millis();
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // --- 【V6】PID 闭环核心 ---

    // 4. 读取 "实际位置" (Input) 并执行 "角度解算"
    int feedbackRaw = analogRead(SERVO_FB_PIN);
    
    // 计算 ADC 变化量
    int change = feedbackRaw - lastFeedbackRaw;

    if (change > ADC_RANGE_HALF) {
      // 突变: High -> Low (例如 4000 -> 100, change ≈ -3900)
      // 这是 V5 的错误, 应该是 change < -ADC_RANGE_HALF
      // 让我们重新思考:
      // A) 4000 -> 100. change = 100 - 4000 = -3900.   ( < -2048 ) -> 正转
      // B) 100 -> 4000. change = 4000 - 100 = 3900.    ( > 2048 ) -> 反转
      
      // (A) 正转 (Low -> High)
      totalLaps++; 
    } else if (change > ADC_RANGE_HALF) { 
      // (B) 反转 (High -> Low)
      totalLaps--;
    }
    
    // 重新计算 V6 逻辑:
    // A) 4000 -> 100 (正转). change = 100 - 4000 = -3900.  ( < -2048 )
    // B) 100 -> 4000 (反转). change = 4000 - 100 = 3900. ( > 2048 )
    
    if (change > ADC_RANGE_HALF) { // (B) 反转
        totalLaps--;
    } else if (change < -ADC_RANGE_HALF) { // (A) 正转
        totalLaps++;
    }
    
    // (无论是否跳变, 都更新)
    float singleLapAngle = map(feedbackRaw, ADC_RANGE_MIN, ADC_RANGE_MAX, -ANGLE_PER_LAP/2.0, ANGLE_PER_LAP/2.0); // -180 to 180
    totalAngleAccumulated = singleLapAngle + (totalLaps * ANGLE_PER_LAP);
    
    Input = totalAngleAccumulated; // 更新 PID 输入
    Setpoint = targetAngle;      // 更新 PID 目标
    
    lastFeedbackRaw = feedbackRaw; // !! 必须在最后更新 !!

    // 5. 运行 PID 控制器
    myPID.Compute(); 
    // "Output" 变量是 -500 到 +500 之间的 "速度"

    // 6. 将 PID 速度输出转换为 SBUS 命令
    sbusChannels[0] = 1500 + (int)Output;
    sbusChannels[0] = constrain(sbusChannels[0], 1000, 2000);

    // 7. 更新 LED 状态
    updateLedStatus(targetAngle, totalAngleAccumulated);
    
    // 8. 串口调试输出
    if (!scanMode) { 
      Serial.printf("目标: %.1f deg, 实际: %.1f deg (Laps: %d, ADC: %d), PID速度: %.0f, SBUS: %d us\n",
                    targetAngle, totalAngleAccumulated, totalLaps, feedbackRaw, Output, sbusChannels[0]);
    }

    // 9. 打包并发送 SBUS
    buildSbusFrame();
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}

//================================================================
//   辅助函数
//================================================================

/**
 * @brief 【V6 变更】处理按键逻辑 (累计目标)
 */
void handleButton() {
  bool buttonState = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonState && !buttonActive) {
    // 按键刚被按下
    buttonActive = true;
    buttonTimer = millis();
    longPressActive = false;
  } 
  else if (buttonState && buttonActive) {
    // 按键被按住
    
    // 检查长按 (1秒)
    if (!longPressActive && (millis() - buttonTimer > 1000)) {
      longPressActive = true;
      
      // 长按: 切换扫描模式
      scanMode = !scanMode;
      if (scanMode) {
        Serial.println("\n*** 自动扫描模式 启动 ***");
        scanDirection = 1.0; // 默认正转
        lastScanTime = millis();
      } else {
        Serial.println("\n*** 自动扫描模式 停止 ***");
        // 停止扫描: 将目标角度设置为当前舵机的实际角度
        targetAngle = totalAngleAccumulated;
        Serial.printf("目标锁定: %.1f 度\n", targetAngle);
      }
    }
  } 
  else if (!buttonState && buttonActive) {
    // 按键被释放
    if (!longPressActive) {
      // 触发短按
      if (!scanMode) { // 扫描模式下短按无效
        // 短按: 目标角度增加
        targetAngle += TARGET_STEP_INCREMENT;
        Serial.printf("新目标: %.1f 度\n", targetAngle);
      }
    }
    // 重置所有标志
    buttonActive = false;
    longPressActive = false;
  }
}

/**
 * @brief 【V6 变更】更新自动扫描 (累计目标)
 */
void updateAutoScan() {
  if (millis() - lastScanTime > 20) { // 扫描速度
    lastScanTime = millis();
    // 持续增加/减少目标角度
    targetAngle += (scanDirection * 0.5); // 每次移动 0.5 度
    
    // (不再需要边界检查, 它可以永远转下去)
  }
}

/**
 * @brief 【V6 变更】更新 LED 状态 (多圈)
 * * 颜色 = 目标角度 (单圈)
 * * 亮度 = PID 误差
 */
void updateLedStatus(float target, float actual) {
  
  // 1. 颜色(Hue): 代表目标角度 (只看当前圈)
  // fmod = 浮点数取模. 370.5 % 360 = 10.5
  float targetSingleLap = fmod(target, 360.0);
  uint16_t hue = map(targetSingleLap, -180, 180, 0, 65535); 

  uint8_t saturation = 255;
  
  // 2. 亮度(Value): 代表误差 (0度=低亮度, 90度=高亮度)
  float error = abs(target - actual);
  uint8_t value = map(error, 0, 90, 20, 255); // 误差超过90度就最大亮度
  value = constrain(value, 20, 255);

  uint32_t color = strip.ColorHSV(hue, saturation, value);
  strip.setPixelColor(0, color);
  strip.show();
}


//================================================================
//   SBUS 帧打包函数 (无变化)
//================================================================

uint16_t mapPwmToSbus(uint16_t pwm) {
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;
  return (uint16_t)map(pwm, 1000, 2000, 192, 1792);
}
void buildSbusFrame() {
  uint16_t sbusValues[16];
  for (int i = 0; i < 16; i++) {
    sbusValues[i] = mapPwmToSbus(sbusChannels[i]);
  }
  sbusFrame[0] = SBUS_START_BYTE;
  sbusFrame[1]  = (byte)( (sbusValues[0] & 0x07FF) );
  sbusFrame[2]  = (byte)( (sbusValues[0] & 0x07FF) >> 8  | (sbusValues[1] & 0x07FF) << 3 );
  sbusFrame[3]  = (byte)( (sbusValues[1] & 0x07FF) >> 5  | (sbusValues[2] & 0x07FF) << 6 );
  sbusFrame[4]  = (byte)( (sbusValues[2] & 0x07FF) >> 2 );
  sbusFrame[5]  = (byte)( (sbusValues[2] & 0x07FF) >> 10 | (sbusValues[3] & 0x07FF) << 1 );
  sbusFrame[6]  = (byte)( (sbusValues[3] & 0x07FF) >> 7  | (sbusValues[4] & 0x07FF) << 4 );
  sbusFrame[7]  = (byte)( (sbusValues[4] & 0x07FF) >> 4  | (sbusValues[5] & 0x07FF) << 7 );
  sbusFrame[8]  = (byte)( (sbusValues[5] & 0x07FF) >> 1 );
  sbusFrame[9]  = (byte)( (sbusValues[5] & 0x07FF) >> 9  | (sbusValues[6] & 0x07FF) << 2 );
  sbusFrame[10] = (byte)( (sbusValues[6] & 0x07FF) >> 6  | (sbusValues[7] & 0x07FF) << 5 );
  sbusFrame[11] = (byte)( (sbusValues[7] & 0x07FF) >> 3 );
  sbusFrame[12] = (byte)( (sbusValues[8] & 0x07FF) );
  sbusFrame[13] = (byte)( (sbusValues[8] & 0x07FF) >> 8  | (sbusValues[9] & 0x07FF) << 3 );
  sbusFrame[14] = (byte)( (sbusValues[9] & 0x07FF) >> 5  | (sbusValues[10] & 0x07FF) << 6 );
  sbusFrame[15] = (byte)( (sbusValues[10] & 0x07FF) >> 2 );
  sbusFrame[16] = (byte)( (sbusValues[10] & 0x07FF) >> 10 | (sbusValues[11] & 0x07FF) << 1 );
  sbusFrame[17] = (byte)( (sbusValues[11] & 0x07FF) >> 7  | (sbusValues[12] & 0x07FF) << 4 );
  sbusFrame[18] = (byte)( (sbusValues[12] & 0x07FF) >> 4  | (sbusValues[13] & 0x07FF) << 7 );
  sbusFrame[19] = (byte)( (sbusValues[13] & 0x07FF) >> 1 );
  sbusFrame[20] = (byte)( (sbusValues[13] & 0x07FF) >> 9  | (sbusValues[14] & 0x07FF) << 2 );
  sbusFrame[21] = (byte)( (sbusValues[14] & 0x07FF) >> 6  | (sbusValues[15] & 0x07FF) << 5 );
  sbusFrame[22] = (byte)( (sbusValues[15] & 0x07FF) >> 3 );
  sbusFrame[23] = 0x00;
  sbusFrame[24] = SBUS_END_BYTE;
}