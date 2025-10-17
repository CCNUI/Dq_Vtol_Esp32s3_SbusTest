/*
 * ESP32-S3 闭环PID转SBUS控制程序
 * * 使用 【QuickPID】 库
 * * 功能:
 * 1. 读取 外部传感器(电位器) 作为 "目标位置" (Setpoint).
 * 2. 读取 5线舵机(内部电位器) 作为 "实际位置" (Input).
 * 3. 使用 QuickPID 控制器计算 1000-2000us 的输出值.
 * 4. 将此输出值打包到 SBUS 帧的 CH1 中.
 * 5. 通过硬件串口 (GPIO 14) 以 50Hz 发送 SBUS 帧.
 * 6. LED 状态:
 * - 颜色(Hue): 代表误差 (绿色=低误差, 红色=高误差)
 * - 亮度(Value): 代表PID输出力度 (舵机出力大小)
 */

#include <HardwareSerial.h>      // 用于 SBUS
#include <Adafruit_NeoPixel.h>   // 用于 LED
#include <QuickPID.h>            // 【新】使用 QuickPID 库

// --- 传感器和舵机引脚 ---
#define SENSOR_PIN  1  // (ADC1_CH0) 外部传感器(目标)
#define SERVO_FB_PIN 2 // (ADC1_CH1) 舵机反馈(实际)

// --- 板载 LED (NeoPixel) 配置 (来自原文件) ---
#define LED_PIN 48       // ESP32-S3-DevKitC 上的板载 LED 引脚
#define LED_COUNT 1
#define MAX_BRIGHTNESS 64 // 1/4 亮度 (64/255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS 配置 (来自原文件) ---
#define SBUS_TX_PIN 14
#define SBUS_BAUD 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_INVERTED true

HardwareSerial& sbusSerial = Serial1;

// --- SBUS 帧结构 (来自原文件) ---
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
byte sbusFrame[SBUS_FRAME_LEN];
uint16_t sbusChannels[16]; // 16个通道的 PWM 值 (1000-2000)

// --- QuickPID 控制器变量 ---
// QuickPID 使用 float 类型
float Setpoint, Input, Output;

// !! 重要 !! Kp, Ki, Kd 值需要您根据舵机响应进行 "调参"
// 建议从 Kp=1.5, Ki=0, Kd=0 开始
float Kp = 2.0, Ki = 0.1, Kd = 0.05; 

// 【新】QuickPID 构造函数
// 传入变量指针
QuickPID myPID(&Input, &Output, &Setpoint);

// --- ADC 校准 (重要！) ---
// 您的电位器和舵机反馈可能不是 0-4095
int sensorMin = 0;
int sensorMax = 4095;
int servoFbMin = 0;
int servoFbMax = 4095;

// --- 时间控制 ---
unsigned long lastFrameTime = 0;
const long frameInterval = 20; // 50Hz (20000 us)

//================================================================
//   主程序
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("==========================================");
  Serial.println("ESP32-S3 闭环PID转SBUS (QuickPID)");
  Serial.println("==========================================");

  // --- 初始化 LED (来自原文件) ---
  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.setPixelColor(0, 50, 0, 50); // 紫色启动
  strip.show();
  delay(500);

  // --- 初始化 SBUS 串口 (来自原文件) ---
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, SBUS_INVERTED);
  Serial.println("SBUS 串口已在 IO14 上初始化。");

  // --- 初始化所有通道到中间值 ---
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
  
  // --- 【新】初始化 QuickPID ---
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(1000, 2000); // 设置输出范围 1000-2000 us
  myPID.SetSampleTimeUs(20000);     // 设置采样时间 20ms (20000 us)
  myPID.SetMode(QuickPID::Control::automatic); // 启动PID
  myPID.Initialize(); // 初始化，防冲击

  Serial.println("QuickPID 控制器已启动。");
  Serial.println("转动外部电位器(传感器)来控制舵机位置。");
}

void loop() {
  
  unsigned long currentTime = millis();
  
  // 保持 50Hz (20ms) 的更新频率
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // 1. 读取 "目标位置" (Setpoint)
    // 从外部传感器电位器读取 (GPIO 1)
    int sensorRaw = analogRead(SENSOR_PIN);
    // 将 ADC 读数 (0-4095) 映射到目标 PWM 范围 (1000-2000)
    Setpoint = map(sensorRaw, sensorMin, sensorMax, 1000, 2000);

    // 2. 读取 "实际位置" (Input)
    // 从舵机反馈电位器读取 (GPIO 2)
    int feedbackRaw = analogRead(SERVO_FB_PIN);
    // 将 ADC 读数 (0-4095) 映射到实际 PWM 范围 (1000-2000)
    Input = map(feedbackRaw, servoFbMin, servoFbMax, 1000, 2000);

    // 3. 【新】运行 QuickPID 控制器
    // QuickPID::Compute() 会在达到 20ms 采样时间时自动计算并返回 true
    myPID.Compute(); 
    // "Output" 变量 (float) 会被自动更新

    // 4. 将 PID 输出应用到 SBUS CH1
    sbusChannels[0] = (int)Output; // 转换为 int
    // (其他通道保持在 1500)

    // 5. 更新 LED 状态
    updateLedStatus(Setpoint, Input, Output);
    
    // 6. 串口调试输出
    Serial.printf("目标(Sensor): %d -> %.0f us,  ", sensorRaw, Setpoint);
    Serial.printf("实际(Servo): %d -> %.0f us,  ", feedbackRaw, Input);
    Serial.printf("PID->SBUS CH1: %d us\n", (int)Output);

    // 7. 打包数据 (来自原文件)
    buildSbusFrame();
    
    // 8. 发送 SBUS 帧 (来自原文件)
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}


//================================================================
//   辅助函数
//================================================================

/**
 * @brief 【新】更新 LED 状态 (逻辑不变)
 * * 颜色 = 误差 (绿=小, 红=大)
 * * 亮度 = PID输出 (离中位点1500越远, 越亮)
 */
void updateLedStatus(float target, float actual, float pwmOutput) {
  
  // 1. 计算误差 (范围: 0-1000 us)
  float error = abs(target - actual);
  
  // 2. 颜色(Hue): 映射误差
  // 0us 误差 (绿色, Hue=21845) -> 250us 以上误差 (红色, Hue=0)
  uint16_t hue = map(error, 0, 250, 21845, 0);
  if (error > 250) hue = 0; // 超过 250us 误差 = 纯红

  uint8_t saturation = 255;
  
  // 3. 亮度(Value): 映射PID输出力度 (1000-2000)
  // 1500us (0力度) -> 20 (最低亮度)
  // 1000/2000us (500力度) -> 255 (最高亮度)
  int outputEffort = abs(pwmOutput - 1500);
  uint8_t value = map(outputEffort, 0, 500, 20, 255);

  uint32_t color = strip.ColorHSV(hue, saturation, value);
  strip.setPixelColor(0, color);
  strip.show();
}

//================================================================
//   SBUS 帧打包函数 (来自 Dq_Vtol_Esp32_SbusTest.ino)
//================================================================

/**
 * @brief 将 1000-2000us 的 PWM 值映射到 11位的 SBUS 值 (192-1792)
 * (来自原文件)
 */
uint16_t mapPwmToSbus(uint16_t pwm) {
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;
  return (uint16_t)map(pwm, 1000, 2000, 192, 1792);
}

/**
 * @brief 将 sbusChannels[16] 数组打包成 25 字节的 SBUS 帧
 * (来自原文件)
 */
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
  sbusFrame[23] = 0x00; // 标志位 (Failsafe 等)
  sbusFrame[24] = SBUS_END_BYTE;
}