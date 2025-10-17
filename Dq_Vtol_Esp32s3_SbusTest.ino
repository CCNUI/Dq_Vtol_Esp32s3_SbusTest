#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// --- 板载 LED (NeoPixel) 配置 ---
#define LED_PIN 48       // ESP32-S3-DevKitC 上的板载 LED 引脚
#define LED_COUNT 1      // 只有一个 LED
#define MAX_BRIGHTNESS 85 // 最大亮度 (0-255), 85 约等于 1/3

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS 配置 ---
#define SBUS_TX_PIN 14       // S3 上的 IO14
#define SBUS_BAUD 100000     // SBUS 标准波特率
#define SBUS_CONFIG SERIAL_8E2 // 8 数据位, 偶校验, 2 停止位
#define SBUS_INVERTED true   // SBUS 使用反向逻辑

HardwareSerial& sbusSerial = Serial1;

// --- SBUS 帧结构 ---
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
byte sbusFrame[SBUS_FRAME_LEN];

/**
 * @brief 核心 API：存储 16 个通道的 PWM 值 (1000-2000µs)
 * * 在 update_sbus_channels() 函数中修改此数组，
 * 50Hz 的主循环会自动将其打包并发送。
 */
uint16_t sbusChannels[16];

// --- 时间控制 ---
unsigned long lastFrameTime = 0;
const long frameInterval = 20; // 20ms = 50Hz


//================================================================
//   未来开发接口 (API)
//================================================================

/**
 * @brief 【你的代码在这里】
 * * 这是为未来开发预留的接口。
 * 你所有的通道控制逻辑（例如，读取遥控器、执行 PID 计算、自动驾驶逻辑等）
 * 都应该在这个函数中完成。
 * * 你只需要更新全局 sbusChannels 数组即可。
 * 此函数将由 loop() 以 50Hz 的频率自动调用。
 */
void update_sbus_channels() {
  
  // --- 演示逻辑开始 ---
  // (在实际应用中，你将用你自己的逻辑替换掉这部分)

  // 我们使用 static 变量，使其值在函数调用之间保持不变
  static int sweepPwm = 1000;
  static int sweepDirection = 1;

  // 更新扫描值
  sweepPwm += (sweepDirection * 10);
  if (sweepPwm >= 2000) {
    sweepPwm = 2000;
    sweepDirection = -1;
  }
  if (sweepPwm <= 1000) {
    sweepPwm = 1000;
    sweepDirection = 1;
  }

  // 演示单独控制：
  // CH1: 1000-2000 扫描
  sbusChannels[0] = sweepPwm;

  // CH2: 2000-1000 扫描 (与 CH1 相反)
  sbusChannels[1] = 3000 - sweepPwm;

  // CH3: 保持在 1500 (中立点)
  sbusChannels[2] = 1500;

  // CH4: 保持在 1000 (最小值)
  sbusChannels[3] = 1000;

  // CH5: 保持在 2000 (最大值)
  sbusChannels[4] = 2000;

  // CH6-16: 保持中立 (在 setup() 中已初始化)
  
  // --- 演示逻辑结束 ---
}


//================================================================
//   SBUS 核心功能函数 (通常不需要修改)
//================================================================

/**
 * @brief 根据 PWM 值 (1000-2000) 更新 LED 颜色
 * 1000 -> Red, 1500 -> Green, 2000 -> Blue
 */
void updateLedColor(int pwmValue) {
  uint8_t r = 0, g = 0, b = 0;
  
  if (pwmValue <= 1500) {
    int mapValue = map(pwmValue, 1000, 1500, 0, 255);
    r = 255 - mapValue; g = mapValue; b = 0;
  } else {
    int mapValue = map(pwmValue, 1500, 2000, 0, 255);
    r = 0; g = 255 - mapValue; b = mapValue;
  }
  
  strip.setPixelColor(0, r, g, b);
  strip.show();
}

/**
 * @brief 将 1000-2000µs 的 PWM 值映射到 11 位的 SBUS 值
 */
uint16_t mapPwmToSbus(uint16_t pwm) {
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;
  return (uint16_t)map(pwm, 1000, 2000, 192, 1792);
}

/**
 * @brief 构建 SBUS 数据帧
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
  sbusFrame[23] = 0x00; // 标志位 (Flags)
  sbusFrame[24] = SBUS_END_BYTE;
}

//================================================================
//   Arduino 主程序
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 SBUS Output Interface Ready");

  // --- 初始化 LED ---
  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.setPixelColor(0, 50, 0, 50); // 紫色启动
  strip.show();
  delay(500);

  // --- 初始化 SBUS 串口 ---
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, SBUS_INVERTED);
  Serial.println("SBUS Serial initialized on IO14");

  // --- 初始化所有通道到中间值 (1500µs) ---
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
}

void loop() {
  unsigned long currentTime = millis();

  // 检查是否到了 50Hz (20ms) 的更新时间
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // 1. 【API】调用你的逻辑来更新 16 个通道的值
    update_sbus_channels();

    // 2. (可选) 更新 LED 状态，反映 CH1
    updateLedColor(sbusChannels[0]);
    
    // 3. 打印 CH1 的值到串口 (用于调试)
    Serial.printf("CH1 PWM: %d us\n", sbusChannels[0]);

    // 4. 打包数据
    buildSbusFrame();

    // 5. 发送 SBUS 帧
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}