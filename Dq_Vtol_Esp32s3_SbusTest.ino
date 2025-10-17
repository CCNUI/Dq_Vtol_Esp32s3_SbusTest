#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// --- 板载 LED (NeoPixel) 配置 ---
#define LED_PIN 48       // ESP32-S3-DevKitC 上的板载 LED 引脚
#define LED_COUNT 1
#define MAX_BRIGHTNESS 64 // 1/4 亮度 (64/255)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS 配置 ---
#define SBUS_TX_PIN 14
#define SBUS_BAUD 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_INVERTED true

HardwareSerial& sbusSerial = Serial1;

// --- SBUS 帧结构 ---
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
byte sbusFrame[SBUS_FRAME_LEN];

uint16_t sbusChannels[16];

// --- 时间控制 ---
unsigned long lastFrameTime = 0;
const long frameInterval = 20; // 50Hz

// --- 新：测试模式逻辑 ---
enum TestMode {
  MODE_AUTOSCAN, // 自动逐通道扫描
  MODE_MANUAL    // 手动选择一个通道
};
TestMode currentMode = MODE_AUTOSCAN; // 默认启动自动扫描
unsigned long lastAutoScanSwitchTime = 0;
const long autoScanInterval = 5000; // 每 5 秒切换一个通道
int autoScanChannelIndex = 0; // 0-15
// --- 结束新增 ---

// 核心变量，保存当前正在测试的通道 (0-15)
int currentTestChannel = 0;


//================================================================
//   API 和 核心功能函数
//================================================================

/**
 * @brief 【API】更新 SBUS 通道值
 * * 此函数逻辑不变：它只负责扫描 `currentTestChannel`
 * * 其他所有通道保持 1500us
 */
void update_sbus_channels() {
  
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

  // 1. 将所有通道重置为默认值 1500us
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }

  // 2. 仅设置当前测试通道的值
  sbusChannels[currentTestChannel] = sweepPwm;
}

/**
 * @brief 【LED 逻辑】更新 LED
 * * 此函数逻辑不变：
 * * 颜色 = `currentTestChannel`
 * * 亮度 = `currentTestChannel` 的 PWM 值
 */
void updateLedColor(int channel, int pwmValue) {
  uint16_t hue = map(channel, 0, 15, 0, 65535); // 颜色
  uint8_t saturation = 255;
  uint8_t value = map(pwmValue, 1000, 2000, 20, 255); // 亮度 (高低位)

  uint32_t color = strip.ColorHSV(hue, saturation, value);
  strip.setPixelColor(0, color);
  strip.show();
}

/**
 * @brief 【新】处理自动扫描的逻辑
 * (仅在 AUTOSCAN 模式下工作)
 */
void updateAutoScan() {
  if (currentMode != MODE_AUTOSCAN) {
    return; // 不在自动模式，退出
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastAutoScanSwitchTime >= autoScanInterval) {
    lastAutoScanSwitchTime = currentTime;
    
    // 切换到下一个通道
    autoScanChannelIndex++;
    if (autoScanChannelIndex > 15) {
      autoScanChannelIndex = 0; // 循环
    }
    
    currentTestChannel = autoScanChannelIndex; // 更新当前测试的通道
    
    Serial.printf("\n*** 自动扫描: 正在测试通道 %d ***\n", currentTestChannel + 1);
  }
}

/**
 * @brief 【修改】检查来自 USB 串口的命令
 */
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("c")) {
      cmd = cmd.substring(1); 
    }

    int ch = cmd.toInt();
    
    if (ch >= 1 && ch <= 16) {
      currentMode = MODE_MANUAL; // **新：切换到手动模式**
      currentTestChannel = ch - 1; // 转换为 0-15 索引
      Serial.printf("\n*** 手动模式: 正在测试通道 %d ***\n", ch);
    } else if (cmd.equalsIgnoreCase("auto") || cmd.equalsIgnoreCase("scan")) {
      currentMode = MODE_AUTOSCAN; // **新：允许用户返回自动模式**
      autoScanChannelIndex = currentTestChannel; // 从当前通道开始继续扫描
      lastAutoScanSwitchTime = millis(); // 马上开始计时
      Serial.println("\n*** 切换到自动扫描模式 ***");
    } else if (cmd.length() > 0) {
      Serial.println("输入无效。请输入 1-16 选择通道, 或输入 'auto' 返回自动扫描。");
    }
  }
}

// (buildSbusFrame 和 mapPwmToSbus 函数保持不变)
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


//================================================================
//   Arduino 主程序
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("==========================================");
  Serial.println("ESP32-S3 SBUS 舵机测试程序");
  Serial.println("==========================================");

  // --- 初始化 LED ---
  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS); // 设置 1/4 亮度
  strip.setPixelColor(0, 50, 0, 50);   // 紫色启动
  strip.show();
  delay(500);

  // --- 初始化 SBUS 串口 ---
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, SBUS_INVERTED);
  Serial.println("SBUS 串口已在 IO14 上初始化。");

  // --- 初始化所有通道到中间值 ---
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
  
  // --- 修改了启动提示 ---
  Serial.println("\n默认启动自动扫描模式 (每 5 秒切换一个通道)。");
  Serial.println("在上方输入 1-16 来切换到手动测试。");
  Serial.println("输入 'auto' 可返回自动扫描模式。");
  Serial.printf("\n*** 自动扫描: 正在测试通道 1 ***\n");
  
  lastAutoScanSwitchTime = millis(); // 初始化自动扫描计时器
}

void loop() {
  // 1. 检查来自用户的串口命令
  checkSerialCommands();
  
  // 2. 【新】检查并更新自动扫描
  updateAutoScan();
  
  unsigned long currentTime = millis();

  // 3. 检查是否到了 50Hz (20ms) 的更新时间
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // 4. 【API】调用逻辑来更新通道值
    update_sbus_channels();

    // 5. 更新 LED 状态
    updateLedColor(currentTestChannel, sbusChannels[currentTestChannel]);
    
    // 6. 打印当前测试通道的值 (只在手动模式下打印，避免刷屏)
    if (currentMode == MODE_MANUAL) {
      Serial.printf("Testing CH%02d: %d us\n", currentTestChannel + 1, sbusChannels[currentTestChannel]);
    }

    // 7. 打包数据
    buildSbusFrame();

    // 8. 发送 SBUS 帧
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}