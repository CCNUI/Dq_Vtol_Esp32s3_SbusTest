#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// --- 板载 LED (NeoPixel) 配置 ---
#define LED_PIN 48       // ESP32-S3-DevKitC 上的板载 LED 引脚
#define LED_COUNT 1
#define MAX_BRIGHTNESS 64 // 1/4 亮度 (64/255)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS 配置 ---
#define SBUS_TX_PIN 14       // SBUS 输出引脚
#define SBUS_BAUD 100000     // SBUS 波特率
#define SBUS_CONFIG SERIAL_8E2 // SBUS 协议 (8E2)
#define SBUS_INVERTED true   // SBUS 使用反向逻辑

HardwareSerial& sbusSerial = Serial1;

// --- SBUS 帧结构 ---
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
byte sbusFrame[SBUS_FRAME_LEN];

uint16_t sbusChannels[16];

// --- 时间控制 ---
unsigned long lastFrameTime = 0;
const long frameInterval = 20; // 50Hz (20ms)

// --- 新：按键和状态机 ---
#define BOOT_BUTTON_PIN 0      // ESP32-S3-DevKitC 上的 BOOT 键
const long shortPressTime = 50;  // 按键去抖时间
const long longPressTime = 1000; // 1秒长按

byte lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool longPressTriggered = false;

// 定义两种控制模式
enum ControlState {
  STATE_STEP,  // 手动步进
  STATE_SWEEP  // 自动扫描
};
ControlState currentState = STATE_STEP; // 默认
uint16_t currentPwm = 1500;           // CH1 的 PWM 值

// 定义步进值
const int pwmSteps[] = {1500, 1000, 1250, 1500, 1750, 2000};
const int numSteps = 6;
int stepIndex = 0; // 0=1500, 1=1000, 2=1250...

//================================================================
//   API 和 核心功能函数
//================================================================

/**
 * @brief 【新】处理按键逻辑 (短按/长按)
 */
void handleButton() {
  byte buttonState = digitalRead(BOOT_BUTTON_PIN);
  unsigned long currentTime = millis();

  // 1. 按键按下
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressTime = currentTime;
    longPressTriggered = false;
  }
  // 2. 按键持续按住
  else if (buttonState == LOW && lastButtonState == LOW) {
    if (!longPressTriggered && (currentTime - buttonPressTime >= longPressTime)) {
      // --- 触发长按事件 ---
      longPressTriggered = true;
      if (currentState == STATE_STEP) {
        currentState = STATE_SWEEP;
        Serial.println("\n*** 模式: 自动扫描 (1000-2000us) ***");
      } else {
        currentState = STATE_STEP;
        // 退出扫描时，重置到中位
        stepIndex = 0;
        currentPwm = pwmSteps[stepIndex];
        Serial.println("\n*** 模式: 手动步进 (已重置到 1500us) ***");
      }
    }
  }
  // 3. 按键释放
  else if (buttonState == HIGH && lastButtonState == LOW) {
    if (!longPressTriggered && (currentTime - buttonPressTime >= shortPressTime)) {
      // --- 触发短按事件 ---
      if (currentState == STATE_STEP) {
        stepIndex++;
        if (stepIndex >= numSteps) {
          stepIndex = 1; // 循环, 跳过 0 (1500us 启动值), 从 1 (1000us) 开始
        }
        currentPwm = pwmSteps[stepIndex];
        Serial.printf("步进: %d us\n", currentPwm);
      }
      // (在扫描模式下，短按无效)
    }
  }
  lastButtonState = buttonState;
}

/**
 * @brief 【新】根据当前模式更新 PWM 值
 * (仅在 SWEEP 模式下更新)
 */
void updateSbusState() {
  static int sweepDirection = 1;
  
  if (currentState == STATE_SWEEP) {
    // 自动扫描逻辑
    currentPwm += (sweepDirection * 10);
    if (currentPwm >= 2000) {
      currentPwm = 2000;
      sweepDirection = -1;
    }
    if (currentPwm <= 1000) {
      currentPwm = 1000;
      sweepDirection = 1;
    }
  }
  // 如果是 STATE_STEP, currentPwm 由 handleButton() 管理
}


/**
 * @brief 【修改】仅更新 CH1 的值
 * 其他通道保持 1500us
 */
void update_sbus_channels() {
  // 1. 将所有通道重置为默认值 1500us
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
  // 2. 仅设置 CH1 的值为当前 PWM
  sbusChannels[0] = currentPwm;
}

/**
 * @brief 【修改】更新 LED
 * * 颜色 = CH1 PWM 值 (1000=红, 2000=绿)
 * * 亮度 = 恒定
 */
void updateLedColor(int pwmValue) {
  // 1000us = 红色 (Hue: 0)
  // 1500us = 黄色 (Hue: ~10922)
  // 2000us = 绿色 (Hue: 21845)
  uint16_t hue = map(pwmValue, 1000, 2000, 0, 21845);
  uint8_t saturation = 255;
  uint8_t value = 255; // 使用全亮度 (最终由 strip.setBrightness 缩放)

  uint32_t color = strip.ColorHSV(hue, saturation, value);
  strip.setPixelColor(0, color);
  strip.show();
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
  sbusFrame[23] = 0x00; // 标志位
  sbusFrame[24] = SBUS_END_BYTE;
}


//================================================================
//   Arduino 主程序
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("==========================================");
  Serial.println("ESP32-S3 SBUS CH1 按键测试程序");
  Serial.println("==========================================");

  // --- 初始化 BOOT 按键 ---
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("使用 BOOT 键 (GPIO 0) 进行控制。");

  // --- 初始化 LED ---
  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS); // 设置 1/4 亮度
  
  // --- 初始化 SBUS 串口 ---
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, SBUS_INVERTED);
  Serial.println("SBUS 串口已在 IO14 上初始化。");

  // --- 初始化通道和状态 ---
  currentPwm = pwmSteps[stepIndex]; // 1500us
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = 1500;
  }
  
  updateLedColor(currentPwm); // 设置初始颜色 (黄色)
  
  Serial.println("\n启动模式: 手动步进 @ 1500us (CH1)");
  Serial.println("短按: 循环步进 (1000-2000us)");
  Serial.println("长按: 切换自动扫描模式");
}

void loop() {
  // 1. 检查按键输入
  handleButton();
  
  unsigned long currentTime = millis();
  
  // 2. 检查是否到了 50Hz (20ms) 的更新时间
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // 3. 【新】更新 PWM 状态 (如果是扫描模式)
    updateSbusState();

    // 4. 【修改】应用 PWM 值 (CH1 = currentPwm, CH2-16 = 1500)
    update_sbus_channels();

    // 5. 更新 LED 颜色
    updateLedColor(currentPwm);
    
    // 6. 打包数据
    buildSbusFrame();

    // 7. 发送 SBUS 帧
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}