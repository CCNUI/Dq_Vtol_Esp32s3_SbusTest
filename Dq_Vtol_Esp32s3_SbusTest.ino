/* Dq_Vtol_Esp32s3_SbusTest.ino
 * 【已修改 V4】PID (0-90, 反转) + 新按键步进 + 串口输入
 * * 适用于: 连续旋转舵机 + 0-90 度限位传感器
 * * 功能:
 * 1. IO6/IO4 为反馈电位器供电.
 * 2. 传感器 (GPIO 5) 角度 "解算":
 * - ADC 最小值 (0度): 1321
 * - ADC 最大值 (90度): 2355
 * - PID Input = 映射后的 0-90 度。
 * 3. 按键 (GPIO 0):
 * - 【新】短按: 目标角度在 (0, 30, 45, 60, 90) 之间循环。
 * - 长按: 扫描模式 (在 0-90 度之间来回扫描)。
 * 4. 【新】串口输入:
 * - 可通过串口监视器发送 "xx.x" 或 "xx" 来设置目标角度。
 * 5. PID:
 * - 【新】Kp/Ki 已调高，以解决死区问题。
 * - PID Action 已设置为 Reverse。
 * 6. SBUS 命令 = 1500 (停止) + PID 输出 (速度).
 */

#include <HardwareSerial.h>      // 用于 SBUS
#include <Adafruit_NeoPixel.h>   // 用于 LED
#include <QuickPID.h>            // 使用 QuickPID 库
#include <math.h>                // 用于 fmod (LED 显示)

// --- 引脚定义 ---
#define SERVO_FB_PIN 5           // (ADC1_CH4 / Touch5) 舵机反馈(实际)
#define BUTTON_PIN 0             // GPIO 0 (BOOT 键)
#define LED_PIN 48               // 板载 LED
#define SBUS_TX_PIN 14           // SBUS 输出
#define SERVO_FB_VCC_PIN 6       // (IO6 / Touch6) 舵机反馈电位器的 "VCC"
#define SERVO_FB_GND_PIN 4       // (IO4 / Touch4) 舵机反馈电位器的 "GND"

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

// --- 【V6 单圈修改】角度解算 (Unwrapping) 配置 ---
#define ADC_RANGE_MIN 1400  // 0 度对应的 ADC 读数
#define ADC_RANGE_MAX 2300  // 90 度对应的 ADC 读数
#define ANGLE_PER_LAP 90.0  // 传感器总行程 = 90 度

// --- 【V6 单圈修改】全局角度变量 ---
float currentAngle = 0.0;        // 【PID Input】当前的实际角度 (0-90)
float targetAngle = 0.0;         // 【PID Setpoint】目标角度 (0-90)

// --- QuickPID 控制器变量 ---
float Setpoint = 0; // (将被 targetAngle 填充)
float Input = 0;    // (将被 currentAngle 填充)
float Output = 0;   // PID 输出 (速度: -500 到 +500)

// 【V4 调参建议: 解决死区问题】
float Kp = 7.0, Ki = 1.0, Kd = 1.0; 

QuickPID myPID(&Input, &Output, &Setpoint);

// --- 【V4 新】按键配置 ---
const int numTestAngles = 5;
float testAngles[numTestAngles] = {0.0, 30.0, 45.0, 60.0, 90.0};
int angleIndex = 0; // 当前在 testAngles 数组中的索引

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
  Serial.println("ESP32-S3 闭环PID V4 (0-90, 反向, Kp/Ki已调)");
  Serial.println("==========================================");
  Serial.printf("ADC 范围: %d (0°) to %d (90°)\n", ADC_RANGE_MIN, ADC_RANGE_MAX);

  // --- 为传感器供电 (IO6/IO4) ---
  pinMode(SERVO_FB_VCC_PIN, OUTPUT);
  digitalWrite(SERVO_FB_VCC_PIN, HIGH); // IO6 输出 3.3V
  pinMode(SERVO_FB_GND_PIN, OUTPUT);
  digitalWrite(SERVO_FB_GND_PIN, LOW);  // IO4 输出 0V (GND)
  Serial.println("已设置 IO6(HIGH) 和 IO4(LOW) 为反馈传感器供电。");

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
  
  // --- 【V6 单圈修改】初始化角度 ---
  delay(100); // 等待 ADC 稳定
  int initialFeedbackRaw = analogRead(SERVO_FB_PIN); // 读取 IO5
  
  currentAngle = map(initialFeedbackRaw, ADC_RANGE_MIN, ADC_RANGE_MAX, 0.0, ANGLE_PER_LAP);
  currentAngle = constrain(currentAngle, 0.0, ANGLE_PER_LAP); 
  targetAngle = currentAngle; // 启动时目标 = 当前位置
  
  // 【新】找到最接近的测试角度索引
  float minDiff = 360.0;
  for (int i = 0; i < numTestAngles; i++) {
    float diff = abs(testAngles[i] - targetAngle);
    if (diff < minDiff) {
      minDiff = diff;
      angleIndex = i;
    }
  }
  Serial.printf("传感器初始化: ADC=%d, 启动角度=%.1f\n", initialFeedbackRaw, currentAngle);
  Serial.printf("启动时最接近的测试角度: %.1f 度 (索引 %d)\n", testAngles[angleIndex], angleIndex);


  // --- 初始化 QuickPID ---
  myPID.SetTunings(Kp, Ki, Kd); // 【已修改】使用新的 Kp/Ki 值
  myPID.SetOutputLimits(-500, 500);     
  myPID.SetSampleTimeUs(20000);         
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.Initialize();
  
  // 【请求 1: 反转舵机方向】
  myPID.SetControllerDirection(QuickPID::Action::reverse); 
  Serial.println("QuickPID 控制器已启动 (Kp=5.0, Ki=0.5, 方向已反转)。");
  Serial.println("短按: (0, 30, 45, 60, 90) 循环 | 长按: 扫描 | 串口输入 'xx.x' 可设置任意角度");
}

void loop() {
  // 1. 【新】检查串口输入 (允许浮点数)
  handleSerialInput();
  
  // 2. 检查按键输入 (更新 targetAngle)
  handleButton();

  // 3. 如果在扫描模式, 更新 (更新 targetAngle)
  if (scanMode) {
    updateAutoScan();
  }

  // 4. 保持 50Hz (20ms) 的更新频率
  unsigned long currentTime = millis();
  if (currentTime - lastFrameTime >= frameInterval) {
    lastFrameTime = currentTime;

    // --- 【V6 单圈修改】PID 闭环核心 ---

    // 5. 读取 "实际位置" (Input)
    int feedbackRaw = analogRead(SERVO_FB_PIN); // 读取 IO5
    float singleLapAngle = map(feedbackRaw, ADC_RANGE_MIN, ADC_RANGE_MAX, 0.0, ANGLE_PER_LAP); // 映射到 0-90
    currentAngle = constrain(singleLapAngle, 0.0, ANGLE_PER_LAP); // 限制在 0-90
    
    Input = currentAngle;     // 更新 PID 输入
    Setpoint = targetAngle;   // 更新 PID 目标

    // 6. 运行 PID 控制器
    myPID.Compute(); 

    // 7. 将 PID 速度输出转换为 SBUS 命令
    sbusChannels[0] = 1500 + (int)Output;
    sbusChannels[0] = constrain(sbusChannels[0], 1000, 2000);

    // 8. 更新 LED 状态
    updateLedStatus(targetAngle, currentAngle);

    // 9. 串口调试输出
    if (!scanMode) { 
      // 【V3 修改】反向计算目标角度对应的 ADC 值
      long targetADC = map(targetAngle, 0.0, ANGLE_PER_LAP, ADC_RANGE_MIN, ADC_RANGE_MAX);
      
      Serial.printf("目标: %.1f deg (ADC: %ld), 实际: %.1f deg (ADC: %d), PID速度: %.0f, SBUS: %d us\n",
                    targetAngle, targetADC, currentAngle, feedbackRaw, Output, sbusChannels[0]);
    }

    // 10. 打包并发送 SBUS
    buildSbusFrame();
    sbusSerial.write(sbusFrame, SBUS_FRAME_LEN);
  }
}

//================================================================
//   辅助函数
//================================================================

/**
 * @brief 【新 V4】处理串口输入 (接受 xx.x 或 xx)
 */
void handleSerialInput() {
  if (Serial.available() > 0) {
    // 读取一个浮点数 (它会处理整数 "xx" 和浮点数 "xx.x")
    // 它会一直读，直到遇到第一个非数字/非'.'/'非'-'的字符 (比如回车) 或超时
    float newAngle = Serial.parseFloat(); 
    
    // 清空缓冲区 (读取所有剩余字符，包括回车符)
    while(Serial.available() > 0) {
      Serial.read();
    }

    // 检查是否在有效范围内 (0.0 到 90.0)
    // (parseFloat 在失败时会返回 0, 但 0.0 也是有效输入, 
    //  所以我们主要依赖范围检查, 假设用户不会输入负数)
    if (newAngle >= 0.0 && newAngle <= ANGLE_PER_LAP) {
      targetAngle = newAngle;
      scanMode = false; // 退出扫描模式
      
      // 【新】同步 angleIndex 到最接近的测试角度
      float minDiff = 360.0;
      int closestIndex = 0;
      for (int i = 0; i < numTestAngles; i++) {
        float diff = abs(testAngles[i] - targetAngle);
        if (diff < minDiff) {
          minDiff = diff;
          closestIndex = i;
        }
      }
      angleIndex = closestIndex;

      Serial.printf("\n*** 新目标 (来自串口): %.1f 度 (下次按键将从 %.1f 度开始) ***\n", targetAngle, testAngles[angleIndex]);
      
    } else {
      Serial.printf("\n*** 输入无效: '%.1f' (必须在 0.0 到 90.0 之间) ***\n", newAngle);
    }
  }
}


/**
 * @brief 【V4 修改】处理按键逻辑
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
        Serial.println("\n*** 自动扫描模式 启动 (0-90 度) ***");
        scanDirection = 1.0; // 默认正转
        lastScanTime = millis();
      } else {
        Serial.println("\n*** 自动扫描模式 停止 ***");
        // 停止扫描: 将目标角度设置为当前舵机的实际角度
        targetAngle = currentAngle;
        
        // 【新】停止扫描时也同步 angleIndex
        float minDiff = 360.0;
        for (int i = 0; i < numTestAngles; i++) {
          float diff = abs(testAngles[i] - targetAngle);
          if (diff < minDiff) {
            minDiff = diff;
            angleIndex = i;
          }
        }
        Serial.printf("目标锁定: %.1f 度 (最接近索引 %d)\n", targetAngle, angleIndex);
      }
    }
  } 
  else if (!buttonState && buttonActive) {
    // 按键被释放
    if (!longPressActive) {
      // 触发短按
      if (scanMode) {
         scanMode = false; // 如果在扫描，短按也退出扫描
         // 目标保持在扫描停止时的位置
         // (并找到最近的索引)
         float minDiff = 360.0;
         for (int i = 0; i < numTestAngles; i++) {
           float diff = abs(testAngles[i] - targetAngle);
           if (diff < minDiff) {
             minDiff = diff;
             angleIndex = i;
           }
         }
         Serial.printf("扫描停止，目标: %.1f 度 (最接近索引 %d)\n", targetAngle, angleIndex);

      } else {
        // 【已修改】短按: 目标角度在 (0, 30, 45, 60, 90) 之间循环
        angleIndex = (angleIndex + 1) % numTestAngles; // 0->1->2->3->4->0
        targetAngle = testAngles[angleIndex];
        Serial.printf("新目标 (按键): %.1f 度\n", targetAngle);
      }
    }
    // 重置所有标志
    buttonActive = false;
    longPressActive = false;
  }
}

/**
 * @brief 【V6 单圈修改】更新自动扫描 (0-90 度)
 */
void updateAutoScan() {
  if (millis() - lastScanTime > 20) { // 扫描速度
    lastScanTime = millis();
    // 持续增加/减少目标角度
    targetAngle += (scanDirection * 0.5); // 每次移动 0.5 度
    
    // 【已修改】在 0 和 90 之间 "反弹"
    if (targetAngle >= ANGLE_PER_LAP) {
        targetAngle = ANGLE_PER_LAP;
        scanDirection = -1.0; // 换向
    } else if (targetAngle <= 0.0) {
        targetAngle = 0.0;
        scanDirection = 1.0; // 换向
    }
  }
}

/**
 * @brief 【V6 单圈修改】更新 LED 状态
 * * 颜色 = 目标角度 (0-90 度)
 * * 亮度 = PID 误差
 */
void updateLedStatus(float target, float actual) {
  
  // 1. 颜色(Hue): 代表目标角度 (0度=红, 90度=绿)
  uint16_t hue = map(target, 0.0, ANGLE_PER_LAP, 0, 65535 / 3); // 0-90 度 -> 红-绿
  uint8_t saturation = 250; 
  
  // 2. 亮度(Value): 代表误差 (0度=低亮度, 45度=高亮度)
  float error = abs(target - actual);
  uint8_t value = map(error, 0, 45, 20, 255); // 误差超过 45 度就最大亮度
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