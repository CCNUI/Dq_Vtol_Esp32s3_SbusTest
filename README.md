### `README.md`



# ESP32-S3 SBUS Output Tester / ESP32-S3 SBUS 输出测试程序



[English](https://github.com/CCNUI/Dq_Vtol_Esp32s3_SbusTest/blob/main/README.md?q=%23en) | [中文](https://github.com/CCNUI/Dq_Vtol_Esp32s3_SbusTest/blob/main/README.md?q=%23cn)



<a name="en"></a>



## ESP32-S3 SBUS Output Tester (EN)



This is a 16-channel SBUS output (transmitter) testing tool for the ESP32-S3 running the Arduino framework. It uses a hardware UART (Serial1) to generate an SBUS protocol compliant signal (100,000 baud, 8E2, inverted logic) on any GPIO.



This project is configured to output the signal on **GPIO 14** and uses the **on-board RGB LED (GPIO 48)** of the ESP32-S3-DevKitC as a status indicator.



The primary function of this tool is to **individually test** servos or other devices connected to an SBUS bus.



-----



### Features



&nbsp; * **16 Channels**: Full 16-channel SBUS PWM output (1000µs - 2000µs).

&nbsp; * **50Hz Refresh Rate**: Synchronously updates all channels every 20ms.

&nbsp; * **Standard SBUS Protocol**: 100,000 baud, 8E2 format, inverted serial logic.

&nbsp; * **Flexible GPIO**: Easily modify `SBUS_TX_PIN` to use almost any GPIO on the ESP32-S3.

&nbsp; * **Dual Test Modes**:

&nbsp;   1.  **Auto-Scan Mode (Default)**: On startup, the program automatically scans all channels one by one. It tests CH1 (5 sec), then automatically switches to CH2 (5 sec), and so on, looping back to CH1.

&nbsp;   2.  **Manual Test Mode**: The user can interrupt the auto-scan at any time by sending a channel number (e.g., `5`) via the serial monitor to lock onto that channel for continuous testing.

&nbsp; * **Advanced Status LED (GPIO 48)**:

&nbsp;     * **Color (Hue)**: Represents the currently tested channel (CH1-CH16).

&nbsp;     * **Brightness (Value)**: Represents the channel's PWM value (low value = dim, high value = bright).



### Hardware Requirements



&nbsp; * **ESP32-S3 Dev Board** (Example based on ESP32-S3-DevKitC)

&nbsp; * **SBUS Receiving Device** (e.g., Servo, Flight Controller, or SBUS Decoder)

&nbsp; * (Optional) Logic Analyzer (for signal verification)



### Software Dependencies (Arduino IDE)



1.  **ESP32 Arduino Core**: Ensure you have installed `esp32` support (v2.0.x or higher) in the Boards Manager.

2.  **Adafruit NeoPixel**: Used to control the on-board RGB LED. Install it via the Library Manager.



### Hardware Connection



```

ESP32-S3 (IO14)  ----->  SBUS Signal Pin (RX)

ESP32-S3 (GND)   ----->  SBUS Ground Pin (GND)

```



**Important**: SBUS uses inverted logic. Most flight controllers (FC) or SBUS servos have a built-in inverter on their `SBUS` port. If you are connecting to a standard 5V UART RX pin, you may need an external inverter (e.g., a simple NPN transistor) between the ESP32's TX and the device's RX.



### How to Use (Test Procedure)



1.  Upload the code to your ESP32-S3.

2.  Open the Arduino **Serial Monitor** with the baud rate set to **115200**.



#### 1. Auto-Scan Mode (Default)



&nbsp; * On startup, you will see the prompt `Defaulting to Auto-Scan Mode`.

&nbsp; * It will begin by testing **CH1**. The PWM value for CH1 will sweep back and forth between 1000-2000µs, while all other channels (CH2-16) remain at 1500µs (neutral).

&nbsp; * The LED will show **Red (color for CH1)**, and its brightness will change with the PWM value.

&nbsp; * **After 5 seconds**, it will automatically switch to **CH2**. CH2 will begin to sweep, and all other channels will be reset to 1500µs. The LED will turn **Orange (color for CH2)**.

&nbsp; * This process will cycle through all 16 channels and then repeat from CH1.



#### 2. Manual Test Mode



&nbsp; * At any time, if you want to **stop the auto-scan** and focus on a specific channel (e.g., Channel 5):

&nbsp; * In the text box at the top of the Serial Monitor, type `5` (or `c5`) and **press Enter**.

&nbsp; * The monitor will print `*** Manual Mode: Testing Channel 5 ***`.

&nbsp; * Now, only **CH5** will sweep from 1000-2000µs. All other channels will be held at 1500µs.

&nbsp; * The LED color will lock to **Cyan (color for CH5)**.



#### 3. Return to Auto-Scan



&nbsp; * While in manual mode, if you wish to return to auto-scanning:

&nbsp; * Type `auto` in the serial input box and press Enter.

&nbsp; * The program will resume auto-scanning, starting with the channel *after* the one you were manually testing (e.g., it will start scanning CH6).



-----



<a name="cn"></a>



## ESP32-S3 SBUS 输出测试程序 (中文)



这是一个为 ESP32-S3 (Arduino 框架) 提供的 16 通道 SBUS 输出（发射）测试工具。它使用硬件 UART (Serial1) 在任意 GPIO 上生成符合 SBUS 协议（100,000 波特率, 8E2, 反向逻辑）的信号。



项目配置为在 **GPIO 14** 上输出信号，并使用 ESP32-S3-DevKitC 上的**板载 RGB LED (GPIO 48)** 作为状态指示灯。



这个工具的主要功能是**逐个测试**连接到 SBUS 总线上的舵机或其他设备。



-----



### 功能



&nbsp; * **16 通道**：全 16 通道 SBUS PWM 输出 (1000µs - 2000µs)。

&nbsp; * **50Hz 刷新率**：以 20ms 的间隔同步更新所有通道。

&nbsp; * **标准 SBUS 协议**：100,000 波特率, 8E2 格式, 反向串行逻辑。

&nbsp; * **GPIO 灵活**：可轻松修改 `SBUS_TX_PIN` 以使用 ESP32-S3 上的几乎任何 GPIO。

&nbsp; * **双测试模式**：

&nbsp;   1.  **自动扫描模式 (默认)**：程序启动时，会自动逐个扫描所有通道。它会先扫描 CH1 (5秒)，然后自动切换到 CH2 (5秒)，依此类推，循环往复。

&nbsp;   2.  **手动测试模式**：用户可以随时通过串口监视器输入通道号 (如 `5`) 来打断自动扫描，并锁定在该通道上进行持续测试。

&nbsp; * **高级状态指示灯 (GPIO 48)**：

&nbsp;     * **颜色 (Hue)**：代表当前正在测试的通道 (CH1-CH16)。

&nbsp;     * **亮度 (Value)**：代表该通道的 PWM 值（低位=低亮度，高位=高亮度）。



### 硬件要求



&nbsp; * **ESP32-S3 开发板** (示例基于 ESP32-S3-DevKitC)

&nbsp; * **SBUS 接收设备** (如舵机、飞控或 SBUS 解码器)

&nbsp; * (可选) 逻辑分析仪 (用于验证信号)



### 软件依赖 (Arduino IDE)



1.  **ESP32 Arduino 核心**：确保已在“开发板管理器”中安装了 `esp32` 支持 (v2.0.x 或更高版本)。

2.  **Adafruit NeoPixel**：用于控制板载 RGB LED。请通过“库管理器”安装。



### 硬件连接



```

ESP32-S3 (IO14)  ----->  SBUS 信号线 (RX)

ESP32-S3 (GND)   ----->  SBUS 地线 (GND)

```



**重要**：SBUS 是反向逻辑。大多数飞控 (FC) 或 SBUS 舵机的 `SBUS` 端口已经内置了反向器。如果你要连接到一个标准的 5V UART RX 引脚，你可能需要在 ESP32 的 TX 和 设备的 RX 之间连接一个外部反向器（例如一个简单的 NPN 晶体管）。



### 如何使用（测试流程）



1.  上传代码到你的 ESP32-S3。

2.  打开 Arduino **串口监视器** (Serial Monitor)，波特率设置为 **115200**。



#### 1. 自动扫描模式 (默认)



&nbsp; * 程序启动后，你会看到提示 `默认启动自动扫描模式`。

&nbsp; * 它将首先测试 **CH1**。CH1 的 PWM 值会在 1000-2000µs 之间来回扫描，而所有其他通道 (CH2-16) 将保持在 1500µs (中立位)。

&nbsp; * LED 灯将显示**红色 (CH1 对应的颜色)**，并且亮度会随 PWM 值变化。

&nbsp; * **5 秒后**，它会自动切换到 **CH2**。CH2 开始扫描，其他通道重置为 1500us。LED 灯变为**橙色 (CH2 对应的颜色)**。

&nbsp; * 这个过程会依次遍历所有 16 个通道，然后从 CH1 重新开始。



#### 2. 手动测试模式



&nbsp; * 在任何时候，如果你想**停止自动扫描**并专注于测试一个特定通道（例如，通道 5）：

&nbsp; * 在串口监视器顶部的**输入框**中，输入 `5` (或者 `c5`)，然后**按回车键**。

&nbsp; * 程序会打印 `*** 手动模式: </em></em> 正在测试通道 5 ***`。

&nbsp; * 现在，只有 **CH5** 会进行 1000-2000us 的扫描，所有其他通道都会停止在 1500us。

&nbsp; * LED 灯的颜色会锁定为**青色 (CH5 对应的颜色)**。



#### 3. 返回自动扫描



&nbsp; * 在手动模式下，如果你想返回自动扫描：

&nbsp; * 在串口输入框中输入 `auto` 并按回车。

&nbsp; * 程序将从你刚才测试的通道 (CH5) **之后**的那个通道 (CH6) 开始，继续进行自动扫描。

