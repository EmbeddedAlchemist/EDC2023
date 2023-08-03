#include "../ExLib/Source/DeviceSupport/DeviceSupport.hpp"
#include "../ExLib/Source/FreeRTOS/FreeRTOSSupport.hpp"
#include "ExLib.hpp"
#include "u8g2.h"
#include <math.h>
#include <stdio.h>
using namespace ExLib;

I2C I2C0(I2C_Periph::I2C0, GPIO_Pin::PB2, GPIO_Pin::PB3);

uint8_t u8x8_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT: // called once during init phase of u8g2/u8x8
            break;                         // can be used to setup pins
        case U8X8_MSG_DELAY_NANO:          // delay arg_int * 1 nano second
            //_delay(1 * arg_int);
            //_delay(1);
            // System::delay(TimeInterval(1 * arg_int));
            break;
        case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
                                     // System::delay(TimeInterval(100 * arg_int));
            //_delay(arg_int);
            break;
        case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
            System::delay(TimeInterval(100 * arg_int));
            break;
        case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
            System::delay(TimeInterval(1000 * arg_int));
            break;
        case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            break;               // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
        case U8X8_MSG_GPIO_D0:   // D0 or SPI clock pin: Output level in arg_int
                                 // case U8X8_MSG_GPIO_SPI_CLOCK:
            break;
        case U8X8_MSG_GPIO_D1: // D1 or SPI data pin: Output level in arg_int
                               // case U8X8_MSG_GPIO_SPI_DATA:
            break;
        case U8X8_MSG_GPIO_D2: // D2 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D3: // D3 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D4: // D4 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D5: // D5 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D6: // D6 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D7: // D7 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_E: // E/WR pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_CS: // CS (chip select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_DC: // DC (data/cmd, A0, register select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_RESET: // Reset pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_CS1: // CS1 (chip select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_CS2: // CS2 (chip select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
            break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
        case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
            break;                    // arg_int=1: Input dir with pullup high for I2C data pin
        case U8X8_MSG_GPIO_MENU_SELECT:
            u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_NEXT:
            u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_PREV:
            u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_HOME:
            u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;
    }
    return 1;
}

uint8_t my_u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {

    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            I2C0.write((char *)arg_ptr, arg_int);
            break;
        case U8X8_MSG_BYTE_INIT:
            /* add your custom code to init i2c subsystem */
            I2C0.begin(400_kHz);
            break;
        case U8X8_MSG_BYTE_SET_DC:
            /* ignored for i2c */
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            I2C0.beginTransmission(u8x8_GetI2CAddress(u8x8) >> 1);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            I2C0.endTransmission();
            break;
        default:
            return 0;
    }
    return 1;
}

u8g2_t _u8g2;
u8g2_t *u8g2 = &_u8g2;

UART UART0(UART_Periph::UART0);

GPIO PE4(GPIO_Pin::PE4, GPIO_Mode::Input);

#include "Motor.hpp"
#include "PID.hpp"
#include "Servo.hpp"

volatile uint32_t edTime;
volatile bool edFlag;

void callback() {
    // unsigned long ulstatus;
    // ulstatus = DeviceSupport::TimerIntStatus(TIMER0_BASE, TIMER_CAPA_EVENT);
    // DeviceSupport::TimerIntClear(TIMER0_BASE, ulstatus);
    // uint32_t t = System::getMicroseconds();
    // // result.sendFromISR(t);
    // edTime = t;
    // edFlag = true;
    unsigned long ulstatus;

    // 读取中断标志位
    ulstatus = DeviceSupport::TimerIntStatus(TIMER0_BASE, TIMER_CAPA_EVENT);

    // 清除中断标志位
    DeviceSupport::TimerIntClear(TIMER0_BASE, ulstatus);

    edTime = System::getMicroseconds();
    // 输出计数完成提示
    // UARTprintf("Counting Finished!\n");
    // UART0.println("int");
    edFlag = true;

    // 因为减计数会自动停止，所以需要重新启用计数模块
    // DeviceSupport::TimerEnable(TIMER0_BASE, TIMER_A);
}

void drawResult(std::uint32_t type, float value, std::uint32_t time) {
    char buffer[64];
    u8g2_ClearBuffer(u8g2);
    u8g2_SetFont(u8g2, u8g2_font_helvB10_tf);
    std::sprintf(buffer, "Time:%dms", time / 1000);
    u8g2_DrawStr(u8g2, 0, 16, buffer);
    u8g2_DrawStr(u8g2, 0, 32, "D:-,P:-");
    u8g2_SetFont(u8g2, u8g2_font_helvB18_tf);
    if (type == 0) {
        if (value < 1000)
            std::sprintf(buffer, "%.4fpF", value);
        else if (value < 1000000)
            std::sprintf(buffer, "%.4fnF", value / 1000);
        else if (value < 1000000000)
            std::sprintf(buffer, "%.4fuF", value / 1000000);
        else
            std::sprintf(buffer, "Overload");
    } else {
        std::sprintf(buffer, "%.4fuH", value);
    }
    u8g2_DrawStr(u8g2, 0, 60, buffer);
    // vTaskEndScheduler();
    taskENTER_CRITICAL();
    u8g2_SendBuffer(u8g2);
    taskEXIT_CRITICAL();
    // vTaskStartScheduler();
}

uint32_t get50EdgeTimes() {
    uint32_t startTime = System::getMicroseconds();
    uint32_t count = 50;
    while (count) {
        while (PE4.read() == false)
            ;
        while (PE4.read() == true)
            ;
        count--;
    }
    return System::getMicroseconds() - startTime;
}

void initCCP() {
    // 启用Timer4模块
    // DeviceSupport::SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // DeviceSupport::SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // // DeviceSupport::GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);
    // DeviceSupport::GPIOPinConfigure(GPIO_PB6_T0CCP0);
    // DeviceSupport::GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
    // DeviceSupport::GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    // DeviceSupport::TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT);
    // DeviceSupport::TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    //     DeviceSupport::TimerLoadSet(TIMER0_BASE, TIMER_A, 0x8FFF);
    // DeviceSupport::TimerMatchSet(TIMER0_BASE, TIMER_A, 0x8FFF - 1000);
    // DeviceSupport::TimerIntRegister(TIMER0_BASE, TIMER_A, callback);
    // DeviceSupport::IntMasterEnable();
    // DeviceSupport::TimerIntEnable(TIMER0_BASE, TIMER_CAPA_MATCH);
    // DeviceSupport::IntEnable(INT_TIMER0A);
    DeviceSupport::SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // 启用GPIO_M作为脉冲捕捉脚
    DeviceSupport::SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    DeviceSupport::GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);
    // 配置GPIO脚为使用Timer4捕捉模式
    DeviceSupport::GPIOPinConfigure(GPIO_PF0_T0CCP0);
    DeviceSupport::GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_0);

    // 为管脚配置弱上拉模式
    DeviceSupport::GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // 初始化UART模块
    // InitConsole();

    // 配置使用Timer4的TimerA模块为边沿触发减计数模式
    DeviceSupport::TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT);

    // 使用下降沿触发
    DeviceSupport::TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);

    // 设置计数范围为0x8FFF~0X8FFA
    DeviceSupport::TimerLoadSet(TIMER0_BASE, TIMER_A, 0x8FFF);
    DeviceSupport::TimerMatchSet(TIMER0_BASE, TIMER_A, 0x8FFF - 500);

    // 注册中断处理函数以响应触发事件
    DeviceSupport::TimerIntRegister(TIMER0_BASE, TIMER_A, callback);

    // 系统总中断开
    DeviceSupport::IntMasterEnable();

    // 时钟中断允许，中断事件为Capture模式中边沿触发，计数到达预设值
    DeviceSupport::TimerIntEnable(TIMER0_BASE, TIMER_CAPA_MATCH);

    // NVIC中允许定时器A模块中断
    DeviceSupport::IntEnable(INT_TIMER0A);

    // 启动捕捉模块
    // DeviceSupport::TimerEnable(TIMER0_BASE, TIMER_A);
}

uint32_t startCCP(uint32_t count) {
    DeviceSupport::TimerEnable(TIMER0_BASE, TIMER_A);
    return System::getMicroseconds();
}

int ExLib::usr_main() {
    System::delay(500_ms);
    UART0.begin(115200);
    System::setDebugStream(UART0);
    initCCP();

    // I2C0.begin(400_kHz);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, my_u8x8_byte_i2c, u8x8_gpio_and_delay_template);
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, false);
    u8g2_ClearBuffer(u8g2);
    u8g2_SendBuffer(u8g2);
    drawResult(0, 0, 0);
    // System::delay(1_s);
    // drawResult(0, 0, 0);
    // System::delay(1_s);

    UART0.println("begin");

    while (true) {
        edFlag = false;
        uint32_t start = startCCP(1000), end;
        while (edFlag == false)
            ;
        end = edTime;
        // result.receiveWithBlocking(end);
        uint32_t during = end - start;
        // UART0.printf("%d, %d, %d\n", start, end, during);
        float cap = (((float)during / 1000000.0f / 500.f) / (0.6931471806 * (52280.0f + 2 * 51386.1f))) * 1000000000000.0f * 0.775;
        float l = (during / 500.f / 1000000.f) / (4.f * 3.14159 * 3.14159 * 0.1) * powf(10, 8);
        drawResult(1, l, during);
        UART0.println(l, 16);
        System::delay(1000_ms);
    }
    return 0;
}
