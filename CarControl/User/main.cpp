

// uint8_t u8x8_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
//     switch (msg) {
//         case U8X8_MSG_GPIO_AND_DELAY_INIT: // called once during init phase of u8g2/u8x8
//             break;                         // can be used to setup pins
//         case U8X8_MSG_DELAY_NANO:          // delay arg_int * 1 nano second
//             //_delay(1 * arg_int);
//             _delay(1);
//             break;
//         case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
//             //_delay(100 * arg_int);
//             _delay(arg_int);
//             break;
//         case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
//             _delay(100 * arg_int);
//             break;
//         case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
//             _delay(10000 * arg_int);
//             break;
//         case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
//             break;               // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
//         case U8X8_MSG_GPIO_D0:   // D0 or SPI clock pin: Output level in arg_int
//                                  // case U8X8_MSG_GPIO_SPI_CLOCK:
//             break;
//         case U8X8_MSG_GPIO_D1: // D1 or SPI data pin: Output level in arg_int
//                                // case U8X8_MSG_GPIO_SPI_DATA:
//             break;
//         case U8X8_MSG_GPIO_D2: // D2 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_D3: // D3 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_D4: // D4 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_D5: // D5 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_D6: // D6 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_D7: // D7 pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_E: // E/WR pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_CS: // CS (chip select) pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_DC: // DC (data/cmd, A0, register select) pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_RESET: // Reset pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_CS1: // CS1 (chip select) pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_CS2: // CS2 (chip select) pin: Output level in arg_int
//             break;
//         case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
//             break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
//         case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
//             break;                    // arg_int=1: Input dir with pullup high for I2C data pin
//         case U8X8_MSG_GPIO_MENU_SELECT:
//             u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
//             break;
//         case U8X8_MSG_GPIO_MENU_NEXT:
//             u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
//             break;
//         case U8X8_MSG_GPIO_MENU_PREV:
//             u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
//             break;
//         case U8X8_MSG_GPIO_MENU_HOME:
//             u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
//             break;
//         default:
//             u8x8_SetGPIOResult(u8x8, 1); // default return value
//             break;
//     }
//     return 1;
// }

// uint8_t my_u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {

//    switch (msg) {
//        case U8X8_MSG_BYTE_SEND:
//            I2C0.write((char *)arg_ptr, arg_int);
//            break;
//        case U8X8_MSG_BYTE_INIT:
//            /* add your custom code to init i2c subsystem */
//            I2C0.begin(400_kHz);
//            break;
//        case U8X8_MSG_BYTE_SET_DC:
//            /* ignored for i2c */
//            break;
//        case U8X8_MSG_BYTE_START_TRANSFER:
//            I2C0.beginTransmission(u8x8_GetI2CAddress(u8x8) >> 1);
//            break;
//        case U8X8_MSG_BYTE_END_TRANSFER:
//            I2C0.endTransmission();
//            break;
//        default:
//            return 0;
//    }
//    return 1;
//}

#include "ExLib.hpp"
using namespace ExLib;

UART UART0(UART_Periph::UART0);

#include "Motor.hpp"
#include "PID.hpp"
#include "Servo.hpp"

void callback(void *unused) {
}

constexpr float kp = 0.0040,
                ki = 0.0005,
                kd = 0.0005;

HardwarePWM Motor0PWMGenerator(HardwarePWM_Periph::Module0Generator3);
PWM_Channel Motor0A(Motor0PWMGenerator, 6, GPIO_Pin::PD0);
PWM_Channel Motor0B(Motor0PWMGenerator, 7, GPIO_Pin::PD1);
Motor Motor0(Motor0A, Motor0B);
QuadraticEncoder Motor0Encoder(QuadraticEncoder_Periph::QuadraticEncoder1, GPIO_Pin::PC5, GPIO_Pin::PC6);

PID_Add Motor0PID(kp, ki, kd);

HardwarePWM Motor1PWMGenerator(HardwarePWM_Periph::Module1Generator3);
PWM_Channel Motor1A(Motor1PWMGenerator, 7, GPIO_Pin::PF3);
PWM_Channel Motor1B(Motor1PWMGenerator, 6, GPIO_Pin::PF2);
Motor Motor1(Motor1A, Motor1B);
QuadraticEncoder Motor1Encoder(QuadraticEncoder_Periph::QuadraticEncoder0, GPIO_Pin::PF0, GPIO_Pin::PF1);

// PID_Add Motor1PID(0.0035 , 0.0005, 0.0);
PID_Add Motor1PID(kp, ki, kd);

HardwarePWM ServoPWMGenerator(HardwarePWM_Periph::Module1Generator1);
PWM_Channel ServoXChannel(ServoPWMGenerator, 2, GPIO_Pin::PA6);
PWM_Channel ServoYChannel(ServoPWMGenerator, 3, GPIO_Pin::PA7);
Servo ServoX(ServoXChannel, -180, 180);
Servo ServoY(ServoYChannel, -180, 180);

I2C I2C0(I2C_Periph::I2C0, GPIO_Pin::PB2, GPIO_Pin::PB3);




int ExLib::usr_main() {
    System::delay(500_ms);
    UART0.begin(115200);
    UART0.onReceive(*new CallbackFunction(callback));
    System::setDebugStream(UART0);
    Motor0Encoder.begin(true);
    Motor1Encoder.begin(true);
    Motor0PID.setOutputLimits(-1, 1);
    Motor0PID.setTarget(1);
    Motor1PID.setOutputLimits(-1, 1);
    Motor1PID.setTarget(1);
    ServoX.setAngel(0);
    ServoY.setAngel(0);
    I2C0.begin(100_kHz);
    // ServoXChannel.setDuty(0.08);
    // ServoYChannel.setDuty(0.08);
    UART0.println("Begin!");
    I2C0.beginTransmission(0X0D);
    I2C0.write(0x0D);
    UART0.print(I2C0.read(), 16);
    UART0.println();
    I2C0.endTransmission();

    I2C0.beginTransmission(0X0D);
    I2C0.write(0x02);
    I2C0.write(0x00);
    I2C0.endTransmission();
    UART0.println("Inited");
    while (true) {
        int x, y, z;

        I2C0.beginTransmission(0X0D);
        I2C0.write(0x03);
        x = (int)I2C0.read() << 8;
        x |= I2C0.read();
        z = (int)I2C0.read() << 8;
        z |= I2C0.read();
        y = (int)I2C0.read() << 8;
        y |= I2C0.read();
        I2C0.endTransmission();
        UART0.printf("%d,%d,%d\n", x, y, z);
        System::delay(250_ms);
    }
    return 0;
}
