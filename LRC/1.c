/*************************************
LCR表驱动程序 V1.0
xjw01 于莆田 2011.10
**************************************/
//====================================
#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long
#include <math.h>
#include <reg52.h>

void delay(uint loop) {
    uint i;
    for (i = 0; i < loop; i++)
        ;
} // 延时函数
void delay2(uint k) {
    for (; k > 0; k--)
        delay(10000);
} // 长延时,k=100大约对应1秒

//========================AD转换=============================
sfr P1ASF = 0x9D;     // 将P1置为模拟口寄存器(使能),各位中为1的有效
sfr ADC_CONTR = 0xBC; // A/D转换控制寄存器
sfr ADC_res = 0xBD;   // A/D转换结果寄存器
sfr ADC_resl = 0xBE;  // A/D转换结果寄存器

void set_channel(char channel) {
    P1ASF = 1 << channel;
    ADC_CONTR = channel + 128; // 最高位是电源开关,低3位通道选择
    delay(1);                  // 首次打开电源应延迟，使输入稳定
}

uint getAD2() {
    ADC_CONTR |= 0x08; // 00001000,置ADC_START=1启动A/D 转换
    while (!(ADC_CONTR & 0x10))
        ;              // 等待A/D转换结束(ADC_FLAG==0)
    ADC_CONTR &= 0xE7; // 11100111,置ADC_FLAG=0清除结束标记, 置ADC_START=0关闭A/D 转换
    return ADC_res * 4 + ADC_resl;
}
/*
uchar get_AD(){
ADC_CONTR |= 0x08;             //00001000,置ADC_START=1启动A/D 转换
while( !(ADC_CONTR & 0x10) );  //等待A/D转换结束(ADC_FLAG==0)
ADC_CONTR &= 0xE7;             //11100111,置ADC_FLAG=0清除结束标记, 置ADC_START=0关闭A/D 转换
return ADC_res;
}
*/
uint getAD10() reentrant { // 10次采样
    char i;
    uint c = 0;
    for (i = 0; i < 10; i++)
        c += getAD2();
    return c;
}

//============================EEPROW偏程=========================
sfr IAP_data = 0xC2;
sfr IAP_addrH = 0xC3;
sfr IAP_addrL = 0xC4;
sfr IAP_cmd = 0xC5;
sfr IAP_trig = 0xC6;
sfr IAP_contr = 0xC7;
/********************
写字节时，可以将原有数据中的1改为0，无法将0改为1，只能使用擦除命令将0改为1
应注意，擦除命令会将整个扇区擦除
*********************/
uchar readEEP(uint k) { // 读取
    IAP_addrL = k;      // 设置读取地址的低字节，地址改变才需要设置
    IAP_addrH = k >> 8; // 设置读取地址的高字节，地址改变才需要设置
    IAP_contr = 0x81;   // 设置等待时间，1MHz以下取7，2M以下取6，3M取5，6M取4，12M取3，20M取2，24M取1，30M取0，前导1表示许档IAP
    IAP_cmd = 1;        // 读取值1，写取2，擦除取3，擦除时按所在字节整个扇区撺除
    IAP_trig = 0x5A;    // 先送5A
    IAP_trig = 0xA5;    // 先送5A再送A5立即触发
    return IAP_data;
}
void writeEEP(uint k, uchar da) { // 写入
    IAP_data = da;                // 传入数据
    IAP_addrL = k;                // 设置读取地址的低字节，地址改变才需要设置
    IAP_addrH = k >> 8;           // 设置读取地址的高字节，地址改变才需要设置
    IAP_contr = 0x81;             // 设置等待时间，1MHz以下取7，2M以下取6，3M取5，6M取4，12M取3，20M取2，24M取1，30M取0，前导1表示许档IAP
    IAP_cmd = 2;                  // 读取值1，写取2，擦除取3，擦除时按所在字节整个扇区撺除
    IAP_trig = 0x5A;              // 先送5A
    IAP_trig = 0xA5;              // 先送5A再送A5立即触发
}
void eraseEEP(uint k) { // 擦除
    IAP_addrL = k;      // 设置读取地址的低字节，地址改变才需要设置
    IAP_addrH = k >> 8; // 设置读取地址的高字节，地址改变才需要设置
    IAP_contr = 0x81;   // 设置等待时间，1MHz以下取7，2M以下取6，3M取5，6M取4，12M取3，20M取2，24M取1，30M取0，前导1表示许档IAP
    IAP_cmd = 3;        // 读取值1，写取2，擦除取3，擦除时按所在字节整个扇区撺除
    IAP_trig = 0x5A;    // 先送5A
    IAP_trig = 0xA5;    // 先送5A再送A5立即触发
}

xdata struct Ida {
    char zo; // 零点改正值
} cs;

void cs_RW(char rw) {
    uchar i, *p = &cs;
    if (rw) {
        eraseEEP(0);
        for (i = 0; i < sizeof(cs); i++)
            writeEEP(i, p);
    } else {
        for (i = 0; i < sizeof(cs); i++)
            p = readEEP(i);
    }
}

/**********
字形编码图
   32
   -
64| | 128
   -  16
1| | 8
   _. 4
   2
**********/
uchar code zk[20] = {235, 136, 179, 186, 216, 122, 123, 168, 251, 250}; // 字库
uchar code zk2[8] = {241, 25, 11, 233, 27, 50, 155, 107};               // p,n,u,m,0,k,M,G

uchar disp[6] = {168, 251, 250};
char cx = -1;    // 显示缓存,cx光标位置
sfr P1M1 = 0x91; // P1端口设置寄存器
sfr P1M0 = 0x92; // P1端口设置寄存器
sfr P0M1 = 0x93; // P0端口设置寄存器
sfr P0M0 = 0x94; // P0端口设置寄存器
sfr P2M1 = 0x95; // P2端口设置寄存器
sfr P2M0 = 0x96; // P2端口设置寄存器
sfr P3M1 = 0xB1; // P3端口设置寄存器
sfr P3M0 = 0xB2; // P3端口设置寄存器

sbit ds3 = P2 ^ 4; // 数码管扫描口
sbit ds2 = P2 ^ 5; // 数码管扫描口
sbit ds1 = P2 ^ 6; // 数码管扫描口
sbit ds0 = P2 ^ 7; // 数码管扫描口

sbit spk = P2 ^ 3; // 蜂鸣器
sbit Kb = P2 ^ 1;  // 量程开关B
sbit Ka = P2 ^ 2;  // 量程开关A

sbit DDS2 = P1 ^ 2; // 移相方波输出口
sbit K3 = P1 ^ 7;
sbit K4 = P1 ^ 6;
sbit K5 = P1 ^ 5; // 7.8kHz滤波开关
sbit K6 = P1 ^ 4;
sbit K8 = P2 ^ 0; // 100Hz滤波开关

//==============字符显示函数====================
xdata uchar menu = 1, menuB = 1;
void cls() {
    char i;
    for (i = 0; i < 6; i++)
        disp = 0;
} // 清屏
void showDig(long f) { // 显示数字
    uchar i;
    cls();
    for (i = 0; i < 6; i++) {
        disp = zk[f % 10], f /= 10;
        if (!f)
            break;
    }
}
void showDig2(float f, char dw) { // 显示浮点数
    char i, b = 0, b2 = 0, fh = 0;
    if (f < 0)
        fh = 1, f = -f;
    for (i = 0; i < 3; i++) {
        if (f >= 1000)
            f /= 1000, b++;
    } // 以3位为单位移动小数点，把大数转粉0至999,小数点在末字
    for (i = 0; i < 4; i++) {
        if (f < 1)
            f *= 1000, b--;
    } // 以3位为单位移动小数点，把小数转粉0至999,小数点在末字
    for (i = 0; i < 3; i++) {
        if (f < 1000)
            f *= 10, b2++;
    } // 对以于1000结果，连同小数点整体移位，使首位移到最左边
    showDig(f);
    disp[b2] += 4; // 小数点
    if (!dw)
        return;
    disp[0] = zk2[b + 4]; // 显示单位
    if (fh)
        disp[0] += 4; // 显示符号
}

//==============低频信号DDS====================
// PCA相关寄存器
sfr CMOD = 0xD9;   // 钟源选择控制等
sfr CH = 0xF9;     // PCA的计数器
sfr CL = 0xE9;     // PCA的计数器
sfr CCON = 0xD8;   // PCA控制寄存器
sfr CCPAM0 = 0xDA; // PCA模块0工作模式寄存器
sfr CCPAM1 = 0xDB; // PCA模块1工作模式寄存器
sfr CCAP0L = 0xEA; // 模块0捕获寄存器低位
sfr CCAP0H = 0xFA; // 模块0捕获寄存器高位

sbit PPCA = IP ^ 7;   // PCA的中断优先级设置
sbit CCF0 = CCON ^ 0; // PCA的模块0中断标志
sbit CCF1 = CCON ^ 1; // PCA的模块1中断标志
sbit CR = CCON ^ 6;   // PCA计数器使能

void PWM_init() { // 把PCA置为PWM
    CMOD = 2;     // 0000 0010 计数源选择,钟源取fosc/2
    CL = CH = 0;
    CCAP0L = CCAP0H = 192; // 占空比为25%
    // CCPAM0=0x42;//0100 0010,PCA的模块0设置为PWM模式,无中断
    CCPAM0 = 0x53; // 0101 0011,PCA的模块0设置为PWM模式,有中断，下降沿中断
    PPCA = 1;      // 优先中断
    // CR = 1;   //开始计数
    EA = 1; // 开总中断
}

uint ph = 0, phM = 256, feq = 1000; // 相位,phM相位步进值
uchar code sinB[256] = {
    // 查询表中不可装载零值，否则会造成无中断产生
    255, 255, 255, 255, 255, 255, 254, 254, 253, 252, 252, 251, 250, 249, 248, 247, 246, 245, 243, 242, 240, 239, 237, 236, 234, 232, 230, 229, 227, 225, 222, 220,
    218, 216, 214, 211, 209, 206, 204, 201, 199, 196, 194, 191, 188, 185, 183, 180, 177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 140, 137, 134, 131,
    128, 125, 122, 119, 116, 112, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82, 79, 76, 73, 71, 68, 65, 62, 60, 57, 55, 52, 50, 47, 45, 42, 40,
    38, 36, 34, 31, 29, 27, 26, 24, 22, 20, 19, 17, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 26, 27, 29, 31, 34, 36,
    38, 40, 42, 45, 47, 50, 52, 55, 57, 60, 62, 65, 68, 71, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 116, 119, 122, 125,
    128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 185, 188, 191, 194, 196, 199, 201, 204, 206, 209, 211, 214, 216,
    218, 220, 222, 225, 227, 229, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 254, 254, 255, 255, 255, 255, 255};
uchar code fbB[256] = { // 方波DDS查询表
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uchar chuX = 0;                   // 方波DDS初相
void PCAinter(void) interrupt 7 { // PCA中断
    uchar x, y;
    CCF0 = 0;         // 清除中断请求,以免反复中断
    x = ph >> 8;      // 截断正弦相位累加器,取高8位
    y = x + chuX;     // 方波相位
    CCAP0H = sinB[x]; // 正弦DDS输出
    DDS2 = fbB[y];    // 方波DDS输出
    ph += phM;        // 相位累加
}
void setDDS(uint f) { // 参考时钟是c=(fosc/2)/256=32000000/2/256=62500,频率f=c*phM/2^16
    feq = f;
    phM = f * 65536.0 / 62500; // phM=f*2^16/62500
    ph = 0;                    // 高频时，使波形对称
    if (!f)
        CR = 0;
    else
        CR = 1;
}

// 相位控制函数
xdata char xw = 0;   // 相位
void set90(char k) { // 设置方波的相位差
    k %= 4;
    if (k < 0)
        k += 4;
    if (k == 0)
        chuX = 0; // 移相0度
    if (k == 1)
        chuX = 128; // 移相180度
    if (k == 2)
        chuX = 64; // 移相90度
    if (k == 3)
        chuX = 192; // 移相270度
    xw = k;
}
void set902() { set90(xw + 1); } // 相位步进

//==============量程控制函数====================
xdata char rng = 1;   // 量程
void setRng(char k) { // 切换量程
    if (k == 0)
        Ka = 0, Kb = 0; // 100欧
    if (k == 1)
        Ka = 0, Kb = 1; // 1k欧
    if (k == 2)
        Ka = 1, Kb = 0; // 10k欧
    if (k == 3)
        Ka = 1, Kb = 1; // 100k欧
    rng = k;
}
void setRng2() { setRng((rng + 1) % 4); } // 量程步进

//==============增益控制函数====================
float gain[4] = {1, 3, 10, 30}; // 增益表
char curGain = 1;               // 当前增益索引号
void setGain(char k) {          // 设置电路增益
    if (k > 3)
        k = 3;
    if (k < 0)
        k = 0;
    if (k == 0)
        K4 = 0, K6 = 0; // 1倍
    if (k == 1)
        K4 = 0, K6 = 1; // 3倍
    if (k == 2)
        K4 = 1, K6 = 0; // 10倍
    if (k == 3)
        K4 = 1, K6 = 1; // 30倍
    curGain = k;
}
void setGain2() { setGain((curGain + 1) % 4); }

//==============设置频率====================
uchar mT = 6; // 测量速度
void setF(char k) {
    if (k == -1) { // 步进
        k = 0;
        if (feq == 100)
            k = 1;
        if (feq == 1000)
            k = 2;
        if (feq == 7813)
            k = 0;
    }
    if (k == 0) {
        setDDS(100);
        K5 = 0;
        K8 = 1;
        mT = 15;
    } // 置为100Hz
    if (k == 1) {
        setDDS(1000);
        K5 = 0;
        K8 = 0;
        mT = 6;
    } // 置为1kHz
    if (k == 2) {
        setDDS(7813);
        K5 = 1;
        K8 = 0;
        mT = 6;
    } // 置为7.8125kHz
}

//==============LCR测量====================
xdata int Vxy[4] = {0, 0, 0, 0};  // Vxy[Vx1,Vy1,Vx2,Vy2]
xdata char Vga[4] = {1, 1, 1, 1}; // 上下臂增益记录表
xdata uchar tim = 0, tims = 0;
xdata char pau = 0; // 暂停坐标自动旋转
int Vfull = 9500;
code float dwR[4] = {20, 1000, 10000, 100000}; // 档位电阻表
int absMax(int a, int b) {
    a = abs(a);
    b = abs(b);
    if (b > a)
        a = b;
    return a;
}
void timerInter1(void) interrupt 3 { // T1中断
    char a, g;
    int c;
    static int Ve0 = 0;
    if (pau)
        return;
    tims++;
    if (tims >= mT) { // tim进位触发
        tims = 0, tim++;
        if (tim >= 8)
            tim = 0;
        a = tim / 2;           // x1,y1,x2,y2指针
        c = getAD10() + cs.zo; // 读取电压值
        if (tim % 2) {
            if (Ve0 > c)
                Vxy[a] = Ve0; // 保存当前电压
            else
                Vxy[a] = -c;
            Vga[a] = curGain; // 保存当前增益
        } else
            Ve0 = c;
        if (tim == 3 || tim == 7) { // 上下臂切换
            // 电压模值才能反应运放的输出幅度，所以增益切换判断得用模值
            if (tim == 3)
                K3 = 1, c = absMax(Vxy[2], Vxy[3]), g = Vga[2]; // 切换到下臂
            if (tim == 7)
                K3 = 0, c = absMax(Vxy[0], Vxy[1]), g = Vga[0]; // 切换到上臂
            // 切换后一定要设定一个g值,不论c为何值
            setGain(g);
            if (c > Vfull)
                setGain(g - 1);
            if (c < Vfull / 33 * 10)
                setGain(g + 1);
            if (c < Vfull / 33 * 3)
                setGain(g + 2);
            if (c < Vfull / 33 * 1)
                setGain(g + 3);
        }
        set90(tim + 1); // 相位旋转
    }
}

char sfdw = 1;        // 是否显示单位
void showR(char xm) { // 显示电阻
    xdata float a, b, c, e, bs;
    xdata float x1 = Vxy[0] / gain[Vga[0]];
    xdata float y1 = Vxy[1] / gain[Vga[1]];
    xdata float x2 = -Vxy[2] / gain[Vga[2]];
    xdata float y2 = -Vxy[3] / gain[Vga[3]];
    bs = dwR[rng];
    a = x2 * x2 + y2 * y2;
    b = x1 * x2 + y1 * y2;
    c = x2 * y1 - x1 * y2;
    if (!a) {
        cls();
        disp[3] = 115;
        disp[2] = disp[1] = 97;
        return;
    }
    if (xm == 0) {
        showDig2(c / a * bs, sfdw);
    } // 显示X值
    if (xm == 1) {
        showDig2(b / a * bs, sfdw);
    } // 显示R值
    if (xm == 2) {
        showDig2(c / a * bs / 6.283 / feq, sfdw);
    } // 显示L值
    if (xm == 3) {
        showDig2(a / c / bs / 6.283 / feq, sfdw);
    }                         // 显示C值
    if (xm == 4 || xm == 9) { // 显示Q值
        if (!b) {
            showDig(9999);
            return;
        }
        c = fabs(c / b);
        if (c >= 1000) {
            showDig(999);
        } else if (c >= 100) {
            showDig(c);
        } else if (c >= 10) {
            showDig(c * 10);
            disp[1] += 4;
        } else if (c >= 1) {
            showDig(c * 100);
            disp[2] += 4;
        } else {
            showDig(c * 1000);
            disp[3] += 4;
        }
    }
    e = (b * b + c * c) / a;
    if (xm == 5) {
        showDig2(e / c * bs, sfdw);
    } // 显示并联X值
    if (xm == 6) {
        showDig2(e / b * bs, sfdw);
    } // 显示并联R值
    if (xm == 7) {
        showDig2(e / c * bs / 6.283 / feq, sfdw);
    } // 显示并联L值
    if (xm == 8) {
        showDig2(c / e / bs / 6.283 / feq, sfdw);
    } // 显示并联C值
}
// void timerInter(void) interrupt 1 {}//T0中断

void showMsg(uchar a) { // 临时跳出信息
    P0 = ~a;
    ds0 = 1, ds1 = ds2 = ds3 = 0;
    delay2(50);
}

main() {
    uchar i = 0, kn = 0, key = 0;
    uchar dispN = 0; // 显示扫描索引
    uchar spkN = 0;  // 蜂鸣器发声时长
    uchar nn = 0;
    uchar XRQ = 1;

    delay2(40); // 启动延时
    cs_RW(0);   // 读EEPROM

    TCON = 0, TMOD = 0x12; // 将T0置为自动重装定时器，T1置为定时器
    TH1 = 47, TL1 = 171;   // 20ms秒定时
    TR1 = 1;               // T1开始计数
    TR0 = 0;               // T0暂停计数
    ET1 = 1;               // T1开中断
    ET0 = 1;               // T1开中断
    EA = 1;                // 开总中断
    PT0 = 1;               // 设置优先级

    set_channel(0); // 设置AD转换通道
    P2M0 = 0xFF;    // P2.01234567置为推勉输出
    P1M0 = 0xFC;    // P1.234567置为推换口
    P1M1 = 0x03;    // P1.0置为高阻抗

    // 请注意启动延时0.5秒方可读取cs_RW
    // cs_RW(0); //读取比值基数(调零时已做开机延时，确保电压上升到可读取EEPROW)

    PWM_init(); // DDS初始化
    set90(2);   // 初始设置相位
    setRng(1);  // 初始设置量程
    setGain(1); // 初始设置增益
    setF(1);    // DDS初始设置为1kHz

    while (1) {
        // 显示disp
        nn++;
        dispN = (++dispN) % 4; // 扫描器移动
        ds0 = ds1 = ds2 = ds3 = 0;
        if (dispN == 0)
            ds0 = 1;
        if (dispN == 1)
            ds1 = 1;
        if (dispN == 2)
            ds2 = 1;
        if (dispN == 3)
            ds3 = 1;
        P0 = ~disp[dispN]; // 显示
        // 扫描键盘
        // 键盘响应
        // key = (~P3)&0xfc;
        key = ~P3;
        if (key && kn < 255)
            kn++;
        else
            kn = 0;
        for (i = 0; key; i++)
            key /= 2;
        key = i;
        if (kn == 20)
            spkN = 50;
        else
            key = 0; // 当按下一定时间后，key才有效，否则无效。spkN发声时长设置
        if (spkN)
            spkN--, spk = 0;
        else
            spk = 1; // 键盘发声
        // 菜单系统
        if (key == 8 && menu) {
            menuB = menu, menu = 0;
            key = 0;
            XRQ = -1;
        } // 菜单键
        if (key == 7 && menu)
            setRng2(); // 量程步进
        if (key == 6 && menu)
            setF(-1);    // 设置频率
        if (menu == 0) { // 显示量程和菜单
            showDig(menuB);
            if (key == 8)
                menu = menuB;
            if (key >= 1 && key <= 7)
                menu = key;
            key = 0;
        }
        if (menu == 1 || menu == 2) { // 自动LCR测量(串联)
            pau = 0;
            if (XRQ == -1)
                XRQ = 1, sfdw = 1;
            if (key >= 1 && key <= 5) { // 扩展一位显示
                if (key - 1 == XRQ) {
                    if (sfdw == 0)
                        sfdw = 1;
                    else
                        sfdw = 0;
                } else
                    sfdw = 1;
                XRQ = key - 1; // X,R,L,C,Q
            }
            if (key >= 1 && key <= 5)
                XRQ = key - 1; // X,R,L,C,Q
            if (menu == 1)
                showR(XRQ);
            else
                showR(XRQ + 5);
        }
        if (menu == 3) { // 手动调试
            pau = 1;
            if (key == 1) {
                setGain2();
                showMsg(zk[curGain]);
            } // 增益控制
            if (key == 2) {
            };
            if (key == 3) {
                K3 = ~K3;
                showMsg(zk[K3]);
            } // 切换上下臂
            if (key == 4) {
                set902();
                showMsg(zk[xw]);
            } // 相位旋转
            if (nn % 64 == 0)
                showDig(getAD10());
        }
        if (menu == 7) { // 设置零点偏移数
            if (key == 1)
                cs.zo += 5; // X键加5
            if (key == 2)
                cs.zo -= 5; // R键减5
            if (key == 3)
                cs_RW(1); // L键保存
            if (key == 4)
                cs.zo = 0; // C键清除
            showDig(abs(cs.zo));
            disp[1] += 4;
            if (cs.zo < 0)
                disp[3] = 16;
        }
        delay(4000);
    } // while end
}
