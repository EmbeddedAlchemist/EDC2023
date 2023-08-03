/*************************************
LCR���������� V1.0
xjw01 ������ 2011.10
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
} // ��ʱ����
void delay2(uint k) {
    for (; k > 0; k--)
        delay(10000);
} // ����ʱ,k=100��Լ��Ӧ1��

//========================ADת��=============================
sfr P1ASF = 0x9D;     // ��P1��Ϊģ��ڼĴ���(ʹ��),��λ��Ϊ1����Ч
sfr ADC_CONTR = 0xBC; // A/Dת�����ƼĴ���
sfr ADC_res = 0xBD;   // A/Dת������Ĵ���
sfr ADC_resl = 0xBE;  // A/Dת������Ĵ���

void set_channel(char channel) {
    P1ASF = 1 << channel;
    ADC_CONTR = channel + 128; // ���λ�ǵ�Դ����,��3λͨ��ѡ��
    delay(1);                  // �״δ򿪵�ԴӦ�ӳ٣�ʹ�����ȶ�
}

uint getAD2() {
    ADC_CONTR |= 0x08; // 00001000,��ADC_START=1����A/D ת��
    while (!(ADC_CONTR & 0x10))
        ;              // �ȴ�A/Dת������(ADC_FLAG==0)
    ADC_CONTR &= 0xE7; // 11100111,��ADC_FLAG=0����������, ��ADC_START=0�ر�A/D ת��
    return ADC_res * 4 + ADC_resl;
}
/*
uchar get_AD(){
ADC_CONTR |= 0x08;             //00001000,��ADC_START=1����A/D ת��
while( !(ADC_CONTR & 0x10) );  //�ȴ�A/Dת������(ADC_FLAG==0)
ADC_CONTR &= 0xE7;             //11100111,��ADC_FLAG=0����������, ��ADC_START=0�ر�A/D ת��
return ADC_res;
}
*/
uint getAD10() reentrant { // 10�β���
    char i;
    uint c = 0;
    for (i = 0; i < 10; i++)
        c += getAD2();
    return c;
}

//============================EEPROWƫ��=========================
sfr IAP_data = 0xC2;
sfr IAP_addrH = 0xC3;
sfr IAP_addrL = 0xC4;
sfr IAP_cmd = 0xC5;
sfr IAP_trig = 0xC6;
sfr IAP_contr = 0xC7;
/********************
д�ֽ�ʱ�����Խ�ԭ�������е�1��Ϊ0���޷���0��Ϊ1��ֻ��ʹ�ò������0��Ϊ1
Ӧע�⣬��������Ὣ������������
*********************/
uchar readEEP(uint k) { // ��ȡ
    IAP_addrL = k;      // ���ö�ȡ��ַ�ĵ��ֽڣ���ַ�ı����Ҫ����
    IAP_addrH = k >> 8; // ���ö�ȡ��ַ�ĸ��ֽڣ���ַ�ı����Ҫ����
    IAP_contr = 0x81;   // ���õȴ�ʱ�䣬1MHz����ȡ7��2M����ȡ6��3Mȡ5��6Mȡ4��12Mȡ3��20Mȡ2��24Mȡ1��30Mȡ0��ǰ��1��ʾ��IAP
    IAP_cmd = 1;        // ��ȡֵ1��дȡ2������ȡ3������ʱ�������ֽ���������ߥ��
    IAP_trig = 0x5A;    // ����5A
    IAP_trig = 0xA5;    // ����5A����A5��������
    return IAP_data;
}
void writeEEP(uint k, uchar da) { // д��
    IAP_data = da;                // ��������
    IAP_addrL = k;                // ���ö�ȡ��ַ�ĵ��ֽڣ���ַ�ı����Ҫ����
    IAP_addrH = k >> 8;           // ���ö�ȡ��ַ�ĸ��ֽڣ���ַ�ı����Ҫ����
    IAP_contr = 0x81;             // ���õȴ�ʱ�䣬1MHz����ȡ7��2M����ȡ6��3Mȡ5��6Mȡ4��12Mȡ3��20Mȡ2��24Mȡ1��30Mȡ0��ǰ��1��ʾ��IAP
    IAP_cmd = 2;                  // ��ȡֵ1��дȡ2������ȡ3������ʱ�������ֽ���������ߥ��
    IAP_trig = 0x5A;              // ����5A
    IAP_trig = 0xA5;              // ����5A����A5��������
}
void eraseEEP(uint k) { // ����
    IAP_addrL = k;      // ���ö�ȡ��ַ�ĵ��ֽڣ���ַ�ı����Ҫ����
    IAP_addrH = k >> 8; // ���ö�ȡ��ַ�ĸ��ֽڣ���ַ�ı����Ҫ����
    IAP_contr = 0x81;   // ���õȴ�ʱ�䣬1MHz����ȡ7��2M����ȡ6��3Mȡ5��6Mȡ4��12Mȡ3��20Mȡ2��24Mȡ1��30Mȡ0��ǰ��1��ʾ��IAP
    IAP_cmd = 3;        // ��ȡֵ1��дȡ2������ȡ3������ʱ�������ֽ���������ߥ��
    IAP_trig = 0x5A;    // ����5A
    IAP_trig = 0xA5;    // ����5A����A5��������
}

xdata struct Ida {
    char zo; // ������ֵ
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
���α���ͼ
   32
   -
64| | 128
   -  16
1| | 8
   _. 4
   2
**********/
uchar code zk[20] = {235, 136, 179, 186, 216, 122, 123, 168, 251, 250}; // �ֿ�
uchar code zk2[8] = {241, 25, 11, 233, 27, 50, 155, 107};               // p,n,u,m,0,k,M,G

uchar disp[6] = {168, 251, 250};
char cx = -1;    // ��ʾ����,cx���λ��
sfr P1M1 = 0x91; // P1�˿����üĴ���
sfr P1M0 = 0x92; // P1�˿����üĴ���
sfr P0M1 = 0x93; // P0�˿����üĴ���
sfr P0M0 = 0x94; // P0�˿����üĴ���
sfr P2M1 = 0x95; // P2�˿����üĴ���
sfr P2M0 = 0x96; // P2�˿����üĴ���
sfr P3M1 = 0xB1; // P3�˿����üĴ���
sfr P3M0 = 0xB2; // P3�˿����üĴ���

sbit ds3 = P2 ^ 4; // �����ɨ���
sbit ds2 = P2 ^ 5; // �����ɨ���
sbit ds1 = P2 ^ 6; // �����ɨ���
sbit ds0 = P2 ^ 7; // �����ɨ���

sbit spk = P2 ^ 3; // ������
sbit Kb = P2 ^ 1;  // ���̿���B
sbit Ka = P2 ^ 2;  // ���̿���A

sbit DDS2 = P1 ^ 2; // ���෽�������
sbit K3 = P1 ^ 7;
sbit K4 = P1 ^ 6;
sbit K5 = P1 ^ 5; // 7.8kHz�˲�����
sbit K6 = P1 ^ 4;
sbit K8 = P2 ^ 0; // 100Hz�˲�����

//==============�ַ���ʾ����====================
xdata uchar menu = 1, menuB = 1;
void cls() {
    char i;
    for (i = 0; i < 6; i++)
        disp = 0;
} // ����
void showDig(long f) { // ��ʾ����
    uchar i;
    cls();
    for (i = 0; i < 6; i++) {
        disp = zk[f % 10], f /= 10;
        if (!f)
            break;
    }
}
void showDig2(float f, char dw) { // ��ʾ������
    char i, b = 0, b2 = 0, fh = 0;
    if (f < 0)
        fh = 1, f = -f;
    for (i = 0; i < 3; i++) {
        if (f >= 1000)
            f /= 1000, b++;
    } // ��3λΪ��λ�ƶ�С���㣬�Ѵ���ת��0��999,С������ĩ��
    for (i = 0; i < 4; i++) {
        if (f < 1)
            f *= 1000, b--;
    } // ��3λΪ��λ�ƶ�С���㣬��С��ת��0��999,С������ĩ��
    for (i = 0; i < 3; i++) {
        if (f < 1000)
            f *= 10, b2++;
    } // ������1000�������ͬС����������λ��ʹ��λ�Ƶ������
    showDig(f);
    disp[b2] += 4; // С����
    if (!dw)
        return;
    disp[0] = zk2[b + 4]; // ��ʾ��λ
    if (fh)
        disp[0] += 4; // ��ʾ����
}

//==============��Ƶ�ź�DDS====================
// PCA��ؼĴ���
sfr CMOD = 0xD9;   // ��Դѡ����Ƶ�
sfr CH = 0xF9;     // PCA�ļ�����
sfr CL = 0xE9;     // PCA�ļ�����
sfr CCON = 0xD8;   // PCA���ƼĴ���
sfr CCPAM0 = 0xDA; // PCAģ��0����ģʽ�Ĵ���
sfr CCPAM1 = 0xDB; // PCAģ��1����ģʽ�Ĵ���
sfr CCAP0L = 0xEA; // ģ��0����Ĵ�����λ
sfr CCAP0H = 0xFA; // ģ��0����Ĵ�����λ

sbit PPCA = IP ^ 7;   // PCA���ж����ȼ�����
sbit CCF0 = CCON ^ 0; // PCA��ģ��0�жϱ�־
sbit CCF1 = CCON ^ 1; // PCA��ģ��1�жϱ�־
sbit CR = CCON ^ 6;   // PCA������ʹ��

void PWM_init() { // ��PCA��ΪPWM
    CMOD = 2;     // 0000 0010 ����Դѡ��,��Դȡfosc/2
    CL = CH = 0;
    CCAP0L = CCAP0H = 192; // ռ�ձ�Ϊ25%
    // CCPAM0=0x42;//0100 0010,PCA��ģ��0����ΪPWMģʽ,���ж�
    CCPAM0 = 0x53; // 0101 0011,PCA��ģ��0����ΪPWMģʽ,���жϣ��½����ж�
    PPCA = 1;      // �����ж�
    // CR = 1;   //��ʼ����
    EA = 1; // �����ж�
}

uint ph = 0, phM = 256, feq = 1000; // ��λ,phM��λ����ֵ
uchar code sinB[256] = {
    // ��ѯ���в���װ����ֵ�������������жϲ���
    255, 255, 255, 255, 255, 255, 254, 254, 253, 252, 252, 251, 250, 249, 248, 247, 246, 245, 243, 242, 240, 239, 237, 236, 234, 232, 230, 229, 227, 225, 222, 220,
    218, 216, 214, 211, 209, 206, 204, 201, 199, 196, 194, 191, 188, 185, 183, 180, 177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 140, 137, 134, 131,
    128, 125, 122, 119, 116, 112, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82, 79, 76, 73, 71, 68, 65, 62, 60, 57, 55, 52, 50, 47, 45, 42, 40,
    38, 36, 34, 31, 29, 27, 26, 24, 22, 20, 19, 17, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 26, 27, 29, 31, 34, 36,
    38, 40, 42, 45, 47, 50, 52, 55, 57, 60, 62, 65, 68, 71, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 116, 119, 122, 125,
    128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 185, 188, 191, 194, 196, 199, 201, 204, 206, 209, 211, 214, 216,
    218, 220, 222, 225, 227, 229, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 254, 254, 255, 255, 255, 255, 255};
uchar code fbB[256] = { // ����DDS��ѯ��
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uchar chuX = 0;                   // ����DDS����
void PCAinter(void) interrupt 7 { // PCA�ж�
    uchar x, y;
    CCF0 = 0;         // ����ж�����,���ⷴ���ж�
    x = ph >> 8;      // �ض�������λ�ۼ���,ȡ��8λ
    y = x + chuX;     // ������λ
    CCAP0H = sinB[x]; // ����DDS���
    DDS2 = fbB[y];    // ����DDS���
    ph += phM;        // ��λ�ۼ�
}
void setDDS(uint f) { // �ο�ʱ����c=(fosc/2)/256=32000000/2/256=62500,Ƶ��f=c*phM/2^16
    feq = f;
    phM = f * 65536.0 / 62500; // phM=f*2^16/62500
    ph = 0;                    // ��Ƶʱ��ʹ���ζԳ�
    if (!f)
        CR = 0;
    else
        CR = 1;
}

// ��λ���ƺ���
xdata char xw = 0;   // ��λ
void set90(char k) { // ���÷�������λ��
    k %= 4;
    if (k < 0)
        k += 4;
    if (k == 0)
        chuX = 0; // ����0��
    if (k == 1)
        chuX = 128; // ����180��
    if (k == 2)
        chuX = 64; // ����90��
    if (k == 3)
        chuX = 192; // ����270��
    xw = k;
}
void set902() { set90(xw + 1); } // ��λ����

//==============���̿��ƺ���====================
xdata char rng = 1;   // ����
void setRng(char k) { // �л�����
    if (k == 0)
        Ka = 0, Kb = 0; // 100ŷ
    if (k == 1)
        Ka = 0, Kb = 1; // 1kŷ
    if (k == 2)
        Ka = 1, Kb = 0; // 10kŷ
    if (k == 3)
        Ka = 1, Kb = 1; // 100kŷ
    rng = k;
}
void setRng2() { setRng((rng + 1) % 4); } // ���̲���

//==============������ƺ���====================
float gain[4] = {1, 3, 10, 30}; // �����
char curGain = 1;               // ��ǰ����������
void setGain(char k) {          // ���õ�·����
    if (k > 3)
        k = 3;
    if (k < 0)
        k = 0;
    if (k == 0)
        K4 = 0, K6 = 0; // 1��
    if (k == 1)
        K4 = 0, K6 = 1; // 3��
    if (k == 2)
        K4 = 1, K6 = 0; // 10��
    if (k == 3)
        K4 = 1, K6 = 1; // 30��
    curGain = k;
}
void setGain2() { setGain((curGain + 1) % 4); }

//==============����Ƶ��====================
uchar mT = 6; // �����ٶ�
void setF(char k) {
    if (k == -1) { // ����
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
    } // ��Ϊ100Hz
    if (k == 1) {
        setDDS(1000);
        K5 = 0;
        K8 = 0;
        mT = 6;
    } // ��Ϊ1kHz
    if (k == 2) {
        setDDS(7813);
        K5 = 1;
        K8 = 0;
        mT = 6;
    } // ��Ϊ7.8125kHz
}

//==============LCR����====================
xdata int Vxy[4] = {0, 0, 0, 0};  // Vxy[Vx1,Vy1,Vx2,Vy2]
xdata char Vga[4] = {1, 1, 1, 1}; // ���±������¼��
xdata uchar tim = 0, tims = 0;
xdata char pau = 0; // ��ͣ�����Զ���ת
int Vfull = 9500;
code float dwR[4] = {20, 1000, 10000, 100000}; // ��λ�����
int absMax(int a, int b) {
    a = abs(a);
    b = abs(b);
    if (b > a)
        a = b;
    return a;
}
void timerInter1(void) interrupt 3 { // T1�ж�
    char a, g;
    int c;
    static int Ve0 = 0;
    if (pau)
        return;
    tims++;
    if (tims >= mT) { // tim��λ����
        tims = 0, tim++;
        if (tim >= 8)
            tim = 0;
        a = tim / 2;           // x1,y1,x2,y2ָ��
        c = getAD10() + cs.zo; // ��ȡ��ѹֵ
        if (tim % 2) {
            if (Ve0 > c)
                Vxy[a] = Ve0; // ���浱ǰ��ѹ
            else
                Vxy[a] = -c;
            Vga[a] = curGain; // ���浱ǰ����
        } else
            Ve0 = c;
        if (tim == 3 || tim == 7) { // ���±��л�
            // ��ѹģֵ���ܷ�Ӧ�˷ŵ�������ȣ����������л��жϵ���ģֵ
            if (tim == 3)
                K3 = 1, c = absMax(Vxy[2], Vxy[3]), g = Vga[2]; // �л����±�
            if (tim == 7)
                K3 = 0, c = absMax(Vxy[0], Vxy[1]), g = Vga[0]; // �л����ϱ�
            // �л���һ��Ҫ�趨һ��gֵ,����cΪ��ֵ
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
        set90(tim + 1); // ��λ��ת
    }
}

char sfdw = 1;        // �Ƿ���ʾ��λ
void showR(char xm) { // ��ʾ����
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
    } // ��ʾXֵ
    if (xm == 1) {
        showDig2(b / a * bs, sfdw);
    } // ��ʾRֵ
    if (xm == 2) {
        showDig2(c / a * bs / 6.283 / feq, sfdw);
    } // ��ʾLֵ
    if (xm == 3) {
        showDig2(a / c / bs / 6.283 / feq, sfdw);
    }                         // ��ʾCֵ
    if (xm == 4 || xm == 9) { // ��ʾQֵ
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
    } // ��ʾ����Xֵ
    if (xm == 6) {
        showDig2(e / b * bs, sfdw);
    } // ��ʾ����Rֵ
    if (xm == 7) {
        showDig2(e / c * bs / 6.283 / feq, sfdw);
    } // ��ʾ����Lֵ
    if (xm == 8) {
        showDig2(c / e / bs / 6.283 / feq, sfdw);
    } // ��ʾ����Cֵ
}
// void timerInter(void) interrupt 1 {}//T0�ж�

void showMsg(uchar a) { // ��ʱ������Ϣ
    P0 = ~a;
    ds0 = 1, ds1 = ds2 = ds3 = 0;
    delay2(50);
}

main() {
    uchar i = 0, kn = 0, key = 0;
    uchar dispN = 0; // ��ʾɨ������
    uchar spkN = 0;  // ����������ʱ��
    uchar nn = 0;
    uchar XRQ = 1;

    delay2(40); // ������ʱ
    cs_RW(0);   // ��EEPROM

    TCON = 0, TMOD = 0x12; // ��T0��Ϊ�Զ���װ��ʱ����T1��Ϊ��ʱ��
    TH1 = 47, TL1 = 171;   // 20ms�붨ʱ
    TR1 = 1;               // T1��ʼ����
    TR0 = 0;               // T0��ͣ����
    ET1 = 1;               // T1���ж�
    ET0 = 1;               // T1���ж�
    EA = 1;                // �����ж�
    PT0 = 1;               // �������ȼ�

    set_channel(0); // ����ADת��ͨ��
    P2M0 = 0xFF;    // P2.01234567��Ϊ�������
    P1M0 = 0xFC;    // P1.234567��Ϊ�ƻ���
    P1M1 = 0x03;    // P1.0��Ϊ���迹

    // ��ע��������ʱ0.5�뷽�ɶ�ȡcs_RW
    // cs_RW(0); //��ȡ��ֵ����(����ʱ����������ʱ��ȷ����ѹ�������ɶ�ȡEEPROW)

    PWM_init(); // DDS��ʼ��
    set90(2);   // ��ʼ������λ
    setRng(1);  // ��ʼ��������
    setGain(1); // ��ʼ��������
    setF(1);    // DDS��ʼ����Ϊ1kHz

    while (1) {
        // ��ʾdisp
        nn++;
        dispN = (++dispN) % 4; // ɨ�����ƶ�
        ds0 = ds1 = ds2 = ds3 = 0;
        if (dispN == 0)
            ds0 = 1;
        if (dispN == 1)
            ds1 = 1;
        if (dispN == 2)
            ds2 = 1;
        if (dispN == 3)
            ds3 = 1;
        P0 = ~disp[dispN]; // ��ʾ
        // ɨ�����
        // ������Ӧ
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
            key = 0; // ������һ��ʱ���key����Ч��������Ч��spkN����ʱ������
        if (spkN)
            spkN--, spk = 0;
        else
            spk = 1; // ���̷���
        // �˵�ϵͳ
        if (key == 8 && menu) {
            menuB = menu, menu = 0;
            key = 0;
            XRQ = -1;
        } // �˵���
        if (key == 7 && menu)
            setRng2(); // ���̲���
        if (key == 6 && menu)
            setF(-1);    // ����Ƶ��
        if (menu == 0) { // ��ʾ���̺Ͳ˵�
            showDig(menuB);
            if (key == 8)
                menu = menuB;
            if (key >= 1 && key <= 7)
                menu = key;
            key = 0;
        }
        if (menu == 1 || menu == 2) { // �Զ�LCR����(����)
            pau = 0;
            if (XRQ == -1)
                XRQ = 1, sfdw = 1;
            if (key >= 1 && key <= 5) { // ��չһλ��ʾ
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
        if (menu == 3) { // �ֶ�����
            pau = 1;
            if (key == 1) {
                setGain2();
                showMsg(zk[curGain]);
            } // �������
            if (key == 2) {
            };
            if (key == 3) {
                K3 = ~K3;
                showMsg(zk[K3]);
            } // �л����±�
            if (key == 4) {
                set902();
                showMsg(zk[xw]);
            } // ��λ��ת
            if (nn % 64 == 0)
                showDig(getAD10());
        }
        if (menu == 7) { // �������ƫ����
            if (key == 1)
                cs.zo += 5; // X����5
            if (key == 2)
                cs.zo -= 5; // R����5
            if (key == 3)
                cs_RW(1); // L������
            if (key == 4)
                cs.zo = 0; // C�����
            showDig(abs(cs.zo));
            disp[1] += 4;
            if (cs.zo < 0)
                disp[3] = 16;
        }
        delay(4000);
    } // while end
}
