#include "../Common/Include/stm32l051xx.h"

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0  ** TIE TO GND **
//      PC15 -|3       30|- PB7  (I2C1_SDA)
//      NRST -|4       29|- PB6  (I2C1_SCL)
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4    (DFPlayer BUSY)
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15
//       PA3 -|9       24|- PA14
//       PA4 -|10      23|- PA13
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (RXD, USART1 / DFPlayer TX)
//       PB0 -|14      19|- PA9  (TXD, USART1 / DFPlayer RX)
//       PB1 -|15      18|- PA8  (LED+1k)
//       VSS -|16      17|- VDD
//             ----------
//
// USART1 (PA9=TX, PA10=RX): shared debug UART + DFPlayer Mini (9600 baud)
//   DFPlayer RX  <- PA9  (receives commands)
//   DFPlayer TX  -> PA10 (not used)
//   DFPlayer BUSY -> PA0 (pin 6, active low while playing)
//
// USART2 (Bluetooth HC-05): PA2=TX (pin 8), PA3=RX (pin 9)
//
// HW-290 (10DOF) I2C addresses:
//   MPU6050  = 0x68  (accel + gyro)
//   HMC5883L = 0x1E  (magnetometer, behind MPU6050 bypass)
//   BMP180   = 0x77  (barometer + temperature, behind MPU6050 bypass)
//
// Wiring: VCC_IN -> VDDA, GND -> GND, SCL -> PB6, SDA -> PB7, FSYNC -> GND
//
// HC-05 Bluetooth:
//   HC-05 TXD -> PA3 (USART2_RX)
//   HC-05 RXD -> PA2 (USART2_TX)
//   HC-05 VCC -> 3.3V or 5V, GND -> GND

// ---- I2C device addresses ----
#define MPU6050_ADDR     0x68
#define HMC5883L_ADDR    0x1E
#define QMC5883L_ADDR    0x0D
#define BMP180_ADDR      0x77

// ---- MPU6050 registers ----
#define MPU_WHO_AM_I     0x75
#define MPU_PWR_MGMT_1   0x6B
#define MPU_USER_CTRL    0x6A
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_INT_PIN_CFG  0x37
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H  0x43

// ---- HMC5883L registers ----
#define HMC_CONFIG_A     0x00
#define HMC_CONFIG_B     0x01
#define HMC_MODE         0x02
#define HMC_DATA_X_H     0x03
#define HMC_ID_A         0x0A

// ---- QMC5883L registers (clone, different addr + register layout) ----
#define QMC_DATA_X_L     0x00
#define QMC_STATUS       0x06
#define QMC_CTRL1        0x09
#define QMC_CTRL2        0x0A
#define QMC_SET_RESET    0x0B
#define QMC_CHIP_ID      0x0D

// ---- BMP180 registers ----
#define BMP_CHIP_ID      0xD0
#define BMP_CTRL_MEAS    0xF4
#define BMP_OUT_MSB      0xF6
#define BMP_CALIB_START  0xAA
#define BMP_CMD_TEMP     0x2E
#define BMP_CMD_PRES_0   0x34   // OSS=0 (low power)

#define SYSCLK_HZ        32000000UL   // startup.c initClock(): HSI16*4/2 = 32 MHz
#define UART_BAUD        9600

// ---- SysTick timer (Cortex-M0+ core peripheral, not defined in this header) ----
#define SYST_CSR   (*(volatile unsigned long *)0xE000E010)  // Control and Status
#define SYST_RVR   (*(volatile unsigned long *)0xE000E014)  // Reload Value
#define SYST_CVR   (*(volatile unsigned long *)0xE000E018)  // Current Value

volatile int x;

void delay(int dly)
{
    while (dly--) x++;
}

// ========================================================================
//  Division-free math (no libgcc needed on Cortex-M0+)
// ========================================================================

unsigned long udiv(unsigned long n, unsigned long d)
{
    if (d == 0) return 0;
    unsigned long q = 0;
    unsigned long r = 0;
    int i;
    for (i = 31; i >= 0; i--)
    {
        r = (r << 1) | ((n >> i) & 1);
        if (r >= d) { r -= d; q |= (1UL << i); }
    }
    return q;
}

unsigned long umod(unsigned long n, unsigned long d)
{
    return n - udiv(n, d) * d;
}

long sdiv(long n, long d)
{
    int neg = (n < 0) ^ (d < 0);
    unsigned long q = udiv(n < 0 ? (unsigned long)-n : (unsigned long)n,
                           d < 0 ? (unsigned long)-d : (unsigned long)d);
    return neg ? -(long)q : (long)q;
}

long smod(long n, long d)
{
    return n - sdiv(n, d) * d;
}

unsigned long isqrt(unsigned long n)
{
    unsigned long res = 0;
    unsigned long bit = 1UL << 30;
    while (bit > n) bit >>= 2;
    while (bit != 0)
    {
        if (n >= res + bit) { n -= res + bit; res = (res >> 1) + bit; }
        else                { res >>= 1; }
        bit >>= 2;
    }
    return res;
}

// atan2 approximation: returns degrees * 10
long iatan2_deg10(long y, long x)
{
    long ax = x < 0 ? -x : x;
    long ay = y < 0 ? -y : y;
    long angle;

    if (ax == 0 && ay == 0) return 0;

    if (ax >= ay)
        angle = (long)udiv((unsigned long)(ay * 573), (unsigned long)ax);
    else
        angle = 900 - (long)udiv((unsigned long)(ax * 573), (unsigned long)ay);

    if (x < 0) angle = 1800 - angle;
    if (y < 0) angle = -angle;

    return angle;
}


// ========================================================================
//  UART  PA9=TX  PA10=RX  AF4  (USART1)
//  NOTE: PA9/PA10 AF4 = USART1, NOT USART2.
//        USART2 TX/RX would be PA2/PA3.
// ========================================================================

void uart_init(void)
{
    RCC->IOPENR  |= BIT0;       // GPIOA clock
    RCC->APB2ENR |= BIT14;      // USART1 clock (APB2, bit 14)

    // PA9 -> AF4 (TX)
    GPIOA->MODER  = (GPIOA->MODER & ~(BIT19|BIT18)) | BIT19;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF << 4)) | (4 << 4);

    // PA10 -> AF4 (RX)
    GPIOA->MODER  = (GPIOA->MODER & ~(BIT21|BIT20)) | BIT21;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF << 8)) | (4 << 8);

    USART1->BRR = SYSCLK_HZ / UART_BAUD;   // 32000000/9600 = 3333
    USART1->CR1 = BIT3 | BIT0;              // TE | UE
}

void uart_putc(char c)
{
    while (!(USART1->ISR & BIT7));
    USART1->TDR = c;
}

void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

void uart_putu(unsigned long v)
{
    char buf[12];
    int i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v)
    {
        unsigned long q = udiv(v, 10);
        buf[i++] = '0' + (int)(v - q * 10);
        v = q;
    }
    while (i--) uart_putc(buf[i]);
}

void uart_puti(long v)
{
    if (v < 0) { uart_putc('-'); v = -v; }
    uart_putu((unsigned long)v);
}

// Print hex byte for debugging
void uart_puthex8(unsigned char b)
{
    const char hex[] = "0123456789ABCDEF";
    uart_putc('0'); uart_putc('x');
    uart_putc(hex[(b >> 4) & 0x0F]);
    uart_putc(hex[b & 0x0F]);
}

void uart_put_deg10(long d)
{
    if (d < 0) { uart_putc('-'); d = -d; }
    unsigned long q = udiv((unsigned long)d, 10);
    unsigned long r = (unsigned long)d - q * 10;
    uart_putu(q);
    uart_putc('.');
    uart_putc('0' + (int)r);
}

// ========================================================================
//  Bluetooth (HC-05)  PA2=TX  PA3=RX  AF4  (USART2)
//  Default HC-05 baud: 9600
// ========================================================================

void bt_init(void)
{
    RCC->IOPENR  |= BIT0;       // GPIOA clock
    RCC->APB1ENR |= BIT17;      // USART2 clock (APB1, bit 17)

    // PA2 -> AF4 (USART2_TX)
    GPIOA->MODER  = (GPIOA->MODER & ~(BIT5|BIT4)) | BIT5;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 8)) | (4 << 8);

    // PA3 -> AF4 (USART2_RX)
    GPIOA->MODER  = (GPIOA->MODER & ~(BIT7|BIT6)) | BIT7;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 12)) | (4 << 12);

    USART2->BRR = SYSCLK_HZ / UART_BAUD;   // 32000000/9600 = 3333
    USART2->CR1 = BIT3 | BIT2 | BIT0;       // TE | RE | UE
}

void bt_putc(char c)
{
    while (!(USART2->ISR & BIT7));
    USART2->TDR = c;
}

void bt_puts(const char *s)
{
    while (*s) bt_putc(*s++);
}

void bt_putu(unsigned long v)
{
    char buf[12];
    int i = 0;
    if (v == 0) { bt_putc('0'); return; }
    while (v)
    {
        unsigned long q = udiv(v, 10);
        buf[i++] = '0' + (int)(v - q * 10);
        v = q;
    }
    while (i--) bt_putc(buf[i]);
}

void bt_puti(long v)
{
    if (v < 0) { bt_putc('-'); v = -v; }
    bt_putu((unsigned long)v);
}

void bt_put_deg10(long d)
{
    if (d < 0) { bt_putc('-'); d = -d; }
    unsigned long q = udiv((unsigned long)d, 10);
    unsigned long r = (unsigned long)d - q * 10;
    bt_putu(q);
    bt_putc('.');
    bt_putc('0' + (int)r);
}

// Check if a byte has been received on BT
int bt_available(void)
{
    return (USART2->ISR & BIT5) ? 1 : 0;   // RXNE flag
}

// Read one byte from BT (call only when bt_available() returns 1)
char bt_getc(void)
{
    return (char)USART2->RDR;
}

// ========================================================================
//  I2C1  PB6=SCL  PB7=SDA  AF1
// ========================================================================

void i2c_init(void)
{
    RCC->IOPENR  |= BIT1;       // GPIOB clock
    RCC->APB1ENR |= BIT21;      // I2C1 clock

    // PB6, PB7 -> alternate function mode
    GPIOB->MODER = (GPIOB->MODER
                    & ~(BIT13|BIT12 | BIT15|BIT14))
                    |  (BIT13       | BIT15);

    GPIOB->OTYPER  |= BIT6 | BIT7;                    // open-drain
    GPIOB->OSPEEDR |= (BIT13|BIT12 | BIT15|BIT14);    // high speed
    GPIOB->PUPDR    = (GPIOB->PUPDR
                       & ~(BIT13|BIT12 | BIT15|BIT14))
                       |  (BIT12       | BIT14);       // pull-up

    // PB6=AF1 bits[27:24], PB7=AF1 bits[31:28]
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFF << 24)) | (0x11 << 24);

    // ~100kHz @ 32MHz PLL (PRESC=7 -> t_PRESC=250ns, SCLH=16*250=4us, SCLL=20*250=5us)
    I2C1->TIMINGR = (7 << 28) | (2 << 20) | (0 << 16) | (0x0F << 8) | (0x13);
    I2C1->CR1 = BIT0;           // PE (peripheral enable)
}

// Returns 0 on success, 1 on NACK/timeout
// CR2 bits: BIT25=AUTOEND, BIT13=START, BIT10=RD_WRN, [23:16]=NBYTES, [9:1]=SADD
int i2c_write(unsigned char addr, unsigned char *data, int len)
{
    I2C1->CR2 = (len << 16) | (addr << 1) | BIT25 | BIT13;  // AUTOEND | START
    for (int i = 0; i < len; i++)
    {
        int t = 100000;
        while (!(I2C1->ISR & BIT1) && --t);   // wait TXIS
        if (!t || (I2C1->ISR & BIT4))          // timeout or NACKF
        {
            I2C1->ICR |= BIT4;  // clear NACK flag
            // wait for STOP to complete (AUTOEND generates STOP on NACK)
            t = 100000;
            while (!(I2C1->ISR & BIT5) && --t);
            I2C1->ICR |= BIT5;
            return 1;
        }
        I2C1->TXDR = data[i];
    }
    int t = 100000;
    while (!(I2C1->ISR & BIT5) && --t);       // wait STOPF
    I2C1->ICR |= BIT5;
    return 0;
}

int i2c_read(unsigned char addr, unsigned char *buf, int len)
{
    I2C1->CR2 = (len << 16) | (addr << 1) | BIT25 | BIT13 | BIT10;  // AUTOEND | START | RD_WRN
    for (int i = 0; i < len; i++)
    {
        int t = 100000;
        while (!(I2C1->ISR & BIT2) && --t);   // wait RXNE
        if (!t) return 1;
        buf[i] = I2C1->RXDR;
    }
    int t = 100000;
    while (!(I2C1->ISR & BIT5) && --t);       // wait STOPF
    I2C1->ICR |= BIT5;
    return 0;
}

// Generic register write/read for any I2C address
int i2c_write_reg(unsigned char addr, unsigned char reg, unsigned char val)
{
    unsigned char buf[2] = {reg, val};
    return i2c_write(addr, buf, 2);
}

int i2c_read_reg(unsigned char addr, unsigned char reg, unsigned char *buf, int len)
{
    if (i2c_write(addr, &reg, 1)) return 1;
    return i2c_read(addr, buf, len);
}

// ========================================================================
//  I2C bus scan - detect which devices respond
// ========================================================================

void i2c_scan(void)
{
    uart_puts("I2C scan: ");
    int found = 0;
    unsigned char addr;
    for (addr = 0x08; addr < 0x78; addr++)
    {
        unsigned char dummy;
        if (i2c_read_reg(addr, 0x00, &dummy, 1) == 0)
        {
            uart_puthex8(addr);
            uart_putc(' ');
            found++;
        }
    }
    if (!found) uart_puts("no devices found!");
    uart_puts("\r\n");
}

// ========================================================================
//  MPU6050 - accelerometer + gyroscope
// ========================================================================

int mpu_ok = 0;   // flag: 1 if MPU6050 detected

void mpu_init(void)
{
    unsigned char id = 0;

    delay(100000);

    // Check WHO_AM_I (should return 0x68)
    if (i2c_read_reg(MPU6050_ADDR, MPU_WHO_AM_I, &id, 1))
    {
        uart_puts("MPU6050: I2C read FAILED (NACK/timeout)\r\n");
        return;
    }
    uart_puts("MPU6050 WHO_AM_I = ");
    uart_puthex8(id);
    if (id == 0x68) uart_puts(" OK\r\n");
    else            uart_puts(" UNEXPECTED!\r\n");

    // Wake up (clear SLEEP bit)
    i2c_write_reg(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00);
    delay(10000);

    // Sample rate divider: 1kHz / (1+7) = 125 Hz
    i2c_write_reg(MPU6050_ADDR, MPU_SMPLRT_DIV, 0x07);

    // DLPF config: ~44 Hz bandwidth
    i2c_write_reg(MPU6050_ADDR, MPU_CONFIG, 0x03);

    // Gyro: +/- 250 deg/s (131 LSB/deg/s)
    i2c_write_reg(MPU6050_ADDR, MPU_GYRO_CONFIG, 0x00);

    // Accel: +/- 2g (16384 LSB/g)
    i2c_write_reg(MPU6050_ADDR, MPU_ACCEL_CONFIG, 0x00);

    // Enable I2C bypass so STM32 can directly reach HMC5883L/QMC5883L and BMP180
    // Step 1: Disable MPU's internal I2C master (USER_CTRL bit 5 = 0)
    i2c_write_reg(MPU6050_ADDR, MPU_USER_CTRL, 0x00);
    delay(5000);
    // Step 2: Enable bypass mode (INT_PIN_CFG bit 1 = BYPASS_EN)
    i2c_write_reg(MPU6050_ADDR, MPU_INT_PIN_CFG, 0x02);
    delay(10000);

    mpu_ok = 1;
    uart_puts("MPU6050 init done (bypass enabled)\r\n");
}

void mpu_read_accel(int *ax, int *ay, int *az)
{
    unsigned char buf[6];
    i2c_read_reg(MPU6050_ADDR, MPU_ACCEL_XOUT_H, buf, 6);
    *ax = (int)((buf[0] << 8) | buf[1]);
    *ay = (int)((buf[2] << 8) | buf[3]);
    *az = (int)((buf[4] << 8) | buf[5]);
    if (*ax > 32767) *ax -= 65536;
    if (*ay > 32767) *ay -= 65536;
    if (*az > 32767) *az -= 65536;
}

void mpu_read_gyro(int *gx, int *gy, int *gz)
{
    unsigned char buf[6];
    i2c_read_reg(MPU6050_ADDR, MPU_GYRO_XOUT_H, buf, 6);
    *gx = (int)((buf[0] << 8) | buf[1]);
    *gy = (int)((buf[2] << 8) | buf[3]);
    *gz = (int)((buf[4] << 8) | buf[5]);
    if (*gx > 32767) *gx -= 65536;
    if (*gy > 32767) *gy -= 65536;
    if (*gz > 32767) *gz -= 65536;
}

// ========================================================================
//  Magnetometer - tries HMC5883L first, then QMC5883L (common clone)
//  Both accessed via MPU6050 I2C bypass
// ========================================================================

int mag_ok = 0;    // flag: 1 = HMC5883L, 2 = QMC5883L
int mag_addr = 0;  // actual I2C address of detected magnetometer

void mag_init(void)
{
    unsigned char id[3];

    // ---- Try HMC5883L (0x1E) first ----
    if (i2c_read_reg(HMC5883L_ADDR, HMC_ID_A, id, 3) == 0)
    {
        uart_puts("HMC5883L ID = ");
        uart_putc(id[0]); uart_putc(id[1]); uart_putc(id[2]);
        uart_puts("\r\n");

        i2c_write_reg(HMC5883L_ADDR, HMC_CONFIG_A, 0x70);  // 8 avg, 15Hz
        i2c_write_reg(HMC5883L_ADDR, HMC_CONFIG_B, 0x20);  // 1.3 Ga
        i2c_write_reg(HMC5883L_ADDR, HMC_MODE, 0x00);       // continuous
        delay(10000);
        mag_ok = 1;
        mag_addr = HMC5883L_ADDR;
        uart_puts("HMC5883L init done\r\n");
        return;
    }

    // ---- Try QMC5883L (0x0D) ----
    unsigned char qid = 0;
    if (i2c_read_reg(QMC5883L_ADDR, QMC_CHIP_ID, &qid, 1) == 0)
    {
        uart_puts("QMC5883L chip ID = ");
        uart_puthex8(qid);
        uart_puts("\r\n");

        // SET/RESET period (recommended by datasheet)
        i2c_write_reg(QMC5883L_ADDR, QMC_SET_RESET, 0x01);
        // CTRL1: continuous mode, 200Hz ODR, 8G range, 512 oversampling
        // bits: OSR=00(512), RNG=01(8G), ODR=11(200Hz), MODE=01(continuous)
        i2c_write_reg(QMC5883L_ADDR, QMC_CTRL1, 0x0D);
        // CTRL2: pointer rollover enabled
        i2c_write_reg(QMC5883L_ADDR, QMC_CTRL2, 0x40);
        delay(10000);
        mag_ok = 2;
        mag_addr = QMC5883L_ADDR;
        uart_puts("QMC5883L init done\r\n");
        return;
    }

    uart_puts("Magnetometer: not found (HMC/QMC both failed)\r\n");
}

void mag_read(int *mx, int *my, int *mz)
{
    unsigned char buf[6];

    if (mag_ok == 1)
    {
        // HMC5883L: X_H, X_L, Z_H, Z_L, Y_H, Y_L  (MSB first, Z before Y!)
        i2c_read_reg(HMC5883L_ADDR, HMC_DATA_X_H, buf, 6);
        *mx = (int)((buf[0] << 8) | buf[1]);
        *mz = (int)((buf[2] << 8) | buf[3]);
        *my = (int)((buf[4] << 8) | buf[5]);
    }
    else
    {
        // QMC5883L: X_L, X_H, Y_L, Y_H, Z_L, Z_H  (LSB first!)
        i2c_read_reg(QMC5883L_ADDR, QMC_DATA_X_L, buf, 6);
        *mx = (int)(buf[0] | (buf[1] << 8));
        *my = (int)(buf[2] | (buf[3] << 8));
        *mz = (int)(buf[4] | (buf[5] << 8));
    }
    if (*mx > 32767) *mx -= 65536;
    if (*my > 32767) *my -= 65536;
    if (*mz > 32767) *mz -= 65536;
}

// ========================================================================
//  BMP180 - barometric pressure + temperature
// ========================================================================

int bmp_ok = 0;   // flag: 1 if BMP180 detected

// BMP180 calibration coefficients (read from EEPROM at init)
short bmp_ac1, bmp_ac2, bmp_ac3;
unsigned short bmp_ac4, bmp_ac5, bmp_ac6;
short bmp_b1, bmp_b2;
short bmp_mb, bmp_mc, bmp_md;

void bmp_init(void)
{
    unsigned char id = 0;

    // Check chip ID (should be 0x55)
    if (i2c_read_reg(BMP180_ADDR, BMP_CHIP_ID, &id, 1))
    {
        uart_puts("BMP180: I2C read FAILED\r\n");
        return;
    }
    uart_puts("BMP180 chip ID = ");
    uart_puthex8(id);
    if (id == 0x55) uart_puts(" OK\r\n");
    else            uart_puts(" UNEXPECTED!\r\n");

    // Read calibration data (22 bytes from 0xAA)
    unsigned char cal[22];
    if (i2c_read_reg(BMP180_ADDR, BMP_CALIB_START, cal, 22))
    {
        uart_puts("BMP180: calib read FAILED\r\n");
        return;
    }

    bmp_ac1 = (short)((cal[0]  << 8) | cal[1]);
    bmp_ac2 = (short)((cal[2]  << 8) | cal[3]);
    bmp_ac3 = (short)((cal[4]  << 8) | cal[5]);
    bmp_ac4 = (unsigned short)((cal[6]  << 8) | cal[7]);
    bmp_ac5 = (unsigned short)((cal[8]  << 8) | cal[9]);
    bmp_ac6 = (unsigned short)((cal[10] << 8) | cal[11]);
    bmp_b1  = (short)((cal[12] << 8) | cal[13]);
    bmp_b2  = (short)((cal[14] << 8) | cal[15]);
    bmp_mb  = (short)((cal[16] << 8) | cal[17]);
    bmp_mc  = (short)((cal[18] << 8) | cal[19]);
    bmp_md  = (short)((cal[20] << 8) | cal[21]);

    bmp_ok = 1;
    uart_puts("BMP180 init done\r\n");
}

// Read raw uncompensated temperature
long bmp_read_raw_temp(void)
{
    unsigned char buf[2];
    i2c_write_reg(BMP180_ADDR, BMP_CTRL_MEAS, BMP_CMD_TEMP);
    delay(10000);   // wait ~4.5 ms for conversion
    i2c_read_reg(BMP180_ADDR, BMP_OUT_MSB, buf, 2);
    return (long)((buf[0] << 8) | buf[1]);
}

// Read raw uncompensated pressure (OSS=0)
long bmp_read_raw_pres(void)
{
    unsigned char buf[3];
    i2c_write_reg(BMP180_ADDR, BMP_CTRL_MEAS, BMP_CMD_PRES_0);
    delay(10000);   // wait ~4.5 ms for OSS=0
    i2c_read_reg(BMP180_ADDR, BMP_OUT_MSB, buf, 3);
    return (long)(((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 8);  // OSS=0: shift by 8-OSS=8
}

// Compute true temperature in 0.1 deg C and pressure in Pa
// Uses BMP180 datasheet compensation formulas with integer math
void bmp_compute(long *temp_x10, long *pressure_pa)
{
    long ut = bmp_read_raw_temp();
    long up = bmp_read_raw_pres();

    // Temperature compensation
    long x1 = sdiv(((long)ut - (long)bmp_ac6) * (long)bmp_ac5, 32768);
    long x2 = sdiv((long)bmp_mc * 2048, x1 + (long)bmp_md);
    long b5 = x1 + x2;
    *temp_x10 = sdiv(b5 + 8, 16);   // temperature in 0.1 deg C

    // Pressure compensation (OSS = 0)
    long b6 = b5 - 4000;
    x1 = sdiv((long)bmp_b2 * sdiv(b6 * b6, 4096), 2048);
    x2 = sdiv((long)bmp_ac2 * b6, 2048);
    long x3 = x1 + x2;
    long b3 = sdiv(((long)bmp_ac1 * 4 + x3) + 2, 4);

    x1 = sdiv((long)bmp_ac3 * b6, 8192);
    x2 = sdiv((long)bmp_b1 * sdiv(b6 * b6, 4096), 65536);
    x3 = sdiv(x1 + x2 + 2, 4);
    unsigned long b4 = udiv((unsigned long)bmp_ac4 * (unsigned long)(x3 + 32768), 32768);
    unsigned long b7 = ((unsigned long)up - (unsigned long)b3) * 50000UL;

    long p;
    if (b7 < 0x80000000UL)
        p = (long)udiv(b7 * 2, b4);
    else
        p = (long)udiv(b7, b4) * 2;

    x1 = sdiv(p, 256);
    x1 = x1 * x1;
    x1 = sdiv(x1 * 3038, 65536);
    x2 = sdiv(-7357 * p, 65536);
    p = p + sdiv(x1 + x2 + 3791, 16);

    *pressure_pa = p;
}


// ========================================================================
//  Button on PA5 (input, pull-up, active low = pressed when GND)
// ========================================================================

void button_init(void)
{
    RCC->IOPENR |= BIT0;   // GPIOA clock (may already be on)
    GPIOA->MODER &= ~(BIT11 | BIT10);   // input mode
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(BIT11 | BIT10)) | BIT10;  // pull-up
}

int button_pressed(void)
{
    return !(GPIOA->IDR & BIT5);
}

// ========================================================================
//  DFPlayer Mini — uses USART1 (PA9 TX) shared with debug output
//  BUSY pin on PA0 (active low while playing)
// ========================================================================

void DFPlayer_SendCommand(unsigned char cmd, unsigned int param)
{
    unsigned char ph = (param >> 8) & 0xFF;
    unsigned char pl = param & 0xFF;
    unsigned int sum = 0xFF + 0x06 + cmd + 0x00 + ph + pl;
    unsigned int chk = (unsigned int)(0 - sum);

    unsigned char pkt[10];
    pkt[0] = 0x7E;
    pkt[1] = 0xFF;
    pkt[2] = 0x06;
    pkt[3] = cmd;
    pkt[4] = 0x00;
    pkt[5] = ph;
    pkt[6] = pl;
    pkt[7] = (chk >> 8) & 0xFF;
    pkt[8] = chk & 0xFF;
    pkt[9] = 0xEF;

    int i;
    for (i = 0; i < 10; i++)
        uart_putc(pkt[i]);          // reuse existing USART1 TX

    // Wait for last byte to finish transmitting
    while (!(USART1->ISR & BIT6));  // TC (transmit complete)
}

void DFPlayer_SetVolume(unsigned char vol)
{
    if (vol > 30) vol = 30;
    DFPlayer_SendCommand(0x06, vol);
}

void DFPlayer_PlayTrack(unsigned int track)
{
    DFPlayer_SendCommand(0x03, track);
}

int DFPlayer_IsBusy(void)
{
    return (GPIOA->IDR & BIT0) == 0;   // LOW = playing
}

void DFPlayer_WaitUntilFinished(void)
{
    unsigned long timeout;

    // Wait until playback starts (BUSY goes low), up to ~3 seconds
    timeout = 0;
    while (!DFPlayer_IsBusy() && timeout < 15000)
    {
        delay(6400);    // ~1 ms at 32 MHz
        timeout++;
    }

    // Wait until playback ends (BUSY goes high)
    while (DFPlayer_IsBusy())
    {
        delay(32000);   // ~5 ms at 32 MHz
    }
}

void DFPlayer_Init(void)
{
    // PA0 = input (BUSY), no pull-up/down
    RCC->IOPENR |= BIT0;                         // GPIOA clock
    GPIOA->MODER &= ~(BIT1 | BIT0);              // PA0 input mode
    GPIOA->PUPDR &= ~(BIT1 | BIT0);              // no pull

    // USART1 already initialised by uart_init() at 9600 baud
    // Let DFPlayer power up, then set volume
    delay(5000000);     // ~1 second
    DFPlayer_SetVolume(30);
    delay(1000000);     // ~200 ms
}

// ========================================================================
//  Main
// ========================================================================

void main(void)
{
    // 1. LED sanity check (PA8)
    RCC->IOPENR |= BIT0;
    GPIOA->MODER = (GPIOA->MODER & ~(BIT17|BIT16)) | BIT16;

    int b;
    for (b = 0; b < 6; b++)
    {
        GPIOA->ODR ^= BIT8;
        delay(200000);
    }

    // 2. UART init (debug + DFPlayer) + Bluetooth init
    uart_init();
    bt_init();
    DFPlayer_Init();    // PA0 BUSY input + set volume (shares USART1)
    uart_puts("Angle Reader\r\n");
    bt_puts("HC05 OK\r\n");

    // 3. Button init
    button_init();

    // 4. I2C + sensors init
    i2c_init();
    delay(50000);
    mpu_init();       // also enables I2C bypass for BMP180
    mag_init();       // tries HMC5883L then QMC5883L (not used for yaw here)
    bmp_init();

    if (!mpu_ok)
    {
        uart_puts("ERROR: MPU6050 not found!\r\n");
        while (1);  // halt
    }

    uart_puts("Showing: Pitch/Roll (accel) + Yaw (gyro) | Temp/Pres (BMP180)\r\n\r\n");

    // ---- SysTick free-running to measure actual dt ----
    SYST_RVR = 0x00FFFFFF;   // max reload
    SYST_CVR = 0;
    SYST_CSR = 5;            // enable, processor clock, no interrupt

    unsigned long tick_prev = SYST_CVR;
    long gyro_yaw_x10 = 0;
    long yaw_r = 0;           // relative yaw (tenths of degrees, 0-3600)
    int cycle_count = 0;      // counts loop iterations for yaw_r reset
    int btn_prev = 0;

    // Print four blank lines so cursor-up works from the start
    uart_puts("\r\n\r\n\r\n\r\n");

    // 4. Main loop
    while (1)
    {
        // ---- Measure actual elapsed time ----
        unsigned long tick_now = SYST_CVR;
        unsigned long dt_ticks = (tick_prev - tick_now) & 0x00FFFFFF;
        tick_prev = tick_now;
        unsigned long dt_ms = udiv(dt_ticks, 32000);
        if (dt_ms == 0) dt_ms = 1;

        // ---- Read accelerometer -> pitch & roll ----
        int ax, ay, az;
        mpu_read_accel(&ax, &ay, &az);

        long pitch = iatan2_deg10((long)ax,  (long)az);
        long roll  = iatan2_deg10((long)-ay, (long)az);

        // Convert from [-1800..+1800] to [0..3600) (0-360 deg, counter-clockwise)
        if (pitch < 0) pitch += 3600;
        if (roll  < 0) roll  += 3600;

        // ---- Read gyroscope -> integrate yaw ----
        int gx, gy, gz;
        mpu_read_gyro(&gx, &gy, &gz);

        // Sensitivity: 131 LSB/deg/s at +/-250 deg/s
        // yaw_change_deg10 = gz * dt_ms / 13100
        gyro_yaw_x10 += sdiv((long)gz * (long)dt_ms, 13100);
        while (gyro_yaw_x10 < 0)    gyro_yaw_x10 += 3600;
        while (gyro_yaw_x10 >= 3600) gyro_yaw_x10 -= 3600;

        // ---- Relative yaw: reset every 10 cycles (1000ms) ----
        cycle_count++;
        if (cycle_count >= 10)
        {
            // Play direction audio once at the end of each period
            if (yaw_r > 3500 || yaw_r < 100)
            {
                DFPlayer_PlayTrack(3);      // move forward
                DFPlayer_WaitUntilFinished();          
            }
            else if (yaw_r > 100 && yaw_r < 1800)
            {
                DFPlayer_PlayTrack(1);      // turn left
                DFPlayer_WaitUntilFinished();  
            }
            else if (yaw_r > 1800 && yaw_r < 3500)
            {
                DFPlayer_PlayTrack(2);      // turn right
                DFPlayer_WaitUntilFinished();          
            }

            yaw_r = 0;
            cycle_count = 0;

            // Reset SysTick baseline so dt is not corrupted by playback time
            tick_prev = SYST_CVR;
        }
        else
        {
            yaw_r += sdiv((long)gz * (long)dt_ms, 13100);
            while (yaw_r < 0)    yaw_r += 3600;
            while (yaw_r >= 3600) yaw_r -= 3600;
        }

        // ---- Button: reset gyro yaw on press ----
        int btn_now = button_pressed();
        if (btn_now && !btn_prev)
        {
            gyro_yaw_x10 = 0;
        }
        btn_prev = btn_now;

        // ---- Read BMP180 temperature & pressure ----
        long temp_x10 = 0, pres_pa = 0;
        if (bmp_ok) bmp_compute(&temp_x10, &pres_pa);

        // ---- Line 1: Pitch, Roll (accel) and Yaw (gyro) ----
        uart_puts("\033[4A\r");

        uart_puts("Pitch=");
        uart_put_deg10(pitch);
        uart_puts(" Roll=");
        uart_put_deg10(roll);
        uart_puts(" Yaw=");
        uart_put_deg10(gyro_yaw_x10);
        uart_puts("       \r\n");

        // ---- Line 2: Raw accel and gyro ----
        uart_puts("AX="); uart_puti(ax);
        uart_puts(" AY="); uart_puti(ay);
        uart_puts(" AZ="); uart_puti(az);
        uart_puts(" GX="); uart_puti(gx);
        uart_puts(" GY="); uart_puti(gy);
        uart_puts(" GZ="); uart_puti(gz);
        uart_puts("       \r\n");

        // ---- Line 3: Temperature and Pressure (BMP180) ----
        if (bmp_ok)
        {
            uart_puts("Temp=");
            uart_put_deg10(temp_x10);
            uart_puts("C Pres=");
            uart_putu((unsigned long)pres_pa);
            uart_puts("Pa      ");
        }
        else
        {
            uart_puts("BMP180 not detected          ");
        }
        uart_puts("\r\n");

        // ---- Line 4: Relative yaw and direction ----
        uart_puts("Yaw_r=");
        uart_put_deg10(yaw_r);
        if (yaw_r > 3500 || yaw_r < 100)
            uart_puts(" move forward    ");
        else if (yaw_r > 100 && yaw_r < 1800)
            uart_puts(" turn left       ");
        else if (yaw_r > 1800 && yaw_r < 3500)
            uart_puts(" turn right      ");
        uart_puts("\r\n");

        // ---- Mirror full display to Bluetooth ----
        bt_puts("\033[4A\r");

        bt_puts("Pitch=");
        bt_put_deg10(pitch);
        bt_puts(" Roll=");
        bt_put_deg10(roll);
        bt_puts(" Yaw=");
        bt_put_deg10(gyro_yaw_x10);
        bt_puts("       \r\n");

        bt_puts("AX="); bt_puti(ax);
        bt_puts(" AY="); bt_puti(ay);
        bt_puts(" AZ="); bt_puti(az);
        bt_puts(" GX="); bt_puti(gx);
        bt_puts(" GY="); bt_puti(gy);
        bt_puts(" GZ="); bt_puti(gz);
        bt_puts("       \r\n");

        if (bmp_ok)
        {
            bt_puts("Temp=");
            bt_put_deg10(temp_x10);
            bt_puts("C Pres=");
            bt_putu((unsigned long)pres_pa);
            bt_puts("Pa      ");
        }
        else
        {
            bt_puts("BMP180 not detected          ");
        }
        /*bt_puts("\r\n");

        bt_puts("Yaw_r=");
        bt_put_deg10(yaw_r);
        if (yaw_r > 3500 || yaw_r < 100)
            bt_puts(" move forward    ");
        else if (yaw_r > 100 && yaw_r < 1800)
            bt_puts(" turn left       ");
        else if (yaw_r > 1800 && yaw_r < 3500)
            bt_puts(" turn right      ");
        bt_puts("\r\n");*/
    }
}