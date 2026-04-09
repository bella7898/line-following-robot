// ADC.c:  Shows how to use the 14-bit ADC.  This program
// measures the voltage from some pins of the EFM8LB1 using the ADC.
//
// (c) 2008-2018, Jesus Calvino-Fraga
//

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include "vl53l0x.h"
#include "vl53l0x.c"

// ~C51~  

#define SYSCLK 72000000L
#define BAUDRATE0 115200L
#define UART1_BAUD 600L
#define SARCLK 18000000L
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L)))
#define  SMB_FREQUENCY  400000L   // I2C SCL clock rate


/*
 * EFM8LB1 pin use
 * P2.5 : left H-bridge input A
 * P2.6 : left H-bridge input B
 * P3.0 : right H-bridge input A
 * P3.1 : right H-bridge input B
 */

#define LEFT_BRIDGE_A  P2_5
#define LEFT_BRIDGE_B  P2_6
#define RIGHT_BRIDGE_A P3_0
#define RIGHT_BRIDGE_B P3_1
#define LED P1_6

 // key1, key2, [data to store/retrive to/from flash] must be consecutive]
unsigned char key1, key2;
volatile int offset;


#define CONST_SIZE 4

#define SaveFdata(X,Y) \
{ 	FLKEY  = 0xA5; \
	FLKEY  = 0xF1; \
	PSCTL  = 0x01; \
	*((unsigned char xdata *) X)=Y; \
	PSCTL  = 0x00;}

#define EraseFdataPage(X) \
{	FLKEY  = 0xA5; \
	FLKEY  = 0xF1; \
	PSCTL  = 0x03; \
	*((unsigned char xdata *) X)=0; \
	PSCTL  = 0x00;	}

#define ReadFdata(X) (*((unsigned char code *) X))

// Each page in the EFM8LB1 is 512 bytes.  This location is only good for 64k devices.
#define BASE_FDATA 0xf800

void Save_Vars(void)
{
	bit saved_EA;
	unsigned int j;
	unsigned int address;
	unsigned char* ptr;

	saved_EA = EA;
	EA = 0; // No interrupts while erasing/writing FLASH memory
	EraseFdataPage(BASE_FDATA);

	key1 = 0x55;
	key2 = 0xaa;
	address = BASE_FDATA;
	ptr = &key1;

	for (j = 0; j < CONST_SIZE; j++)
	{
		SaveFdata(address++, *ptr);
		ptr++;
	}

	EA = saved_EA;
}

void Restore_Vars(void)
{
	unsigned int j;
	unsigned int address;
	unsigned char* ptr;

	if ((ReadFdata(BASE_FDATA) != 0x55) || (ReadFdata(BASE_FDATA + 1) != 0xaa))
	{
		offset = 0;
	}
	else
	{
		address = BASE_FDATA;
		ptr = &key1;
		for (j = 0; j < CONST_SIZE; j++)
		{
			*ptr = ReadFdata(address++);
			ptr++;
		}
	}
}

void Wait_SI(void)
{
	unsigned int I2C_t = 5000;
	while ((!SI) && (I2C_t > 0)) I2C_t--;
}

void Wait_STO(void)
{
	unsigned int I2C_t = 5000;
	while ((STO) && (I2C_t > 0)) I2C_t--;
}

void I2C_write(unsigned char output_data)
{
	SMB0DAT = output_data; // Put data into buffer
	SI = 0;  // Proceed with write
	Wait_SI(); // Wait until done with send
}

unsigned char I2C_read(bit ack)
{
	ACK = ack;
	SI = 0; // Proceed with read
	Wait_SI(); // Wait until done with read
	return SMB0DAT;
}

void I2C_start(void)
{
	ACK = 0;
	STO = 0;
	STA = 1; // Send I2C start
	SI = 0; // Proceed with start
	Wait_SI(); // Wait until done with start
}

void I2C_stop(void)
{
	ACK = 0;
	STA = 0;
	STO = 1; // Perform I2C stop
	SI = 0; // Proceed with stop
	Wait_STO(); // Wait until done with stop
	STO = 0;
}

bit i2c_read_addr8_data8(unsigned char address, unsigned char* value)
{
	I2C_start();
	I2C_write(0x52); // Write address
	I2C_write(address);
	I2C_stop();

	I2C_start();
	I2C_write(0x53); // Read address
	*value = I2C_read(1);
	I2C_stop();

	return 1;
}

bit i2c_read_addr8_data16(unsigned char address, unsigned int* value)
{
	I2C_start();
	I2C_write(0x52); // Write address
	I2C_write(address);
	I2C_stop();

	I2C_start();
	I2C_write(0x53); // Read address
	*value = I2C_read(0) * 256;
	*value += I2C_read(1);
	I2C_stop();

	return 1;
}

bit i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
	I2C_start();
	I2C_write(0x52); // Write address
	I2C_write(address);
	I2C_write(value);
	I2C_stop();

	return 1;
}

// From the VL53L0X datasheet:
//
// The registers shown in the table below can be used to validate the user I2C interface.
// Address (After fresh reset, without API loaded)
//    0xC0 0xEE
//    0xC1 0xAA
//    0xC2 0x10
//    0x51 0x0099
//    0x61 0x0000
//
// Not needed, but it was useful to debug the I2C interface, so left here.
void validate_I2C_interface(void)
{
	unsigned char val8 = 0;
	unsigned int val16 = 0;

	printf("\n");

	i2c_read_addr8_data8(0xc0, &val8);
	printf("Reg(0xc0): 0x%02x\n", val8);

	i2c_read_addr8_data8(0xc1, &val8);
	printf("Reg(0xc1): 0x%02x\n", val8);

	i2c_read_addr8_data8(0xc2, &val8);
	printf("Reg(0xc2): 0x%02x\n", val8);

	i2c_read_addr8_data16(0x51, &val16);
	printf("Reg(0x51): 0x%04x\n", val16);

	i2c_read_addr8_data16(0x61, &val16);
	printf("Reg(0x61): 0x%04x\n", val16);

	printf("\n");
}

 /* PWM globals (used by Timer5 ISR) */
xdata unsigned int pwm_counter = 0;
xdata unsigned char pwm_left = 0, pwm_right = 0;
xdata unsigned char left_motor_dir = 0, right_motor_dir = 0; /* 0=forward, 1=backward */

/* Forward declarations */
void Set_Pin_Output(unsigned char pin);
void motors_stop(void);
void waitms(unsigned int ms);
void TURN_LEFT(unsigned char speed);
void TURN_RIGHT(unsigned char speed);
void TURN_FORWARD(unsigned char speed);
void TURN_BACKWARD(unsigned char speed);


char _c51_external_startup(void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key

	VDM0CN = 0x80;       // enable VDD monitor
	RSTSRC = 0x02 | 0x04;  // Enable reset on missing clock detector and VDD

#if (SYSCLK == 48000000L)	
	SFRPAGE = 0x10;
	PFE0CN = 0x10; // SYSCLK < 50 MHz.
	SFRPAGE = 0x00;
#elif (SYSCLK == 72000000L)
	SFRPAGE = 0x10;
	PFE0CN = 0x20; // SYSCLK < 75 MHz.
	SFRPAGE = 0x00;
#endif

#if (SYSCLK == 12250000L)
	CLKSEL = 0x10;
	CLKSEL = 0x10;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 24500000L)
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 48000000L)	
	// Before setting clock to 48 MHz, must transition to 24.5 MHz first
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
	CLKSEL = 0x07;
	CLKSEL = 0x07;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 72000000L)
	// Before setting clock to 72 MHz, must transition to 24.5 MHz first
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
	CLKSEL = 0x03;
	CLKSEL = 0x03;
	while ((CLKSEL & 0x80) == 0);
#else
#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
#endif
	SFRPAGE = 0x20;
	//P0SKIP |= 0x03;  //Skips P0.0 and P0.1
	SFRPAGE = 0x00;

	// Configure Uart 0
	SCON0 = 0x10;
	TH1 = 0x100 - ((SYSCLK / BAUDRATE0) / (12L * 2L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |= 0x20;
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	P0MDOUT |= 0x10;  // P0.4 TX0 push-pull (unchanged)
	XBR0 = 0b_0000_0101;
	XBR1 = 0x00;
	XBR2 = 0x41;

	// Configure Timer 0 as the I2C clock source
	CKCON0 |= 0b_0000_0100; // Timer0 clock source = SYSCLK
	TMOD &= 0xf0;  // Mask out timer 1 bits
	TMOD |= 0x02;  // Timer0 in 8-bit auto-reload mode
	// Timer 0 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
	TL0 = TH0 = 256 - (SYSCLK / SMB_FREQUENCY / 3);
	TR0 = 1; // Enable timer 0

	// Configure and enable SMBus
	SMB0CF = 0b_0101_1100; //INH | EXTHOLD | SMBTOE | SMBFTE ;
	//SMB0CF = 0b_0100_0100; //INH | EXTHOLD | SMBTOE | SMBFTE ;
	SMB0CF |= 0b_1000_0000;  // Enable SMBus

	// Configure Uart 0
#if (((SYSCLK/BAUDRATE0)/(2L*12L))>0xFFL)
#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE0)/(2L*12L) > 0xFF
#endif

	SCON0 = 0x10;

	TH1 = 0x100 - ((SYSCLK / BAUDRATE0) / (2L * 12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |= 0x20;
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	/* Timer5 for 10us PWM tick */

	SFRPAGE = 0x10;
	TMR5CN0 = 0x00;
	TMR5 = 0xFFFF;
	EIE2 |= 0x08;
	TR5 = 1;
	EA = 1;
	SFRPAGE = 0x00;

	//tof stuff
	SFRPAGE = 0x10;
	TMR3CN1 |= 0b_0110_0000; // Timer 3 will only reload on overflow events
	SFRPAGE = 0x00;

	return 0;
}

void UART1_Init(unsigned long baudrate)
{
	unsigned int reload;

	reload = 0x10000L - (SYSCLK / (2L * 12L * baudrate));

	SFRPAGE = 0x20;

	SMOD1 = 0x0C;
	SCON1 = 0x10;
	SBCON1 = 0x00;

	SBRLH1 = (reload >> 8) & 0xFF;
	SBRLL1 = reload & 0xFF;

	RI1 = 0;
	TI1 = 1;

	SBCON1 |= 0x40;   // enable baud generator

	SFRPAGE = 0x00;
}

//------------------------------------------------
// UART1 RX helpers
//------------------------------------------------
bit UART1_Available(void)
{
	bit b;
	SFRPAGE = 0x20;
	b = RI1;
	SFRPAGE = 0x00;
	return b;
}

char UART1_GetChar(void)
{
	char c;

	SFRPAGE = 0x20;
	while (!RI1);
	c = SBUF1;
	RI1 = 0;
	SCON1 &= 0x3F;
	SFRPAGE = 0x00;

	return c;
}

void putchar(char c)
{
	while (!TI);
	TI = 0;
	SBUF = c;

	if (c == '\n')
	{
		while (!TI);
		TI = 0;
		SBUF = '\r';
	}
}

/* ---------- Motor helpers ---------- */
void motors_stop(void)
{
	pwm_left = 0;
	pwm_right = 0;
	LEFT_BRIDGE_A = 0;
	LEFT_BRIDGE_B = 0;
	RIGHT_BRIDGE_A = 0;
	RIGHT_BRIDGE_B = 0;
}

void TURN_LEFT(unsigned char speed)
{
	left_motor_dir = 0;  /* LEFT:  A=0, B=1 (reverse) */
	right_motor_dir = 1;  /* RIGHT: A=1, B=0 (forward) */
	pwm_right = speed;
	pwm_left = speed;
}

void TURN_RIGHT(unsigned char speed)
{
	left_motor_dir = 1;  /* LEFT:  A=1, B=0 (forward) */
	right_motor_dir = 0;  /* RIGHT: A=0, B=1 (reverse) */
	pwm_right = speed;
	pwm_left = speed;
}

void TURN_FORWARD(unsigned char speed)
{
	left_motor_dir = 1;  /* LEFT:  A=1, B=0 (forward) */
	right_motor_dir = 1;  /* RIGHT: A=1, B=0 (forward) */
	pwm_right = speed - 3;
	pwm_left = speed;
}

void TURN_BACKWARD(unsigned char speed)
{
	left_motor_dir = 0;  /* LEFT:  A=0, B=1 (reverse) */
	right_motor_dir = 0;  /* RIGHT: A=0, B=1 (reverse) */
	pwm_right = speed;
	pwm_left = speed;
}

/* ---------- Timer5 ISR: PWM for both H-bridges ---------- */
void Timer5_ISR(void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE = 0x10;
	TF5H = 0;
	TMR5RL = RELOAD_10us;

	pwm_counter++;
	if (pwm_counter == 100)
		pwm_counter = 0;

	/* Left H-bridge */
	if (pwm_right > pwm_counter)
	{
		if (left_motor_dir)
		{
			LEFT_BRIDGE_A = 1;
			LEFT_BRIDGE_B = 0;
		}
		else
		{
			LEFT_BRIDGE_A = 0;
			LEFT_BRIDGE_B = 1;
		}
	}
	else
	{
		LEFT_BRIDGE_A = 0;
		LEFT_BRIDGE_B = 0;
	}

	/* Right H-bridge */
	if (pwm_left > pwm_counter)
	{
		if (right_motor_dir) { RIGHT_BRIDGE_A = 1; RIGHT_BRIDGE_B = 0; }
		else { RIGHT_BRIDGE_A = 0; RIGHT_BRIDGE_B = 1; }
	}
	else
	{
		RIGHT_BRIDGE_A = 0;
		RIGHT_BRIDGE_B = 0;
	}

	SFRPAGE = 0x00;
}

/* ---------- Pin utility ---------- */
void Set_Pin_Output(unsigned char pin)
{
	unsigned char mask = (1 << (pin & 0x7));
	switch (pin / 0x10)
	{
	case 0: P0MDOUT |= mask; break;
	case 1: P1MDOUT |= mask; break;
	case 2: P2MDOUT |= mask; break;
	case 3: P3MDOUT |= mask; break;
	}
}

/* ---------- Timing ---------- */
void Timer3us(unsigned char us)
{
	unsigned char i;
	CKCON0 |= 0x40;
	TMR3RL = (-(SYSCLK) / 1000000L);
	TMR3 = TMR3RL;
	TMR3CN0 = 0x04;
	for (i = 0; i < us; i++)
	{
		while (!(TMR3CN0 & 0x80));
		TMR3CN0 &= ~(0x80);
	}
	TMR3CN0 = 0;
}

void InitADC(void)
{
	SFRPAGE = 0x00;
	ADEN = 0; // Disable ADC

	ADC0CN1 =
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
		(0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0); // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32

	ADC0CF0 =
		((SYSCLK / SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.

	ADC0CF1 =
		(0 << 7) | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)

	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0); // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2 =
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)

	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0); // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN = 1; // Enable ADC
}

void waitms(unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for (j = 0; j < ms; j++)
		for (k = 0; k < 4; k++) Timer3us(250);
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC(unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;

	mask = 1 << pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
	case 0:
		P0MDIN &= (~mask); // Set pin as analog input
		P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	case 1:
		P1MDIN &= (~mask); // Set pin as analog input
		P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	case 2:
		P2MDIN &= (~mask); // Set pin as analog input
		P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	return ((ADC_at_Pin(pin) * VDD) / 16383.0);
}

void set_direction(char steps[], char array[]) {
	int i = 0;
	for (i = 0; i < 8; i++) {
		array[i] = steps[i];
	}
}

void main(void)
{
	float middle;
	float right;
	float left;
	char ser_in = 'K';
	unsigned char success;
	int range = 0;
	int distance = 1000;
	int pwm = 20;

	int flag = 0;
	int index = 0;
	int mode = 3; // 0 for manual, 1 for automatic

	const char directions1[] = { 0, 2, 2, 0, 1, 2, 1, 3 };
	const char directions2[] = { 2, 1, 2, 1, 0, 0, 3, 3 };
	const char directions3[] = { 1, 0, 1, 2, 1, 2, 0, 3 };

	char array[8] = { 0, 2, 2, 0, 1, 2, 1, 3 };

	Set_Pin_Output(0x25);  // P2.5 = LEFT_BRIDGE_A  
	Set_Pin_Output(0x26);  // P2.6 = LEFT_BRIDGE_B  
	Set_Pin_Output(0x30);  // P3.0 = RIGHT_BRIDGE_A 
	Set_Pin_Output(0x31);  // P3.1 = RIGHT_BRIDGE_B 
	Set_Pin_Output(0x16); //LED

	waitms(500); // Give PuTTy a chance to start before sending
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.

	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitPinADC(2, 3); // Configure P2.3 as analog input
	InitPinADC(1, 7); // Configure P2.4 as analog input
	
	InitPinADC(1, 3); // Configure P2.4 as analog input
	InitPinADC(1, 4); // Configure P2.4 as analog input
	InitPinADC(1, 5); // Configure P2.4 as analog input
	InitADC();

	motors_stop();
	
	Restore_Vars(); // Some vars stored in flash
	success = vl53l0x_init();
	if (success)
	{
		printf("VL53L0x initialization succeeded.\n");
	}
	else
	{
		printf("VL53L0x initialization failed.\n");
	}

	UART1_Init(UART1_BAUD);
	
	while (1)
	{
		
		if (distance < 15) {
			motors_stop();
			waitms(500);
			TURN_BACKWARD(50);
			waitms(750);
			motors_stop();
		}
		else {
			if (UART1_Available()) {

				ser_in = UART1_GetChar();

				if (ser_in == 'A') {
					index = 0;
					mode = 1;
					motors_stop();

					while (ser_in != 'X' && ser_in != 'Y' && ser_in != 'Z') {
						if (UART1_Available()) {
							ser_in = UART1_GetChar();

							if (ser_in == 'X')
								set_direction(directions1, array);
							else if (ser_in == 'Y')
								set_direction(directions2, array);
							else if (ser_in == 'Z')
								set_direction(directions3, array);
						}
					}
				}

				else if (ser_in == 'M') {
					mode = 0;
					motors_stop();
				}
				else if (ser_in == 'N'){
					mode = 2;
					motors_stop();
					waitms(1000);
				}

			}

			if (mode == 0) {

				switch (ser_in) {
				case '1':
					pwm = 20;
					motors_stop();
					break;
				case '2':
					pwm = 25;
					motors_stop();
					break;
				case '3':
					pwm = 30;
					motors_stop();
					break;
				case '4':
					pwm = 40;
					motors_stop();
					break;
				case '5':
					pwm = 50;
					motors_stop();
					break;
				case '6':
					pwm = 60;
					motors_stop();
					break;
				case '7':
					pwm = 70;
					motors_stop();
					break;
				case '8':
					pwm = 80;
					motors_stop();
					break;
				case '9':
					pwm = 90;
					motors_stop();
					break;
				case '0':
					pwm = 20;
					motors_stop();
					break;
				}
				if (ser_in == 'F') {
					TURN_FORWARD(pwm);
				}
				else if (ser_in == 'B') {
					TURN_BACKWARD(pwm);
				}
				else if (ser_in == 'R') {
					TURN_LEFT(pwm);
				}
				else if (ser_in == 'L') {
					TURN_RIGHT(pwm);
				}
				else if (ser_in == 'T') {
					TURN_RIGHT(35);
					waitms(2700);
					motors_stop();
					ser_in = 'S';
				}
				else if (ser_in == 'S') {
					motors_stop();
				}
			}

			else if (mode == 1) {
				left = Volts_at_Pin(QFP32_MUX_P2_2); //Middle inductor
				right = Volts_at_Pin(QFP32_MUX_P2_3); //Right inductor
				middle = Volts_at_Pin(QFP32_MUX_P1_7); //Left inductor

				if (right - left > 0.06) {
					TURN_RIGHT(30);
				}
				else if (left - right > 0.06) {
					TURN_LEFT(30);
				}

				else if (middle >= 0.2) {
					flag++;

					if (flag >= 15) {
						motors_stop();
						waitms(1000);

						if (array[index] == 0) {				// RIGHT TURN
							TURN_FORWARD(30);
							waitms(1500);
						}
						else if (array[index] == 2) { 		// LEFT TURN
							TURN_RIGHT(30);
							waitms(1500);
							TURN_FORWARD(30);
							waitms(500);
						}
						else if (array[index] == 1) {
							TURN_LEFT(30);
							waitms(1500);
							TURN_FORWARD(30);
							waitms(500);
						}
						else {
							motors_stop();
						}
						flag = 0;

						if (index < 8)
							index++;
					}

				}

				else {
					//flag = 0;
					TURN_FORWARD(35);
				}

			}// END (else if (mode == 1)
			else if (mode == 2) {
				
				if (Volts_at_Pin(QFP32_MUX_P1_3) > 1.5) {
					TURN_FORWARD(50);
				}
				
				else if (Volts_at_Pin(QFP32_MUX_P1_4) > 1.5) {
					TURN_LEFT(50);
				}


				else if (Volts_at_Pin(QFP32_MUX_P1_5) > 1.5) {
					TURN_RIGHT(50);
				}
				
				else{
					motors_stop();
				}

			}

		//} // END if (UART_available())
		}

		success = vl53l0x_read_range_single(&range);
		if (success)
		{
			printf("D: %4d (mm)\r", range - offset);
			distance = (int) ((range - offset)/10);
		}

	} // END while loop 
} // end void main  