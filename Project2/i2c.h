#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <util/delay.h>

#define SCL_DDR   DDRC
#define SCL_PORT  PORTC
#define SCL_PIN   PC5

#define SDA_DDR   DDRC
#define SDA_PORT  PORTC
#define SDA_PIN   PC4
#define SDA_IN    PINC

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);

#endif