#include "i2c.h"

// Pull line LOW = drive it as output LOW
// Release line HIGH = set as input (let pull-up resistor pull high)

#define SCL_LOW()    { SCL_DDR |=  (1<<SCL_PIN); SCL_PORT &= ~(1<<SCL_PIN); }
#define SCL_HIGH()   { SCL_DDR &= ~(1<<SCL_PIN); SCL_PORT &= ~(1<<SCL_PIN); }

#define SDA_LOW()    { SDA_DDR |=  (1<<SDA_PIN); SDA_PORT &= ~(1<<SDA_PIN); }
#define SDA_HIGH()   { SDA_DDR &= ~(1<<SDA_PIN); SDA_PORT &= ~(1<<SDA_PIN); }

#define SDA_READ()   ( SDA_IN  &   (1<<SDA_PIN) )

#define I2C_DELAY()  _delay_us(5)   // ~100kHz

void I2C_Init(void)
{
    // Start with both lines released (high via pull-ups)
    SCL_HIGH();
    SDA_HIGH();
}

void I2C_Start(void)
{
    SDA_HIGH(); I2C_DELAY();
    SCL_HIGH(); I2C_DELAY();
    SDA_LOW();  I2C_DELAY();   // SDA falls while SCL high = START
    SCL_LOW();  I2C_DELAY();
}

void I2C_Stop(void)
{
    SDA_LOW();  I2C_DELAY();
    SCL_HIGH(); I2C_DELAY();
    SDA_HIGH(); I2C_DELAY();   // SDA rises while SCL high = STOP
}

uint8_t I2C_Write(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80) {SDA_HIGH(); }
        else             {SDA_LOW(); }
        data <<= 1;

        I2C_DELAY();
        SCL_HIGH(); I2C_DELAY();
        SCL_LOW();  I2C_DELAY();
    }

    // Read ACK bit
    SDA_HIGH();         // release SDA so slave can pull low
    SCL_HIGH(); I2C_DELAY();
    uint8_t ack = !SDA_READ();   // LOW = ACK
    SCL_LOW();  I2C_DELAY();

    return ack;   // 1 = ACK, 0 = NACK
}