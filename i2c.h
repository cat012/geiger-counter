// i2c.h
//
// 04-Sep-2020


#ifndef I2C_H_
#define I2C_H_


#include <xc.h>
#include <stdint.h>


#define _XTAL_FREQ 2000000


#define I2C_DELAY  __delay_us(5)


///////////////////////////////////////////////////////////////////////////////

#define I2C_SDA_PIN   PORTCbits.RC4
#define I2C_SDA_TRIS  TRISCbits.RC4
#define I2C_SDA_LAT   LATCbits.LATC4

#define I2C_SCK_PIN   PORTCbits.RC3
#define I2C_SCK_TRIS  TRISCbits.RC3
#define I2C_SCK_LAT   LATCbits.LATC3

///////////////////////////////////////////////////////////////////////////////


#define I2C_SDA_CLR  I2C_SDA_LAT=0
#define I2C_SDA_SET  I2C_SDA_LAT=1
#define I2C_SDA_OUT  I2C_SDA_TRIS=0
#define I2C_SDA_INP  I2C_SDA_TRIS=1

#define I2C_SCK_CLR  I2C_SCK_LAT=0
#define I2C_SCK_SET  I2C_SCK_LAT=1
#define I2C_SCK_OUT  I2C_SCK_TRIS=0
#define I2C_SCK_INP  I2C_SCK_TRIS=1


#define I2C_SDA_PULL  I2C_SDA_INP
#define I2C_SDA_NULL  I2C_SDA_CLR; I2C_SDA_OUT

#define I2C_SDA_IS_PULL  I2C_SDA_PIN==1

#define I2C_SCK_PULL  I2C_SCK_INP
#define I2C_SCK_NULL  I2C_SCK_CLR; I2C_SCK_OUT


#define I2C_ACK   0
#define I2C_NACK  1


void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_send_byte(uint8_t data);
uint8_t i2c_read_byte(uint8_t ack);


#endif  //end of I2C_H_

