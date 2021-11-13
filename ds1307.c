// ds1307.c
//
// 04-Sep-2020


#include "ds1307.h"
#include "i2c.h"



//-----------------------------------------------------------------------------
void ds1307_write_reg(uint8_t addr, uint8_t val)
    {
    i2c_start();
    i2c_send_byte(DS1307 | WRITE);

    i2c_send_byte(addr);
    i2c_send_byte(val);

    i2c_stop();
    }


//-----------------------------------------------------------------------------
uint8_t ds1307_read_reg(uint8_t addr)
    {
    i2c_start();
    i2c_send_byte(DS1307 | WRITE);

    i2c_send_byte(addr);

    i2c_start();
    i2c_send_byte(DS1307 | READ);

    uint8_t data = i2c_read_byte(I2C_NACK);

    i2c_stop();

    return data;
    }


//-----------------------------------------------------------------------------
void rtc_init(void)
    {
    uint8_t temp=0;

    temp = ds1307_read_reg(0x00);
    ds1307_write_reg(0x00, temp & 0b01111111); //rtc start

    temp = ds1307_read_reg(0x02);
    ds1307_write_reg(0x02, temp & 0b10111111); //set 24-hour mode

    ds1307_write_reg(0x07, 0b10000000);
    }


//-----------------------------------------------------------------------------
static void rtc_conv_data(uint8_t *d, uint8_t m)  //d-data //m-mask
    {
    *d=(((((*d & m)>>4)<<1)+(((*d & m)>>4)<<3))+(*d & 0x0F));
    }


//-----------------------------------------------------------------------------
void rtc_read(uint8_t *data)
    {
    i2c_start();
    i2c_send_byte(DS1307 | WRITE);

    i2c_send_byte(0x00);

    i2c_start();

    i2c_send_byte(DS1307 | READ);

    for(uint8_t i=0; i<7; i++) data[i] = i2c_read_byte(I2C_ACK);

    data[7] = i2c_read_byte(I2C_NACK);

    i2c_stop();

    rtc_conv_data(&data[SECONDS_REG], 0b01110000);
    rtc_conv_data(&data[MINUTES_REG], 0b01110000);
    rtc_conv_data(&data[HOURS_REG],   0b00110000);
    rtc_conv_data(&data[DATE_REG],    0b00110000);
    rtc_conv_data(&data[MONTH_REG],   0b00010000);
    rtc_conv_data(&data[YEAR_REG],    0b11110000);
    }


/* seconds */

//-----------------------------------------------------------------------------
void rtc_set_sec(uint8_t val)
    {
    ds1307_write_reg(SECONDS_REG, (((val/10)<<4)+(val%10)) & 0b01111111);
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_sec(void)
    {
    uint8_t t = ds1307_read_reg(SECONDS_REG);
    rtc_conv_data(&t, 0b01110000);
    return t;
    }


/* minutes */

//-----------------------------------------------------------------------------
void rtc_set_min(uint8_t val)
    {
    ds1307_write_reg(MINUTES_REG, (((val/10)<<4)+(val%10)) & 0b01111111);
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_min(void)
    {
    uint8_t t = ds1307_read_reg(MINUTES_REG);
    rtc_conv_data(&t, 0b01110000);
    return t;
    }

//-----------------------------------------------------------------------------
void rtc_set_min_incr(uint8_t* rtcdata)
    {
    uint8_t tmp = rtcdata[MINUTES_REG];
    if(++tmp>59) tmp=0;
    rtc_set_min(tmp);
    }


/* hours */

//-----------------------------------------------------------------------------
void rtc_set_hrs(uint8_t val)
    {
    ds1307_write_reg(HOURS_REG, (((val/10)<<4)+(val%10)) & 0b10111111);
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_hrs(void)
    {
    uint8_t t = ds1307_read_reg(HOURS_REG);
    rtc_conv_data(&t, 0b00110000);
    return t;
    }

//-----------------------------------------------------------------------------
void rtc_set_hrs_incr(uint8_t* rtcdata)
    {
    uint8_t tmp = rtcdata[HOURS_REG];
    if(++tmp>23) tmp=0;
    rtc_set_hrs(tmp);
    }


/* day */

//-----------------------------------------------------------------------------
void rtc_set_day(uint8_t val)
    {
    ds1307_write_reg(DAY_REG, (val & 0b00000111));
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_day(void)
    {
    return ds1307_read_reg(DAY_REG);
    }

//-----------------------------------------------------------------------------
void rtc_set_day_incr(uint8_t* rtcdata)
    {
    uint8_t tmp=rtcdata[DAY_REG];
    if(++tmp>7) tmp=1;
    rtc_set_day(tmp);
    }


/* date */

//-----------------------------------------------------------------------------
void rtc_set_dat(uint8_t val)
    {
    ds1307_write_reg(DATE_REG, (((val/10)<<4)+(val%10)) & 0b00111111);
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_dat(void)
    {
    uint8_t t = ds1307_read_reg(DATE_REG);
    rtc_conv_data(&t, 0b00110000);
    return t;
    }

//-----------------------------------------------------------------------------
void rtc_set_dat_incr(uint8_t* rtcdata)
    {
    uint8_t tmp=rtcdata[DATE_REG];
    if(++tmp>31) tmp=1;
    rtc_set_dat(tmp);
    }


/* month */

//-----------------------------------------------------------------------------
void rtc_set_mon(uint8_t val)
    {
    ds1307_write_reg(MONTH_REG, (((val/10)<<4)+(val%10)) & 0b00011111);
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_mon(void)
    {
    uint8_t t = ds1307_read_reg(MONTH_REG);
    rtc_conv_data(&t, 0b00010000);
    return t;
    }

//-----------------------------------------------------------------------------
void rtc_set_mon_incr(uint8_t* rtcdata)
    {
    uint8_t tmp=rtcdata[MONTH_REG];
    if(++tmp>12) tmp=1;
    rtc_set_mon(tmp);
    }


/* year */

//-----------------------------------------------------------------------------
void rtc_set_year(uint8_t val)
    {
    ds1307_write_reg(YEAR_REG, ((val/10)<<4)+(val%10));
    }

//-----------------------------------------------------------------------------
uint8_t rtc_read_year(void)
    {
    uint8_t t = ds1307_read_reg(YEAR_REG);
    rtc_conv_data(&t, 0b11110000);
    return t;
    }

//-----------------------------------------------------------------------------
void rtc_set_year_incr(uint8_t* rtcdata)
    {
    uint8_t tmp=rtcdata[YEAR_REG];
    if(++tmp>99) tmp=0;
    rtc_set_year(tmp);
    }

