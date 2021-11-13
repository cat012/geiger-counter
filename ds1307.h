// ds1307.h
//
// 04-Sep-2020


#ifndef DS1307_H_
#define DS1307_H_


#include <stdint.h>


//DS1307 I2C address
#define DS1307 0b11010000
#define WRITE  0b00000000
#define READ   0b00000001


#define SECONDS_REG  0
#define MINUTES_REG  1
#define HOURS_REG    2
#define DAY_REG      3
#define DATE_REG     4
#define MONTH_REG    5
#define YEAR_REG     6
#define CONTROL_REG  7


#define DS1307_RTC_SIZE    7

#define DS1307_RAM_SIZE    56
#define DS1307_RAM_OFFSET  0x08
#define DS1307_RAM_MAX     0x3f


void rtc_init(void);

void ds1307_write_reg(uint8_t addr, uint8_t val);
uint8_t ds1307_read_reg(uint8_t addr);

void rtc_read(uint8_t *data);

void rtc_set_sec(uint8_t val);
uint8_t rtc_read_sec(void);

void rtc_set_min(uint8_t val);
uint8_t rtc_read_min(void);
void rtc_set_min_incr(uint8_t* rtcdata);

void rtc_set_hrs(uint8_t val);
uint8_t rtc_read_hrs(void);
void rtc_set_hrs_incr(uint8_t* rtcdata);

void rtc_set_day(uint8_t val);
uint8_t rtc_read_day(void);
void rtc_set_day_incr(uint8_t* rtcdata);

void rtc_set_dat(uint8_t val);
uint8_t rtc_read_dat(void);
void rtc_set_dat_incr(uint8_t* rtcdata);

void rtc_set_mon(uint8_t val);
uint8_t rtc_read_mon(void);
void rtc_set_mon_incr(uint8_t* rtcdata);

void rtc_set_year(uint8_t val);
uint8_t rtc_read_year(void);
void rtc_set_year_incr(uint8_t* rtcdata);


#endif //DS1307_H_

