//
//  PCF8563RTC.h
//  IntervalTimer
//
//  Created by Jens Willy Johannsen on 30-07-11.
//  Copyright 2011 Greener Pastures. All rights reserved.
//
#ifndef PCF8563RTC
#define PCF8563RTC

void initRTC();
void setRTCClock( unsigned char year, unsigned char month, unsigned char day, unsigned char weekDay, unsigned char hour, unsigned char minute, unsigned char second);
void readRTCClock( unsigned char *year, unsigned char *month, unsigned char *day, unsigned char *weekDay, unsigned char *hour, unsigned char *minute, unsigned char *second);

int i2c_writebyte(unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char data);
int i2c_writedata( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data, unsigned char len );
unsigned char decToBcd(unsigned char val);
unsigned char bcdToDec(unsigned char val);
unsigned char i2c_transmit(unsigned char type);int i2c_readbyte( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data );
int i2c_readbyte( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data );
int i2c_readdata( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data, unsigned len );

#define rtc_write( memaddress, value ) i2c_writebyte( memaddress, RTC_ID, RTC_ADDR, value );

#endif