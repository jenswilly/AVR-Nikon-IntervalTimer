//
//  PCF8563RTC.c
//  IntervalTimer
//
//  Created by Jens Willy Johannsen on 30-07-11.
//  Copyright 2011 Greener Pastures. All rights reserved.
//

#include "PCF8563RTC.h"
#include <compat/twi.h>

#define MAX_TRIES 10

#define RTC_ID    0xA0        // I2C PCF8563 EEPROM Device Identifier
#define RTC_ADDR  0x01        // I2C PCF8563 EEPROM Device Address
// For final addresses of write 0xA2 and read 0xA3
// Bits will be AND'ed and shifted and OR'ed with the R/W bit in i2c_writebyte


#define I2C_START 0
#define I2C_DATA  1
#define I2C_STOP  2

void initRTC()
{
	// Reset control registers 1 and 2
	rtc_write( 0, 0 );	// normal run mode
	rtc_write( 1, 0 );	// no interrupts
}

void setRTCClock( unsigned char year, unsigned char month, unsigned char day, unsigned char weekDay, unsigned char hour, unsigned char minute, unsigned char second)
{
	char data[7] = { decToBcd( second ), decToBcd( minute ), decToBcd( hour ), decToBcd( day ), decToBcd( weekDay ), decToBcd( month ), decToBcd( year )};
	i2c_writedata( 2, RTC_ID, RTC_ADDR, data, 7 );

}

void readRTCClock( unsigned char *year, unsigned char *month, unsigned char *day, unsigned char *weekDay, unsigned char *hour, unsigned char *minute, unsigned char *second)
{
	char data[7];
	unsigned char read;
	read = i2c_readdata( 2, RTC_ID, RTC_ADDR, data, 7 );
	
	if( read == 7 )
	{
		*second = bcdToDec( data[0] );
		*minute = bcdToDec( data[1] );
		*hour = bcdToDec( data[1] );
		*day = bcdToDec( data[1] );
		*weekDay = bcdToDec( data[1] );
		*month = bcdToDec( data[1] );
		*year = bcdToDec( data[1] );
	}
}

// Convert normal decimal numbers to binary coded decimal
unsigned char decToBcd(unsigned char val)
{
	return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
unsigned char bcdToDec(unsigned char val)
{
	return ( (val/16*10) + (val%16) );
}

unsigned char i2c_transmit(unsigned char type)
{
	switch(type)
	{
		case I2C_START:    // Send Start Condition
			TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
			break;
		case I2C_DATA:     // Send Data
			TWCR = (1 << TWINT) | (1 << TWEN);
			break;
		case I2C_STOP:     // Send Stop Condition
			TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
			return 0;
	}
	
	// Wait for TWINT flag set in TWCR Register
	while (!(TWCR & (1 << TWINT)));
	
	// Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
	return (TWSR & 0xF8);
}

int i2c_writedata( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data, unsigned char len )
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	
i2c_retry:
	if (n++ >= MAX_TRIES) 
		return r_val;
	
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_W)
	TWDR = (dev_id & 0xF0) | ((dev_addr & 0x07) << 1) | TW_WRITE;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MT_SLA_ACK) 
		goto i2c_quit;
	
	// Send the Low 8-bit of I2C Address
	TWDR = i2c_address;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	// Send the High 8-bit of I2C Address
	TWDR = i2c_address >> 8;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	for (; len > 0; len--)
    {
 		// Put data into data register and start transmission
		TWDR = *data++;	// And increase data counter
		
		// Transmit I2C Data
		twi_status=i2c_transmit(I2C_DATA);
		
		// Check the TWSR status
		if (twi_status != TW_MT_DATA_ACK) 
			goto i2c_quit;
		
	}
	
	// TWI Transmit Ok
	r_val=1;
	
i2c_quit:
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_STOP);
	return r_val;
}

int i2c_writebyte(unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char data)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	
i2c_retry:
	if (n++ >= MAX_TRIES) 
		return r_val;
	
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_W)
	TWDR = (dev_id & 0xF0) | ((dev_addr & 0x07) << 1) | TW_WRITE;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MT_SLA_ACK) 
		goto i2c_quit;
	
	// Send word address – high 4 bits are not used
	TWDR = i2c_address & 0x0F;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	// Put data into data register and start transmission
	TWDR = data;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	// TWI Transmit Ok
	r_val=1;
	
i2c_quit:
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_STOP);
	return r_val;
}

int i2c_readbyte( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data )
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	
i2c_retry:
	if (n++ >= MAX_TRIES) 
		return r_val;
	
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWSR status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_W) 0xa0
	//	TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_WRITE;
	TWDR = (dev_id & 0xF0) | ((dev_addr & 0x07) << 1) | TW_WRITE;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MT_SLA_ACK) 
		goto i2c_quit;
	
	// Send the word address (high 4 bits unused)
	TWDR = i2c_address & 0x0F;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	// Send start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWSR status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_R)
	TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_READ;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);  
	
	// Check the TWSR status
	if ((twi_status == TW_MR_SLA_NACK) || (twi_status == TW_MR_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MR_SLA_ACK) 
		goto i2c_quit;
	
	// Read I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	if (twi_status != TW_MR_DATA_NACK) 
		goto i2c_quit;
	
	// Get the Data
	*data=TWDR;
	r_val=1;
	
i2c_quit:
	// Send Stop Condition
	twi_status=i2c_transmit(I2C_STOP);
	return r_val;
}

int i2c_readdata( unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data, unsigned len )
{
	unsigned char n = 0;
	unsigned char twi_status;
	unsigned char twcr;
	char r_val = -1;
	
i2c_retry:
	if (n++ >= MAX_TRIES) 
		return r_val;
	
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWSR status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_W) 0xa0
	//	TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_WRITE;
	TWDR = (dev_id & 0xF0) | ((dev_addr & 0x07) << 1) | TW_WRITE;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MT_SLA_ACK) 
		goto i2c_quit;
	
	// Send the word address (high 4 bits unused)
	TWDR = i2c_address & 0x0F;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) 
		goto i2c_quit;
	
	// Send start Condition
	twi_status=i2c_transmit(I2C_START);
	
	// Check the TWSR status
	if (twi_status == TW_MT_ARB_LOST) 
		goto i2c_retry;
	
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) 
		goto i2c_quit;
	
	// Send slave address (SLA_R)
	TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_READ;
	
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);  
	
	// Check the TWSR status
	if ((twi_status == TW_MR_SLA_NACK) || (twi_status == TW_MR_ARB_LOST)) 
		goto i2c_retry;
	
	if (twi_status != TW_MR_SLA_ACK) 
		goto i2c_quit;
	
	// Read I2C Data
	r_val = 0;
	for( twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
		 len > 0;
		 len-- )
    {
		if( len == 1 )
			twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */
		TWCR = twcr;              /* clear int to start transmission */

		while ( !(TWCR & _BV(TWINT)) )
			; /* wait for transmission */
		
		switch( (twi_status = TW_STATUS) )	// TW_STATUS masks prescaler bits from status register
        {
			case TW_MR_DATA_NACK:
				len = 0;              /* force end of loop */
				/* FALLTHROUGH */

			case TW_MR_DATA_ACK:
				*data++ = TWDR;
				r_val++;
				break;
				
			default:
				r_val = -1;
				goto i2c_quit;
        }
    }

	twi_status=i2c_transmit(I2C_DATA);
	if (twi_status != TW_MR_DATA_NACK) 
		goto i2c_quit;
	
	// Get the Data
	*data=TWDR;
	r_val=1;
	
i2c_quit:
	// Send Stop Condition
	twi_status=i2c_transmit(I2C_STOP);
	return r_val;
}