#include "nRF24L01p.h"
#include <avr/io.h>

//static uint8_t nRF24L01p_channel = 5;	// default channel 5 (0-126)
static uint8_t nRF24L01p_payload = 4;	// default payload length = 2 bytes (1-32 bytes)
static uint8_t nRF24L01p_isPTX = 0;	// are we in PTX (sending) mode?
static unsigned char nRF25L01p_RX_ADDR[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

/* sends a byte on the SPI bus */
uint8_t SPI_sendByte( uint8_t data )
{
	SPDR = data;						// send data
	while( !( SPSR & _BV( SPIF )) )		// wait until SPI interrupt flag is set to indicate sending complete
		;
	return SPDR;						// read SPI data register
}

/* sends multiple bytes on the SPI bus using SPI_sendByte() */
void SPI_sendArray( uint8_t *dataArray, uint8_t len )
{
	// send all bytes
	uint8_t i;
	for( i = 0; i < len; i++)
	{
		SPI_sendByte( dataArray[i] );
	}
}

/* read data by sending dummy data buffer */
void SPI_readAndSendArray( uint8_t *dataOut, uint8_t *dataIn, uint8_t len )
{
	uint8_t i;
	for( i = 0; i < len; i++)
	{
		dataIn[i] = SPI_sendByte( dataOut[i] );	// send dummy data and receive read data
	}
}

/* sends a byte to the specified configuration register
 * pass the 5 bit register address in the 'reg' parameter. b0010 0000 will be OR'ed before sending. 	
 */
void nRF24L01p_write_configByte( uint8_t reg, uint8_t value )
{
	reg &= 0x1F;	// and b00011111 to enforce reg specifying only the last 5 bits
	
	CSN_LOW;							// chip select LOW
	SPI_sendByte( W_REGISTER | reg );	// OR W_REGISTER command
	SPI_sendByte( value );				// send value byte
	CSN_HIGH;							// we're done: chip select HIGH again
}

/* sends multiple bytes to the specified config register
 * pass the 5 bit register address in the 'reg' parameter. b0010 0000 will be OR'ed before sending.
 */
void nRF24L01p_write_configArray( uint8_t reg, uint8_t* buffer, uint8_t bufferLen ) 
{
	reg &= 0x1F;	// and b00011111 to enforce reg specifying only the last 5 bits
	
	CSN_LOW;							// chip select LOW
	SPI_sendByte( W_REGISTER | reg );	// OR W_REGISTER command
	SPI_sendArray( buffer, bufferLen ); // send data bytes
	CSN_HIGH;							// we're done: chip select HIGH again
}

// Reads an array of bytes from the given start position in the MiRF registers.
void nRF24L01p_read_configArray( uint8_t reg, uint8_t* buffer, uint8_t bufferLen )
{
	reg &= 0x1F;	// and b00011111 to enforce reg specifying only the last 5 bits
	
	CSN_LOW;										// chip select LOW
	SPI_sendByte( R_REGISTER | reg );				// OR R_REGISTER command
    SPI_readAndSendArray( buffer, buffer, bufferLen );	// read data
	CSN_HIGH;
}

/* sets receive address on data pipe 1
 * NOTE: assumes an address length of 5 bytes
 * pass a pointer to a 5 byte string: (uint8_t*)"rx001"
 */
void nRF24L01p_set_RX_ADDR( uint8_t *address ) 
{
	//	CE_LOW;
	nRF24L01p_write_configArray( RX_ADDR_P1, address, 5 );
	//	CE_HIGH;
}

/* sets transmit address on data pipes 0 and 1
 * (same address must be set on RX data pipe 0 for auto-ack to work)
 * NOTE: assumes an address length of 5 bytes
 * pass a pointer to a 5 byte string: (uint8_t*)"tx001"
 */
void nRF24L01p_set_TX_ADDR(uint8_t * address)
{
	nRF24L01p_write_configArray( RX_ADDR_P0, address, 5 );
	nRF24L01p_write_configArray( TX_ADDR, address, 5 );
}

/* power up in RX mode and start listening */
void nRF24L01p_startRX()
{
	nRF24L01p_isPTX = 0;	// no: in RX mode
	
	CE_LOW;
	nRF24L01p_write_configByte( CONFIG, 0x3B );	// IRQ on RX_DR only, 1 bit CRC enabled, PRIM_RX, power up
	CE_HIGH;

	//	nRF24L01p_write_configByte( STATUS, _BV( TX_DS ) | _BV( MAX_RT )); // clear TX_DS and MAX_RT flags from status
}

/* power up in TX mode */
void nRF24L01p_startTX()
{
	nRF24L01p_isPTX = 1;	// yes: we are in TX mode
	// config: power up, TX mode, no interrupt on TX_DS, 2 bit CRC on
	nRF24L01p_write_configByte( CONFIG, _BV( PWR_UP ) | _BV( MASK_TX_DS ) | _BV( EN_CRC ) | _BV( CRCO ));
}

/* flush RX buffer */
void nRF24L01p_flushRX()
{
	CSN_LOW;
	SPI_sendByte( FLUSH_RX );
	CSN_HIGH;
}

/* returns the STATUS register */
uint8_t nRF24L01p_getStatus()
{
	uint8_t status;
	nRF24L01p_read_configArray( STATUS, &status, 1) ;
	return status;
}

uint8_t nRF24L01p_isSending()
{
	uint8_t status;
	
	// are we in TX mode?
	if( nRF24L01p_isPTX )
	{
		// yes: get status
		status = nRF24L01p_getStatus();
		
		// send completed or max retry limit reached?
		if( status & (_BV( TX_DS ) | _BV( MAX_RT )) )
		{
			// yes: we are not sending
			nRF24L01p_startRX();	// why?
			return 0;
		}
		
		// no: we are still sending
		return 1;
	}
	
	// no (not in TX mode): we are not sending
	return 0;
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
/*
 void nRF24L01p_send( uint8_t *data ) 
{
    uint8_t status;
    
	// wait until we're not sending
	if( nRF24L01p_isPTX )
	{
		do
		{
			status = nRF24L01p_getStatus();
		} while( (status & ( _BV( TX_DS) | _BV( MAX_RT ))) == 0 );	// until data sent of max retries exceeded
	}
	
	nRF24L01p_isPTX = 0;
	
	CE_LOW;
	nRF24L01p_startTX();		// power up TX
	
	CSN_LOW;
	SPI_sendByte( FLUSH_TX );	// flush TX buffer
	CSN_HIGH;
	
	CSN_LOW;
	SPI_sendByte( W_TX_PAYLOAD );				// send payload command
	SPI_sendArray( data, nRF24L01p_payload );	// send data
	CSN_HIGH;
	
	CE_HIGH;					// send it!
}
*/
/* is data ready?
 * 1=yes, 0=no
 */
uint8_t nRF24L01p_dataReady() 
{
	uint8_t status = nRF24L01p_getStatus();
	return( status & _BV( RX_DR ));
}

/* read data into buffer
 * length is defined by nRF24L01p_payload
 */
void nRF24L01p_readData( uint8_t* data )
{
	CSN_LOW;
	SPI_sendByte( R_RX_PAYLOAD );							// read payload command
	SPI_readAndSendArray( data, data, nRF24L01p_payload );	// read data
	CSN_HIGH;
	
	nRF24L01p_write_configByte( STATUS, _BV( RX_DR ));		// clear RX_DR flag
}

void nRF24L01p_powerDown()
{
	nRF24L01p_write_configByte( STATUS, 0x70 );		// Power down
}

void nRF24L01p_init()
{
	/*
	// set pin modes
	DDRB |= _BV( PIN_SCK ) | _BV( PIN_MOSI ) | _BV( PIN_SS ) | _BV( PIN_CE ) | _BV( PIN_CSN );	// all pins OUTPUT
	DDRB &= ~_BV( PIN_MISO );		// except MISO, which is INPUT
	
	SPCR = _BV( SPR0 );				// CLK rate: Fosc/4; MSB first, SPI mode 0 (sample on rising leading edge, setup on falling trailing edge of CLK pulse)
	SPCR |= (1<<SPE) | (1<<MSTR);	// SPI enable + SPI master mode
	uint8_t tmp;
	tmp = SPSR;						// read SPI status...
	tmp = SPDR;						// and SPI data registers to clear flags
	
	// CE LOW and CSN HIGH
	CE_LOW;
	CSN_HIGH;
	
	// set channel
	nRF24L01p_write_configByte( RF_CH, nRF24L01p_channel );
	
	// data rate 250 kbps and high power
	nRF24L01p_write_configByte( RF_SETUP, _BV( RF_DR_LOW ) | 3 << RF_PWR );	// SETUP = 00100110: 250 kbps @ 0 dBm
	
	// set payload length on data pipes 0 and 1
	nRF24L01p_write_configByte( RX_PW_P0, nRF24L01p_payload );
	nRF24L01p_write_configByte( RX_PW_P1, nRF24L01p_payload );
	
	// power up
	nRF24L01p_startRX();
	
	// flush RX buffer
	nRF24L01p_flushRX();
	 */
	
	CE_LOW;
	nRF24L01p_write_configByte( STATUS, 0x39 );		// Enable 1 bit CRC, set PRIM_RX, IRQ only by RX_DR, Power down
	nRF24L01p_write_configByte( EN_AA, 0x00 );		// Disable auto-acknowledge
	nRF24L01p_write_configByte( SETUP_AW, 0x03 );	// Address width: 5 bytes
	nRF24L01p_write_configByte( RF_SETUP, 0x07 );	// 1 Mbps, 0 dBm
	nRF24L01p_write_configByte( RX_PW_P0, 0x04 );	// Pipeline 0 packet length = 4 bytes
	nRF24L01p_write_configByte( RF_CH, 0x02 );		// RF channel 2
	nRF24L01p_write_configArray( RX_ADDR_P0, nRF25L01p_RX_ADDR, 5 );
	
	// CE is low and unit is powered down
}