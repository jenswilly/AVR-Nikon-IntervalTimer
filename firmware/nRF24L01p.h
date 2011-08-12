#include <avr/io.h>

#ifndef nRF24L01p
#define nRF24L01p

// prototypes
void nRF24L01p_init();
void nRF24L01p_readData( uint8_t *data );
uint8_t nRF24L01p_dataReady();
void nRF24L01p_send( uint8_t *data );
uint8_t nRF24L01p_isSending();
uint8_t nRF24L01p_getStatus();
void nRF24L01p_flushRX();
void nRF24L01p_startTX();
void nRF24L01p_startRX();
void nRF24L01p_set_TX_ADDR(uint8_t * address);
void nRF24L01p_set_RX_ADDR( uint8_t *address );
void nRF24L01p_read_configArray( uint8_t reg, uint8_t* buffer, uint8_t bufferLen );
void nRF24L01p_write_configArray( uint8_t reg, uint8_t* buffer, uint8_t bufferLen );
void nRF24L01p_write_configByte( uint8_t reg, uint8_t value );
void SPI_readAndSendArray( uint8_t *dataOut, uint8_t *dataIn, uint8_t len );
void SPI_sendArray( uint8_t *dataArray, uint8_t len );
uint8_t SPI_sendByte( uint8_t data );
void nRF24L01p_powerDown();

//static uint8_t nRF24L01p_channel;
//static uint8_t nRF24L01p_payload;
//static uint8_t nRF24L01p_isPTX;


/*
 SPI hard-wired pins
 SCK:	PB5, pin19, DIG13
 MISO:	PB4, pin18, DIG12
 MOSI:	PB3, pin17, DIG11
 SS:		PB2, pin16, DIG10 -not used-
 
 nRF24L01+ pins
 CE:
 CSN:
 */
#define PIN_SCK		PB5		// pin19, DIG13
#define PIN_MISO	PB4		// pin18, DIG12
#define PIN_MOSI	PB3		// pin17, DIG11
#define PIN_SS		PB2		// pin16, DIG10	-- NOT USED
#define	PIN_CE		PB1		// pin15, DIG9
#define PIN_CSN		PB0		// pin14, DIG8	

// macros for CE and CSN HIGH/LOW
// NOTE: requires the CE and CSN are on PORT B pins
#define CE_LOW		PORTB &= ~_BV( PIN_CE )
#define CE_HIGH		PORTB |= _BV( PIN_CE )
#define CSN_LOW		PORTB &= ~_BV( PIN_CSN )
#define CSN_HIGH	PORTB |= _BV( PIN_CSN )

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define RF_DR_LOW	5
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#endif
