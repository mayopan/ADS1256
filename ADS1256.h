/*
        ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
        Written by Adien Akhmad, August 2015
	Modifified  Jan 2019 by Axel Sepulveda for ATMEGA328
*/

#ifndef ADS1256_h
#define ADS1256_h

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
	// Define PORT
	#define PORT_DRDY PORTB // Pin 9 on Arduino UNO
	#define PIN_DRDY PINB
	#define PINDEX_DRDY PB1
	#define DDR_DRDY DDRB

	#define PORT_CS PORTB // Pin 10 on Arduino UNO
	#define PIN_CS PINB
	#define PINDEX_CS PB2
	#define DDR_CS DDRB

	#define PORT_RESET PORTB // PIN 8 on Arduino UNO
	#define PIN_REST PINB
	#define PINDEX_RESET PB0
	#define DDR_RESET DDRB

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	// Define PORT
	#define PORT_DRDY PORTL // Pin 49 on Arduino Mega
	#define PIN_DRDY PINL
	#define PINDEX_DRDY PL0
	#define DDR_DRDY DDRL

	#define PORT_CS PORTB // Pin 53 on Arduino Mega
	#define PIN_CS PINB
	#define PINDEX_CS PB0
	#define DDR_CS DDRB

	#define PORT_RESET PORTL // PIN 48 on Arduino Mega
	#define PIN_REST PINL
	#define PINDEX_RESET PL1
	#define DDR_RESET DDRL
	
	// Contributions are welcome
#elif   defined(ARDUINO_ARCH_ESP32)||defined(ARDUINO_ARCH_SAMD)
	//They has no fixed CS/DataReady/Reset pins. So you have to set them in the constructor.

	// Contributions are welcome
#else 
	// Contributions are welcome
	#error "Oops! Your board architecture is not supported yet'"
#endif


enum class GAIN : unsigned char
{
	GAIN_1,
	GAIN_2,
	GAIN_4,
	GAIN_8,
	GAIN_16,
	GAIN_32,
	GAIN_64
};

// define drate codes
/*
        NOTE : 	Data Rate vary depending on crystal frequency. Data rates
   listed below assumes the crystal frequency is 7.68Mhz
                for other frequency consult the datasheet.
*/
enum class DATARATE : unsigned char
{
	RATE_2_5SPS 	= 0x03,
	RATE_5SPS		= 0x13,
	RATE_10SPS		= 0x23,
	RATE_15SPS		= 0x33,
	RATE_25SPS		= 0x43,
	RATE_30SPS		= 0x53,
	RATE_50SPS		= 0x63,
	RATE_60SPS		= 0x72,
	RATE_100SPS		= 0x82,
	RATE_500SPS		= 0x92,
	RATE_1000SPS	= 0xA1,
	RATE_2000SPS	= 0xB0,
	RATE_3750SPS	= 0xC0,
	RATE_7500SPS	= 0xD0,
	RATE_15000SPS	= 0xE0,
	RATE_30000SPS	= 0xF0
};

#include "Arduino.h"
#include "SPI.h"

class ADS1256 {
 public:
   #if defined (ARDUINO_ARCH_ESP32)||defined (ARDUINO_ARCH_SAMD)
   ADS1256(uint32_t _speedSPI, float vref, uint8_t _pinCS, uint8_t _pinRDY, uint8_t _pinRESET);
   #else
  ADS1256(float clockspdMhz, float vref, bool useresetpin);
  #endif
  void writeRegister(unsigned char reg, unsigned char wdata);
  unsigned char readRegister(unsigned char reg);
  void sendCommand(unsigned char cmd);
  float readCurrentChannel();
  void setConversionFactor(float val);
  void setChannel(byte channel);
  void setChannel(byte AIP, byte AIN);
  void begin(DATARATE drate, GAIN gain, bool bufferenable);
  void waitDRDY();
  void setGain(uint8_t gain);
  void readTest();

 private:
  void CSON();
  void CSOFF();
  unsigned long read_uint24();
  long read_int32();
  float read_float32();
  byte _pga;
  float _VREF;
  float _conversionFactor;
  #if defined (ARDUINO_ARCH_ESP32)| defined (ARDUINO_ARCH_SAMD)
  		uint8_t pinCS;
		uint8_t pinRDY;
		uint8_t pinRESET;
		uint32_t speedSPI;
  #endif

  // ADS1256 Register
	enum REG
	{
		STATUS,
		MUX,
		ADCON,
		DRATE,
		IO,
		OFC0,
		OFC1,
		OFC2,
		FSC0,
		FSC1,
		FSC2
	};

	// ADS1256 Command
	enum COMMAND
	{
		WAKEUP,
		RDATA,
		RDATAC= 0x03,
		SDATAC =0x0f,
		RREG,
		WREG =0x50,
		SELFCAL= 0xF0,
		SELFOCAL,
		SELFGCAL,
		SYSOCAL,
		SYSGCAL,
		SYNC =0xFC,
		STANDBY,
		RESET
	};
};

#endif
