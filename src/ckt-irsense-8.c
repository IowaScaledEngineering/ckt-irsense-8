/*************************************************************************
Title:    CKT-IRSENSE-8 8 Channel IR Detector
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Michael Petersen, Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#define   TMD26711_ADDR   0x39
#define   INFO_ADDR       0x20

#define   PROXIMITY_THRESHOLD     0x300
#define   SENSOR_ERROR_THRESHOLD  0
#define   PPULSE_DEFAULT          8

#define   ON_DEBOUNCE_DEFAULT   1


#define   SCL     PA3

#define   SDA_1   PB0
#define   SDA_2   PB1
#define   SDA_3   PB2
#define   SDA_4   PB3
#define   SDA_5   PB4
#define   SDA_6   PB5
#define   SDA_7   PB6
#define   SDA_8   PB7

#define ON_DEBOUNCE_DEFAULT     1
#define OFF_DEBOUNCE_DEFAULT    20


static void sda_low() 
{
	PORTB &= ~(_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4) | _BV(SDA_5) | _BV(SDA_6) | _BV(SDA_7) | _BV(SDA_8));
	DDRB |= _BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4) | _BV(SDA_5) | _BV(SDA_6) | _BV(SDA_7) | _BV(SDA_8);
	_delay_us(10);
}

static void sda_high() 
{
	DDRB &= ~(_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4) | _BV(SDA_5) | _BV(SDA_6) | _BV(SDA_7) | _BV(SDA_8));
	PORTB |= (_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4) | _BV(SDA_5) | _BV(SDA_6) | _BV(SDA_7) | _BV(SDA_8));
	_delay_us(10);
}
static void scl_low() { PORTA &= ~(_BV(SCL)); _delay_us(10); }
static void scl_high() { PORTA |= _BV(SCL); _delay_us(10); }

void i2cStart()
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop()
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack = 0x00;

	do
	{
		if(byte & i)
			sda_high();
		else
			sda_low();
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	scl_high();

	ack = 0xFF ^ PINB;
	
	scl_low();

	return ack;
}

void i2cReadByte(uint8_t ack, uint8_t* data)
{
	uint8_t i, j;

	for(i=0; i<8; i++)
		data[i] = 0;

	for(i=0; i<8; i++)
	{
		for(j=0; j<8; j++)
			data[j] <<= 1;

		scl_high();

		for(j=0; j<8; j++)
			data[j] |= (PINB & _BV(j))?0x01:0x00;

		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();
}

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint8_t writeVal)
{
	uint8_t ack;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	ack = i2cWriteByte(writeVal);

	i2cStop();

	return ack;
}

uint8_t readWord(uint8_t addr, uint8_t cmd, uint16_t wdata[])
{
	uint8_t ack = 0xFF, i;
	uint8_t data[8];
	
	i2cStart();
	
	ack &= i2cWriteByte(addr << 1);
	ack &= i2cWriteByte(cmd);

	i2cStart();

	ack &= i2cWriteByte((addr << 1) | 0x01);
	i2cReadByte(1, data);
	for(i=0; i<8; i++)
		wdata[i] = data[i];

	i2cReadByte(0, data);
	for(i=0; i<8; i++)
		wdata[i] |= ((uint16_t)data[i])<<8;
	
	i2cStop();

	return ack;
}

void initializeTMD26711()
{
	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(TMD26711_ADDR, 0x80|0x00, 0x00);   // Start with everything disabled
	writeByte(TMD26711_ADDR, 0x80|0x01, 0xFF);   // Minimum ATIME
	writeByte(TMD26711_ADDR, 0x80|0x02, 0xFF);   // Maximum integration time
	writeByte(TMD26711_ADDR, 0x80|0x03, 0xFF);   // Minimum wait time
	
	// Note: IRQ not currently used
	writeByte(TMD26711_ADDR, 0x80|0x08, 0x00);   // Set interrupt low threshold to 0x0000
	writeByte(TMD26711_ADDR, 0x80|0x09, 0x00);
	writeByte(TMD26711_ADDR, 0x80|0x0A, 0x00);   // Set interrupt low threshold to 0x0300
	writeByte(TMD26711_ADDR, 0x80|0x0B, 0x03);
	writeByte(TMD26711_ADDR, 0x80|0x0C, 0x10);   // Single out-of-range cycle triggers interrupt

	writeByte(TMD26711_ADDR, 0x80|0x0D, 0x00);   // Long wait disabled
	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT); // Pulse count
	writeByte(TMD26711_ADDR, 0x80|0x0F, 0x28);   // 100% LED drive strength, 4x gain, Use channel 1 diode (ch 1 seems less sensitive to fluorescent) light)

	writeByte(TMD26711_ADDR, 0x80|0x00, 0x27);   // Power ON, Enable proximity, Enable proximity interrupt (not used currently)
}

void readTMD26711s(uint8_t* detectorState, uint8_t* failState)
{
	static uint8_t sensorError[8] = {0,0,0,0,0,0,0,0};
	static bool detect[8] = {false, false, false, false, false, false, false, false};
	static uint8_t count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t proximity[8];
	uint8_t ack = 0, i;

	*detectorState = 0;
	*failState = 0;

	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT);

	// If any of the sensors have ongoing errors, re-initialize the sensors.
	if (sensorError[0] || sensorError[1] || sensorError[2] || sensorError[3] 
		|| sensorError[4] || sensorError[5] || sensorError[6] || sensorError[7])
		initializeTMD26711();

	ack = readWord(TMD26711_ADDR, 0x80|0x20|0x18, proximity);  // Read data register (0x80 = command, 0x20 = auto-increment)

	// Check for missing ACKs, which would indicate a sensor malfunction
	for(i=0; i<8; i++)
	{
		if (0 == (ack & _BV(i)))
		{
			// Sensor's gone wonky, reset it and try again
			if (sensorError[i] < 255)
				sensorError[i]++;

			if (sensorError[i] > SENSOR_ERROR_THRESHOLD)
			{
				detect[i] = false;
				proximity[i] = 0;
				count[i] = 0;
				*failState |= _BV(i); // Indicate that this sensor has exceeded the error threshold and is considered failed
			}

			// This sensor didn't answer, disregard it for now
			continue;
		}

		sensorError[i] = 0;


		if(!detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD))
		{
			// ON debounce
			if(++count[i] > ON_DEBOUNCE_DEFAULT)
			{
				detect[i] = true;
				count[i] = 0;
			}
		}
		else if( (!detect[i] && (proximity[i] < PROXIMITY_THRESHOLD)) 
			|| (detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD)) )
		{
			count[i] = 0;
		}
		else if(detect[i] && (proximity[i] < PROXIMITY_THRESHOLD))
		{
			// OFF debounce
			if(++count[i] > OFF_DEBOUNCE_DEFAULT)
			{
				detect[i] = false;
				count[i] = 0;
			}
		}

		if (detect[i])
			*detectorState |= _BV(i);
	}
}

#define INVERT_OUTPUTS  (PA2)

#define OUTPUT_1  (PC0)
#define OUTPUT_2  (PC1)
#define OUTPUT_3  (PC2)
#define OUTPUT_4  (PC3)
#define OUTPUT_5  (PC4)
#define OUTPUT_6  (PC5)
#define OUTPUT_7  (PD0)
#define OUTPUT_8  (PD1)

void setOutputs(uint8_t outputState)
{
	PORTC = (PORTC & ~(0x3F)) | (outputState & 0x3F);
	PORTD = (PORTD & ~(0x03)) | ((outputState >> 6) & 0x03);
}

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x4D;  // 8MHz / 1024 / 0x4D ~= 100Hz
	TCCR0A = _BV(CTC0) | _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK0 |= _BV(OCIE0A);
}

volatile uint8_t readSensors = 0;
volatile uint8_t masterFlash = 0;

ISR(TIMER0_COMPA_vect)
{
	static uint8_t ticks = 0;

	if (++ticks >= 100)
		ticks = 0;

	if (0 == (ticks % 10))
		readSensors = 1;

	if (0 == (ticks % 10))
		masterFlash ^= 0xFF;
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	CLKPR = _BV(CLKPCE);
	CLKPR = 0x00;

	cli();

	initialize100HzTimer();

	DDRB = 0x00;     // PORTB is all the I2C inputs
	PORTB = _BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4) | _BV(SDA_5) | _BV(SDA_6) | _BV(SDA_7) | _BV(SDA_8);    // Enable pullup

	PORTA = _BV(SCL) | _BV(INVERT_OUTPUTS) | _BV(PA0);  // SCL high, along with pullups on SDA1-4
	DDRA |= _BV(SCL);

	DDRC = 0xFF; // Everything's an output
	PORTC = 0x00; // And they're all low
	
	DDRD = 0xFF; // Everything's an output
	PORTD = 0x00; // And they're all low
	
	initializeTMD26711();

//	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); // Enable ADC, set div/64 clock (125kHz)
	DIDR0 = _BV(ADC6D); // Make ADC6 an analog input
//	ADMUX = _BV(REFS0) | _BV(MUX2) | _BV(MUX1); // Set to VCC reference and channel ADC6

	sei();

	while(1)
	{
		wdt_reset();

		if (readSensors >= 1)
		{
			uint8_t detectorState = 0, errorState = 0;
			uint8_t outputState = 0;
			
			readTMD26711s(&detectorState, &errorState);

			outputState = detectorState & ~(errorState); // Only get real outputs from sensors in non-error
			outputState |= (errorState & masterFlash); // Add flashing for all sensors in error
			setOutputs(outputState);
			readSensors = 0;
		}
	}
}

