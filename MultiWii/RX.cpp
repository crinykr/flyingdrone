#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

//crinyhere

/** Global RX related variables **/
//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
//Channel order for PPM SUM RX Configs
static uint8_t rcChannel[RC_CHANS] = { SERIAL_SUM_PPM };

void rxInt(void);

/** RX Pin Setup **/
void configureReceiver()
{
	PPM_PIN_INTERRUPT
	;
}

/** PPM SUM RX Pin reading **/
// Read PPM SUM RX Data
void rxInt(void)
{
	uint16_t now, diff;
	static uint16_t last = 0;
	static uint8_t chan = 0;

	now = micros();
	sei();
	diff = now - last;
	last = now;
	if (diff > 3000)
		chan = 0;
	else
	{
		if (900 < diff && diff < 2200 && chan < RC_CHANS)
		{ //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
			rcValue[chan] = diff;
		}
		chan++;
	}
}

/** combine and sort the RX Datas **/
uint16_t readRawRC(uint8_t chan)
{
	uint16_t data;
	uint8_t oldSREG;

	oldSREG = SREG;
	cli(); // Let's disable interrupts
	data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
	SREG = oldSREG;        // Let's restore interrupt state
	return data; // We return the value correctly copied when the IRQ's where disabled
}

/** compute and Filter the RX data **/
#define AVERAGING_ARRAY_LENGTH 4
void computeRC()
{
	static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH - 1];
	uint16_t rcDataMean, rcDataTmp;
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan, a;
	uint8_t failsafeGoodCondition = 1;

	rc4ValuesIndex++;
	if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH - 1)
		rc4ValuesIndex = 0;
	for (chan = 0; chan < RC_CHANS; chan++)
	{
		rcDataTmp = readRawRC(chan);

		if (failsafeGoodCondition)
		{
			rcDataMean = rcDataTmp;
			for (a = 0; a < AVERAGING_ARRAY_LENGTH - 1; a++)
				rcDataMean += rcData4Values[chan][a];
			rcDataMean = (rcDataMean + (AVERAGING_ARRAY_LENGTH / 2)) / AVERAGING_ARRAY_LENGTH;
			if (rcDataMean < (uint16_t) rcData[chan] - 3)
				rcData[chan] = rcDataMean + 2;
			if (rcDataMean > (uint16_t) rcData[chan] + 3)
				rcData[chan] = rcDataMean - 2;
			rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
		}
		if (chan < 8 && rcSerialCount > 0)
		{ // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
			rcSerialCount--;
			if (rcSerial[chan] > 900)
			{
				rcData[chan] = rcSerial[chan];
			} // only relevant channels are overridden
		}
	}
}
