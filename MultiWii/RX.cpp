#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/
volatile uint16_t rcValue[12] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 };
static uint8_t rcChannel[12] = { ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4, 8, 9, 10, 11 };

void rxInt(void);

void configureReceiver()
{
	attachInterrupt(0, rxInt, 3);
}

void rxInt(void)
{
	uint16_t now, diff;
	static uint16_t last = 0;
	static uint8_t chan = 0;

	now = micros();
	__asm__ __volatile__ ("sei" ::: "memory");
	diff = now - last;
	last = now;
	if (diff > 3000)
		chan = 0;
	else
	{
		if (900 < diff && diff < 2200 && chan < 12)
		{
			rcValue[chan] = diff;
		}
		chan++;
	}
}

uint16_t readRawRC(uint8_t chan)
{
	uint16_t data;

	uint8_t oldSREG;
	oldSREG = (*(volatile uint8_t *) ((0x3F) + 0x20));
	__asm__ __volatile__ ("cli" ::: "memory");
	data = rcValue[rcChannel[chan]];
	(*(volatile uint8_t *) ((0x3F) + 0x20)) = oldSREG;

	return data;
}

void computeRC()
{
	static uint16_t rcData4Values[12][4 - 1];
	uint16_t rcDataMean, rcDataTmp;
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan, a;
	uint8_t failsafeGoodCondition = 1;

	rc4ValuesIndex++;
	if (rc4ValuesIndex == 4 - 1)
		rc4ValuesIndex = 0;
	for (chan = 0; chan < 12; chan++)
	{
		rcDataTmp = readRawRC(chan);

		if (failsafeGoodCondition)
		{
			rcDataMean = rcDataTmp;
			for (a = 0; a < 4 - 1; a++)
				rcDataMean += rcData4Values[chan][a];
			rcDataMean = (rcDataMean + (4 / 2)) / 4;
			if (rcDataMean < (uint16_t) rcData[chan] - 3)
				rcData[chan] = rcDataMean + 2;
			if (rcDataMean > (uint16_t) rcData[chan] + 3)
				rcData[chan] = rcDataMean - 2;
			rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
		}

		if (chan < 8 && rcSerialCount > 0)
		{
			rcSerialCount--;

			if (rcSerial[chan] > 900)
			{
				rcData[chan] = rcSerial[chan];
			}
		}
	}
}
