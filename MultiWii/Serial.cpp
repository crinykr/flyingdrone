#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MultiWii.h"

static volatile uint8_t serialHeadRX[1], serialTailRX[1];
static uint8_t serialBufferRX[256][1];
static volatile uint8_t serialHeadTX[1], serialTailTX[1];
static uint8_t serialBufferTX[128][1];
extern "C" void __vector_19(void) __attribute__ ((signal,used, externally_visible));

void __vector_19(void)
{
	uint8_t t = serialTailTX[0];

	if (serialHeadTX[0] != t)
	{
		if (++t >= 128)
			t = 0;
		(*(volatile uint8_t *) (0xC6)) = serialBufferTX[t][0];
		serialTailTX[0] = t;
	}
	if (t == serialHeadTX[0])
		(*(volatile uint8_t *) (0xC1)) &= ~(1 << 5);
}

void UartSendData(uint8_t port)
{
	(*(volatile uint8_t *) (0xC1)) |= (1 << 5);
}

void SerialOpen(uint8_t port, uint32_t baud)
{
	uint8_t h = ((16000000L / 4 / baud - 1) / 2) >> 8;
	uint8_t l = ((16000000L / 4 / baud - 1) / 2);
	switch (port)
	{
	case 0:
		(*(volatile uint8_t *) (0xC0)) = (1 << 1);
		(*(volatile uint8_t *) (0xC5)) = h;
		(*(volatile uint8_t *) (0xC4)) = l;
		(*(volatile uint8_t *) (0xC1)) |= (1 << 4) | (1 << 3) | (1 << 7);
		break;
	}
}

void SerialEnd(uint8_t port)
{
	switch (port)
	{
	case 0:
		(*(volatile uint8_t *) (0xC1)) &= ~((1 << 4) | (1 << 3) | (1 << 7) | (1 << 5));
		break;
	}
}

void store_uart_in_buf(uint8_t data, uint8_t portnum)
{
	uint8_t h = serialHeadRX[portnum];
	serialBufferRX[h++][portnum] = data;
	if (h >= 256)
		h = 0;
	serialHeadRX[portnum] = h;
}

extern "C" void __vector_18(void) __attribute__ ((signal,used, externally_visible));

void __vector_18(void)
{
	store_uart_in_buf((*(volatile uint8_t *) (0xC6)), 0);
}

uint8_t SerialRead(uint8_t port)
{
	uint8_t t = serialTailRX[port];
	uint8_t c = serialBufferRX[t][port];
	if (serialHeadRX[port] != t)
	{
		if (++t >= 256)
			t = 0;
		serialTailRX[port] = t;
	}
	return c;
}

uint8_t SerialAvailable(uint8_t port)
{
	return ((uint8_t)(serialHeadRX[port] - serialTailRX[port])) % 256;
}

uint8_t SerialUsedTXBuff(uint8_t port)
{
	return ((uint8_t)(serialHeadTX[port] - serialTailTX[port])) % 128;
}

void SerialSerialize(uint8_t port, uint8_t a)
{
	uint8_t t = serialHeadTX[port];
	if (++t >= 128)
		t = 0;
	serialBufferTX[t][port] = a;
	serialHeadTX[port] = t;
}

void SerialWrite(uint8_t port, uint8_t c)
{
	SerialSerialize(port, c);
	UartSendData(port);
}
