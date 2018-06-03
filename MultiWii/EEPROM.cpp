#include <avr/eeprom.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "MultiWii.h"
#include "Alarms.h"

void LoadDefaults(void);

uint8_t calculate_sum(uint8_t *cb, uint8_t siz)
{
	uint8_t sum = 0x55;
	while (--siz)
		sum += *cb++;
	return sum;
}

void readGlobalSet()
{
	eeprom_read_block((void*) &global_conf, (void*) 0, sizeof(global_conf));
	if (calculate_sum((uint8_t*) &global_conf, sizeof(global_conf)) != global_conf.checksum)
	{
		global_conf.currentSet = 0;
		global_conf.accZero[ROLL] = 5000;
	}
}

bool readEEPROM()
{
	uint8_t i;
	int8_t tmp;
	uint8_t y;

	global_conf.currentSet = 0;
	eeprom_read_block((void*) &conf, (void*) (global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
	if (calculate_sum((uint8_t*) &conf, sizeof(conf)) != conf.checksum)
	{
		blinkLED(6, 100, 3);
		LoadDefaults();
		return false;
	}
	for (i = 0; i < 5; i++)
	{
		lookupPitchRollRC[i] = (1526 + conf.rcExpo8 * (i * i - 15)) * i * (int32_t) conf.rcRate8 / 1192;
	}
	for (i = 0; i < 11; i++)
	{
		tmp = 10 * i - conf.thrMid8;
		y = conf.thrMid8;
		if (tmp > 0)
			y = 100 - y;
		lookupThrottleRC[i] = 100 * conf.thrMid8 + tmp * ((int32_t) conf.thrExpo8 * (tmp * tmp) / ((uint16_t) y * y) + 100 - conf.thrExpo8);
		lookupThrottleRC[i] = conf.minthrottle + (uint32_t)((uint16_t)(1850 - conf.minthrottle)) * lookupThrottleRC[i] / 10000;
	}
	return true;
}

void writeGlobalSet(uint8_t b)
{
	global_conf.checksum = calculate_sum((uint8_t*) &global_conf, sizeof(global_conf));
	eeprom_write_block((const void*) &global_conf, (void*) 0, sizeof(global_conf));
	if (b == 1)
		blinkLED(15, 20, 1);
}

void writeParams(uint8_t b)
{
	global_conf.currentSet = 0;
	conf.checksum = calculate_sum((uint8_t*) &conf, sizeof(conf));
	eeprom_write_block((const void*) &conf, (void*) (global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
	readEEPROM();
	if (b == 1)
		blinkLED(15, 20, 1);
}

void update_constants()
{
	conf.minthrottle = 1150;
	conf.mag_declination = (int16_t)(4.02f * 10);
	conf.yawCollPrecomp = 10;
	conf.yawCollPrecompDeadband = 120;
	writeParams(0);
}

void LoadDefaults()
{
	uint8_t i;

	conf.pid[ROLL].P8 = 33;
	conf.pid[ROLL].I8 = 30;
	conf.pid[ROLL].D8 = 23;
	conf.pid[PITCH].P8 = 33;
	conf.pid[PITCH].I8 = 30;
	conf.pid[PITCH].D8 = 23;
	conf.pid[PIDLEVEL].P8 = 90;
	conf.pid[PIDLEVEL].I8 = 10;
	conf.pid[PIDLEVEL].D8 = 100;

	conf.pid[YAW].P8 = 68;
	conf.pid[YAW].I8 = 45;
	conf.pid[YAW].D8 = 0;
	conf.pid[PIDALT].P8 = 64;
	conf.pid[PIDALT].I8 = 25;
	conf.pid[PIDALT].D8 = 24;

	conf.pid[PIDPOS].P8 = .15 * 100;
	conf.pid[PIDPOS].I8 = 0.0 * 100;
	conf.pid[PIDPOS].D8 = 0;
	conf.pid[PIDPOSR].P8 = 3.4 * 10;
	conf.pid[PIDPOSR].I8 = 0.14 * 100;
	conf.pid[PIDPOSR].D8 = 0.053 * 1000;
	conf.pid[PIDNAVR].P8 = 2.5 * 10;
	conf.pid[PIDNAVR].I8 = 0.33 * 100;
	conf.pid[PIDNAVR].D8 = 0.083 * 1000;

	conf.pid[PIDMAG].P8 = 40;

	conf.pid[PIDVEL].P8 = 0;
	conf.pid[PIDVEL].I8 = 0;
	conf.pid[PIDVEL].D8 = 0;

	conf.rcRate8 = 90;
	conf.rcExpo8 = 65;
	conf.rollPitchRate = 0;
	conf.yawRate = 0;
	conf.dynThrPID = 0;
	conf.thrMid8 = 50;
	conf.thrExpo8 = 0;
	for (i = 0; i < CHECKBOXITEMS; i++)
		conf.activate[i] = 0;
	conf.angleTrim[0] = 0;
	conf.angleTrim[1] = 0;
	conf.powerTrigger1 = 0;

	update_constants();
}
