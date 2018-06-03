#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);
uint8_t PWM_PIN[8] = { 9, 10, 11, 3, 6, 5, A2, 12 };

void writeMotors()
{
	(*(volatile uint16_t *) (0x88)) = motor[0] >> 3;
	(*(volatile uint16_t *) (0x8A)) = motor[1] >> 3;
	(*(volatile uint8_t *) (0xB3)) = motor[2] >> 3;
	(*(volatile uint8_t *) (0xB4)) = motor[3] >> 3;
}

void writeAllMotors(int16_t mc)
{
	for (uint8_t i = 0; i < 4; i++)
		motor[i] = mc;
	writeMotors();
}

void initOutput()
{
	for (uint8_t i = 0; i < 4; i++)
		pinMode(PWM_PIN[i], 0x1);
	(*(volatile uint8_t *) (0x80)) |= (1 << (7));
	(*(volatile uint8_t *) (0x80)) |= (1 << (5));
	(*(volatile uint8_t *) (0xB0)) |= (1 << (7));
	(*(volatile uint8_t *) (0xB0)) |= (1 << (5));
	writeAllMotors(1000);
	delay(300);

}

int16_t get_middle(uint8_t nr)
{
	return (conf.servoConf[nr].middle < 12) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

void mixTable()
{
	int16_t maxMotor;
	uint8_t i;

	motor[0] = rcCommand[THROTTLE] + axisPID[ROLL] * -1 + axisPID[PITCH] * +1 + 1 * axisPID[YAW] * -1;
	motor[1] = rcCommand[THROTTLE] + axisPID[ROLL] * -1 + axisPID[PITCH] * -1 + 1 * axisPID[YAW] * +1;
	motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] * +1 + axisPID[PITCH] * +1 + 1 * axisPID[YAW] * +1;
	motor[3] = rcCommand[THROTTLE] + axisPID[ROLL] * +1 + axisPID[PITCH] * -1 + 1 * axisPID[YAW] * -1;
	maxMotor = motor[0];
	for (i = 1; i < 4; i++)
		if (motor[i] > maxMotor)
			maxMotor = motor[i];
	for (i = 0; i < 4; i++)
	{
		if (maxMotor > 1850)
			motor[i] -= maxMotor - 1850;
		motor[i] = ((motor[i]) < (conf.minthrottle) ? (conf.minthrottle) : ((motor[i]) > (1850) ? (1850) : (motor[i])));
		if ((rcData[THROTTLE] < 1100) && !f.BARO_MODE)
			motor[i] = conf.minthrottle;
		if (!f.ARMED)
			motor[i] = 1000;
	}
}
