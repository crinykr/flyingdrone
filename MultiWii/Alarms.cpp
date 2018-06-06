#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Sensors.h"
#include "Alarms.h"

void patternDecode(uint8_t resource, uint16_t first, uint16_t second, uint16_t third, uint16_t cyclepause, uint16_t endpause);
void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause);
void turnOff(uint8_t resource);
void toggleResource(uint8_t resource, uint8_t activate);
void vario_output(uint16_t d, uint8_t up);
void inline switch_led_flasher(uint8_t on);
void inline switch_landing_lights(uint8_t on);
void PilotLampSequence(uint16_t speed, uint16_t pattern, uint8_t num_patterns);

static uint8_t cycleDone[5] = { 0, 0, 0, 0, 0 }, resourceIsOn[5] = { 0, 0, 0, 0, 0 };
static uint32_t LastToggleTime[5] = { 0, 0, 0, 0, 0 };
static int16_t i2c_errors_count_old = 0;
static uint8_t SequenceActive[5] = { 0, 0, 0, 0, 0 };

void alarmHandler(void)
{
	if (i2c_errors_count > i2c_errors_count_old + 100 || i2c_errors_count < -1)
		alarmArray[ALRM_FAC_I2CERROR] = ALRM_LVL_ON;
	else
		alarmArray[ALRM_FAC_I2CERROR] = ALRM_LVL_OFF;
}

void patternDecode(uint8_t resource, uint16_t first, uint16_t second, uint16_t third, uint16_t cyclepause, uint16_t endpause)
{
	static uint16_t pattern[5][5];
	static uint8_t icnt[5] = { 0, 0, 0, 0, 0 };

	if (SequenceActive[resource] == 0) {
		SequenceActive[resource] = 1;
		pattern[resource][0] = first;
		pattern[resource][1] = second;
		pattern[resource][2] = third;
		pattern[resource][3] = endpause;
		pattern[resource][4] = cyclepause;
	}
	if (icnt[resource] < 3) {
		if (pattern[resource][icnt[resource]] != 0)
			setTiming(resource, pattern[resource][icnt[resource]], pattern[resource][4]);
	}
	else if (LastToggleTime[resource] < (millis() - pattern[resource][3])) { //sequence is over: reset everything
		icnt[resource] = 0;
		SequenceActive[resource] = 0; //sequence is now done, cycleDone sequence may begin
		alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_OFF; //reset toggle bit
		alarmArray[ALRM_FAC_CONFIRM] = ALRM_LVL_OFF; //reset confirmation bit
		turnOff(resource);
		return;
	}
	if (cycleDone[resource] == 1 || pattern[resource][icnt[resource]] == 0) { //single on off cycle is done
		if (icnt[resource] < 3)
			icnt[resource]++;
		cycleDone[resource] = 0;
		turnOff(resource);
	}
}

void turnOff(uint8_t resource)
{
	if (resource == 1) {
		if (resourceIsOn[1]) {
			BUZZERPIN_OFF
			resourceIsOn[1] = 0;
		}
	}
	else if (resource == 0) {
		if (resourceIsOn[0]) {
			resourceIsOn[0] = 0;
			LEDPIN_OFF
		}
	}
	else if (resource == 2) {
		if (resourceIsOn[2])
			resourceIsOn[2] = 0;
	}
	else if (resource == 3) {
		if (resourceIsOn[3])
			resourceIsOn[3] = 0;
	}
	else if (resource == 4) {
		if (resourceIsOn[4])
			resourceIsOn[4] = 0;
	}
}

void blinkLED(uint8_t num, uint8_t ontime, uint8_t repeat)
{
	uint8_t i, r;
	for (r = 0; r < repeat; r++) {
		for (i = 0; i < num; i++) {
			LEDPIN_TOGGLE
			// switch LEDPIN state
			delay(ontime);
		}
		delay(60); // wait 60 ms
	}
}

void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause)
{
	if (!resourceIsOn[resource] && (millis() >= (LastToggleTime[resource] + pause)) && pulse != 0) {
		resourceIsOn[resource] = 1;
		toggleResource(resource, 1);
		LastToggleTime[resource] = millis();
	}
	else if ((resourceIsOn[resource] && (millis() >= LastToggleTime[resource] + pulse)) || (pulse == 0 && resourceIsOn[resource])) {
		resourceIsOn[resource] = 0;
		toggleResource(resource, 0);
		LastToggleTime[resource] = millis();
		cycleDone[resource] = 1;
	}
}

void toggleResource(uint8_t resource, uint8_t activate)
{
	switch (resource)
	{
	case 0:
	default:
		if (activate == 1) {
			LEDPIN_ON
		}
		else {
			LEDPIN_OFF
		}
		break;
	}
	return;
}
