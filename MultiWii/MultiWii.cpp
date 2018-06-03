/*
 MultiWiiCopter by Alexandre Dubus
 www.multiwii.com
 March  2015     V2.4
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"
#include <avr/pgmspace.h>

const char pidnames[] __attribute__((__progmem__)) =
		"ROLL;"
		"PITCH;"
		"YAW;"
		"ALT;"
		"Pos;"
		"PosR;"
		"NavR;"
		"LEVEL;"
		"MAG;"
		"VEL;";
const char boxnames[] __attribute__((__progmem__)) =
		"ARM;"
		"ANGLE;"
		"HORIZON;"
		"BARO;"
		"MAG;";
const uint8_t boxids[] __attribute__((__progmem__)) = { 0, 1, 2, 3, 5, };
uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;
uint16_t calibratingA = 0;
uint16_t calibratingB = 0;
uint16_t calibratingG;
int16_t magHold, headFreeModeHold;
uint8_t vbatMin = 126;
uint8_t rcOptions[CHECKBOXITEMS];
int32_t AltHold;
int16_t sonarAlt;
int16_t BaroPID = 0;
int16_t errorAltitudeI = 0;
int16_t gyroZero[3] = { 0, 0, 0 };
imu_t imu;
analog_t analog;
alt_t alt;
att_t att;
int16_t debug[4];
flags_struct_t f;
int16_t i2c_errors_count = 0;
uint16_t intPowerTrigger1;
int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;
int16_t rcData[12];
int16_t rcSerial[8];
int16_t rcCommand[4];
uint8_t rcSerialCount = 0;
int16_t lookupPitchRollRC[5];
uint16_t lookupThrottleRC[11];
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000 };
static uint8_t dynP8[2], dynD8[2];
global_conf_t global_conf;
conf_t conf;
int16_t GPS_angle[2] = { 0, 0 };
int32_t GPS_coord[2];
int32_t GPS_home[2];
int32_t GPS_hold[2];
int32_t GPS_prev[2];
int32_t GPS_poi[2];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;
int16_t GPS_directionToHome;
int32_t GPS_directionToPoi;
uint16_t GPS_altitude;
uint16_t GPS_speed;
uint8_t GPS_update = 0;
uint16_t GPS_ground_course = 0;
uint8_t NAV_state = 0;
uint8_t NAV_error = 0;
uint8_t prv_gps_modes = 0;
uint32_t nav_timer_stop = 0;
uint16_t nav_hold_time;
uint8_t NAV_paused_at = 0;
uint8_t next_step = 1;
int16_t jump_times = -10;
int16_t nav[2];
int16_t nav_rated[2];
int32_t original_altitude;
int32_t target_altitude;
int32_t alt_to_hold;
uint32_t alt_change_timer;
int8_t alt_change_flag;
uint32_t alt_change;
uint8_t alarmArray[ALRM_FAC_SIZE];
int32_t baroPressure;
int16_t baroTemperature;
int32_t baroPressureSum;

void annexCode()
{
	static uint32_t calibratedAccTime;
	uint16_t tmp, tmp2;
	uint8_t axis, prop1, prop2;

	prop2 = 128;
	if (rcData[THROTTLE] > 1500)
	{
		if (rcData[THROTTLE] < 2000)
			prop2 -= ((uint16_t) conf.dynThrPID * (rcData[THROTTLE] - 1500) >> 9);
		else
			prop2 -= conf.dynThrPID;
	}
	for (axis = 0; axis < 3; axis++)
	{
		tmp = ((((rcData[axis] - 1500) > 0 ? (rcData[axis] - 1500) : -(rcData[axis] - 1500))) < (500) ? (((rcData[axis] - 1500) > 0 ? (rcData[axis] - 1500) : -(rcData[axis] - 1500))) : (500));
		if (axis != 2)
		{
			tmp2 = tmp >> 7;
			rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
			prop1 = 128 - ((uint16_t) conf.rollPitchRate * tmp >> 9);
			prop1 = (uint16_t) prop1 * prop2 >> 7;
			dynP8[axis] = (uint16_t) conf.pid[axis].P8 * prop1 >> 7;
			dynD8[axis] = (uint16_t) conf.pid[axis].D8 * prop1 >> 7;
		}
		else
			rcCommand[axis] = tmp;
		if (rcData[axis] < 1500)
			rcCommand[axis] = -rcCommand[axis];
	}
	tmp = ((rcData[THROTTLE]) < (1100) ? (1100) : ((rcData[THROTTLE]) > (2000) ? (2000) : (rcData[THROTTLE])));
	tmp = (uint32_t)(tmp - 1100) * 2559 / (2000 - 1100);
	tmp2 = tmp / 256;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 256) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 256;

	if ((calibratingA > 0 && 1) || (calibratingG > 0))
		(*(volatile uint8_t *) ((0x03) + 0x20)) |= 1 << 5;
	else
	{
		if (f.ACC_CALIBRATED)
			(*(volatile uint8_t *) ((0x05) + 0x20)) &= ~(1 << 5);
		if (f.ARMED)
			(*(volatile uint8_t *) ((0x05) + 0x20)) |= (1 << 5);
	}

	if (currentTime > calibratedAccTime)
	{
		if (!f.SMALL_ANGLES_25)
		{
			f.ACC_CALIBRATED = 0;
			(*(volatile uint8_t *) ((0x03) + 0x20)) |= 1 << 5;
			calibratedAccTime = currentTime + 100000;
		}
		else
			f.ACC_CALIBRATED = 1;
	}
	serialCom();
}

void setup()
{
	SerialOpen(0, 115200);
	pinMode(13, 0x1);
	pinMode(8, 0x1);
	initOutput();
	readGlobalSet();
	global_conf.currentSet = 0;
	while (1)
	{
		readEEPROM();
		if (global_conf.currentSet == 0)
			break;
		global_conf.currentSet--;
	}
	readGlobalSet();
	readEEPROM();
	blinkLED(2, 40, global_conf.currentSet + 1);
	configureReceiver();
	initSensors();
	previousTime = micros();
	calibratingG = 512;
	calibratingB = 200;
	f.SMALL_ANGLES_25 = 1;
}

void go_arm()
{
	if (calibratingG == 0 && f.ACC_CALIBRATED)
	{
		if (!f.ARMED && !f.BARO_MODE)
		{
			f.ARMED = 1;
			magHold = att.heading;
		}
	}
	else if (!f.ARMED)
	{
		blinkLED(2, 255, 1);
		alarmArray[ALRM_FAC_ACC] = ALRM_LVL_ON;
	}
}

void go_disarm()
{
	if (f.ARMED)
		f.ARMED = 0;
}

void loop()
{
	static uint8_t rcDelayCommand;
	static uint8_t rcSticks;
	uint8_t axis, i;
	int16_t error, errorAngle;
	int16_t delta;
	int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
	static int16_t lastGyro[2] = { 0, 0 };
	static int16_t errorAngleI[2] = { 0, 0 };
	static int32_t errorGyroI_YAW;
	static int16_t delta1[2], delta2[2];
	static int16_t errorGyroI[2] = { 0, 0 };
	static uint16_t rcTime = 0;
	static int16_t initialThrottleHold;
	int16_t rc;
	int32_t prop = 0;

	if ((int16_t)(currentTime - rcTime) > 0)
	{
		rcTime = currentTime + 20000;
		computeRC();
		uint8_t stTmp = 0;
		for (i = 0; i < 4; i++)
		{
			stTmp >>= 2;
			if (rcData[i] > 1100)
				stTmp |= 0x80;
			if (rcData[i] < 1900)
				stTmp |= 0x40;
		}
		if (stTmp == rcSticks)
		{
			if (rcDelayCommand < 250)
				rcDelayCommand++;
		}
		else
			rcDelayCommand = 0;
		rcSticks = stTmp;
		if (rcData[THROTTLE] <= 1100)
		{
			errorGyroI[ROLL] = 0;
			errorGyroI[PITCH] = 0;
			errorGyroI_YAW = 0;
			errorAngleI[ROLL] = 0;
			errorAngleI[PITCH] = 0;
			if (conf.activate[BOXARM] > 0)
			{
				if (rcOptions[BOXARM] && f.OK_TO_ARM)
					go_arm();
				else if (f.ARMED)
					go_disarm();
			}
		}
		if (rcDelayCommand == 20)
		{
			if (f.ARMED)
			{
				if (conf.activate[BOXARM] == 0 && rcSticks == (1 << (2 * THROTTLE)) + (1 << (2 * YAW)) + (3 << (2 * PITCH)) + (3 << (2 * ROLL)))
					go_disarm();
			}
			else
			{
				i = 0;
				if (rcSticks == (1 << (2 * THROTTLE)) + (1 << (2 * YAW)) + (1 << (2 * PITCH)) + (3 << (2 * ROLL)))
				{
					calibratingG = 512;
					calibratingB = 10;
				}
				if (rcSticks == (1 << (2 * THROTTLE)) + (2 << (2 * YAW)) + (2 << (2 * PITCH)) + (3 << (2 * ROLL)))
					previousTime = micros();
				else if (conf.activate[BOXARM] == 0 && rcSticks == (1 << (2 * THROTTLE)) + (2 << (2 * YAW)) + (3 << (2 * PITCH)) + (3 << (2 * ROLL)))
					go_arm();
				else if (rcSticks == (2 << (2 * THROTTLE)) + (1 << (2 * YAW)) + (1 << (2 * PITCH)) + (3 << (2 * ROLL)))
					calibratingA = 512;
				else if (rcSticks == (2 << (2 * THROTTLE)) + (2 << (2 * YAW)) + (1 << (2 * PITCH)) + (3 << (2 * ROLL)))
					f.CALIBRATE_MAG = 1;
				i = 0;
				if (rcSticks == (2 << (2 * THROTTLE)) + (3 << (2 * YAW)) + (2 << (2 * PITCH)) + (3 << (2 * ROLL)))
				{
					conf.angleTrim[PITCH] += 2;
					i = 1;
				}
				else if (rcSticks == (2 << (2 * THROTTLE)) + (3 << (2 * YAW)) + (1 << (2 * PITCH)) + (3 << (2 * ROLL)))
				{
					conf.angleTrim[PITCH] -= 2;
					i = 1;
				}
				else if (rcSticks == (2 << (2 * THROTTLE)) + (3 << (2 * YAW)) + (3 << (2 * PITCH)) + (2 << (2 * ROLL)))
				{
					conf.angleTrim[ROLL] += 2;
					i = 1;
				}
				else if (rcSticks == (2 << (2 * THROTTLE)) + (3 << (2 * YAW)) + (3 << (2 * PITCH)) + (1 << (2 * ROLL)))
				{
					conf.angleTrim[ROLL] -= 2;
					i = 1;
				}
				if (i)
				{
					writeParams(1);
					rcDelayCommand = 0;
				}
			}
		}
		uint16_t auxState = 0;
		for (i = 0; i < 4; i++)
			auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
		for (i = 0; i < CHECKBOXITEMS; i++)
			rcOptions[i] = (auxState & conf.activate[i]) > 0;
		if (rcOptions[BOXANGLE] || (failsafeCnt > 5 * 10))
		{
			if (!f.ANGLE_MODE)
			{
				errorAngleI[ROLL] = 0;
				errorAngleI[PITCH] = 0;
				f.ANGLE_MODE = 1;
			}
		}
		else
		{
			if (f.ANGLE_MODE)
			{
				errorGyroI[ROLL] = 0;
				errorGyroI[PITCH] = 0;
			}
			f.ANGLE_MODE = 0;
		}
		if (rcOptions[BOXHORIZON])
		{
			f.ANGLE_MODE = 0;
			if (!f.HORIZON_MODE)
			{
				errorAngleI[ROLL] = 0;
				errorAngleI[PITCH] = 0;
				f.HORIZON_MODE = 1;
			}
		}
		else
		{
			if (f.HORIZON_MODE)
			{
				errorGyroI[ROLL] = 0;
				errorGyroI[PITCH] = 0;
			}
			f.HORIZON_MODE = 0;
		}
		if (rcOptions[BOXARM] == 0)
			f.OK_TO_ARM = 1;
		if (rcOptions[BOXBARO])
		{
			if (!f.BARO_MODE)
			{
				f.BARO_MODE = 1;
				AltHold = alt.EstAlt;
				initialThrottleHold = rcCommand[THROTTLE];
				errorAltitudeI = 0;
				BaroPID = 0;
			}
		}
		else
			f.BARO_MODE = 0;
		if (rcOptions[BOXMAG])
		{
			if (!f.MAG_MODE)
			{
				f.MAG_MODE = 1;
				magHold = att.heading;
			}
		}
		else
			f.MAG_MODE = 0;
	}
	else
	{
		static uint8_t taskOrder = 0;
		switch (taskOrder)
		{
		case 0:
			taskOrder++;
			if (Mag_getADC() != 0)
				break;
		case 1:
			taskOrder++;
			if (Baro_update() != 0)
				break;
		case 2:
			taskOrder++;
			if (getEstimatedAltitude() != 0)
				break;
		case 3:
			taskOrder++;
		case 4:
			taskOrder = 0;
			break;
		}
	}
	while (1)
	{
		currentTime = micros();
		cycleTime = currentTime - previousTime;
		if (cycleTime >= 2800)
			break;
	}
	previousTime = currentTime;
	computeIMU();
	if (((rcCommand[YAW]) > 0 ? (rcCommand[YAW]) : -(rcCommand[YAW])) < 70 && f.MAG_MODE)
	{
		int16_t dif = att.heading - magHold;
		if (dif <= -180)
			dif += 360;
		if (dif >= +180)
			dif -= 360;
		if (f.SMALL_ANGLES_25 || (f.GPS_mode != 0))
			rcCommand[YAW] -= dif * conf.pid[PIDMAG].P8 >> 5;
	}
	else
		magHold = att.heading;
	if (f.BARO_MODE)
	{
		static uint8_t isAltHoldChanged = 0;
		static int16_t AltHoldCorr = 0;
		if ((((rcCommand[THROTTLE] - initialThrottleHold) > 0 ? (rcCommand[THROTTLE] - initialThrottleHold) : -(rcCommand[THROTTLE] - initialThrottleHold)) > 50) && !f.THROTTLE_IGNORED)
		{
			AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
			if (((AltHoldCorr) > 0 ? (AltHoldCorr) : -(AltHoldCorr)) > 512)
			{
				AltHold += AltHoldCorr / 512;
				AltHoldCorr %= 512;
			}
			isAltHoldChanged = 1;
		}
		else if (isAltHoldChanged)
		{
			AltHold = alt.EstAlt;
			isAltHoldChanged = 0;
		}
		rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
	}
	if (f.HORIZON_MODE)
		prop = (((
				(((rcCommand[PITCH]) > 0 ? (rcCommand[PITCH]) : -(rcCommand[PITCH]))) > (((rcCommand[ROLL]) > 0 ? (rcCommand[ROLL]) : -(rcCommand[ROLL]))) ?
						(((rcCommand[PITCH]) > 0 ? (rcCommand[PITCH]) : -(rcCommand[PITCH]))) : (((rcCommand[ROLL]) > 0 ? (rcCommand[ROLL]) : -(rcCommand[ROLL]))))) < (512) ?
				(((((rcCommand[PITCH]) > 0 ? (rcCommand[PITCH]) : -(rcCommand[PITCH]))) > (((rcCommand[ROLL]) > 0 ? (rcCommand[ROLL]) : -(rcCommand[ROLL]))) ?
						(((rcCommand[PITCH]) > 0 ? (rcCommand[PITCH]) : -(rcCommand[PITCH]))) : (((rcCommand[ROLL]) > 0 ? (rcCommand[ROLL]) : -(rcCommand[ROLL]))))) :
				(512));
	for (axis = 0; axis < 2; axis++)
	{
		rc = rcCommand[axis] << 1;
		error = rc - imu.gyroData[axis];
		errorGyroI[axis] = ((errorGyroI[axis] + error) < (-16000) ? (-16000) : ((errorGyroI[axis] + error) > (+16000) ? (+16000) : (errorGyroI[axis] + error)));
		if (((imu.gyroData[axis]) > 0 ? (imu.gyroData[axis]) : -(imu.gyroData[axis])) > 640)
			errorGyroI[axis] = 0;
		ITerm = (errorGyroI[axis] >> 7) * conf.pid[axis].I8 >> 6;
		PTerm = mul(rc, conf.pid[axis].P8) >> 6;
		if (f.ANGLE_MODE || f.HORIZON_MODE)
		{
			errorAngle = ((rc + GPS_angle[axis]) < (-500) ? (-500) : ((rc + GPS_angle[axis]) > (+500) ? (+500) : (rc + GPS_angle[axis]))) - att.angle[axis] + conf.angleTrim[axis];
			errorAngleI[axis] = ((errorAngleI[axis] + errorAngle) < (-10000) ? (-10000) : ((errorAngleI[axis] + errorAngle) > (+10000) ? (+10000) : (errorAngleI[axis] + errorAngle)));
			PTermACC = mul(errorAngle, conf.pid[PIDLEVEL].P8) >> 7;
			int16_t limit = conf.pid[PIDLEVEL].D8 * 5;
			PTermACC = ((PTermACC) < (-limit) ? (-limit) : ((PTermACC) > (+limit) ? (+limit) : (PTermACC)));
			ITermACC = mul(errorAngleI[axis], conf.pid[PIDLEVEL].I8) >> 12;
			ITerm = ITermACC + ((ITerm - ITermACC) * prop >> 9);
			PTerm = PTermACC + ((PTerm - PTermACC) * prop >> 9);
		}
		PTerm -= mul(imu.gyroData[axis], dynP8[axis]) >> 6;
		delta = imu.gyroData[axis] - lastGyro[axis];
		lastGyro[axis] = imu.gyroData[axis];
		DTerm = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;
		DTerm = mul(DTerm, dynD8[axis]) >> 5;
		axisPID[axis] = PTerm + ITerm - DTerm;
	}
	rc = mul(rcCommand[YAW], (2 * conf.yawRate + 30)) >> 5;
	error = rc - imu.gyroData[YAW];
	errorGyroI_YAW += mul(error, conf.pid[YAW].I8);
	errorGyroI_YAW = ((errorGyroI_YAW) < (2 - ((int32_t) 1 << 28)) ? (2 - ((int32_t) 1 << 28)) : ((errorGyroI_YAW) > (-2 + ((int32_t) 1 << 28)) ? (-2 + ((int32_t) 1 << 28)) : (errorGyroI_YAW)));
	if (((rc) > 0 ? (rc) : -(rc)) > 50)
		errorGyroI_YAW = 0;
	PTerm = mul(error, conf.pid[YAW].P8) >> 6;
	int16_t limit = 300 - conf.pid[YAW].D8;
	PTerm = ((PTerm) < (-limit) ? (-limit) : ((PTerm) > (+limit) ? (+limit) : (PTerm)));
	ITerm = (((int16_t)(errorGyroI_YAW >> 13)) < (-250) ? (-250) : (((int16_t)(errorGyroI_YAW >> 13)) > (+250) ? (+250) : ((int16_t)(errorGyroI_YAW >> 13))));
	axisPID[YAW] = PTerm + ITerm;
	mixTable();
	writeMotors();
}
