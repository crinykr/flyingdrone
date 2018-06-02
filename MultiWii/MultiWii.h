#ifndef MULTIWII_H_
#define MULTIWII_H_

#define  VERSION        240
#define  NAVI_VERSION   7     //This allow sync with GUI
#include "types.h"
#include "Alarms.h"

#define MINCHECK 1100
#define MAXCHECK 1900

extern volatile unsigned long timer0_overflow_count;

extern const char pidnames[];
extern const char boxnames[];
extern const uint8_t boxids[];

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t magHold, headFreeModeHold;
extern uint8_t vbatMin;
extern uint8_t rcOptions[CHECKBOXITEMS];
extern int32_t AltHold;
extern int16_t sonarAlt;
extern int16_t BaroPID;
extern int16_t errorAltitudeI;

extern int16_t i2c_errors_count;
extern uint8_t alarmArray[ALRM_FAC_SIZE];
extern global_conf_t global_conf;

extern imu_t imu;
extern analog_t analog;
extern alt_t alt;
extern att_t att;
extern int16_t debug[4];

extern conf_t conf;
extern int16_t annex650_overrun_count;
extern flags_struct_t f;
extern uint16_t intPowerTrigger1;

extern int16_t gyroZero[3];
extern int16_t angle[2];

extern int32_t baroPressure;
extern int16_t baroTemperature; // temp in 0.01 deg
extern int32_t baroPressureSum;

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[8];

extern int16_t failsafeEvents;
extern volatile int16_t failsafeCnt;

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern int16_t rcCommand[4];
extern uint8_t rcSerialCount;
extern int16_t lookupPitchRollRC[5];
extern uint16_t lookupThrottleRC[11];

// default POSHOLD control gains
#define POSHOLD_P              .15
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         3.4
#define POSHOLD_RATE_I         0.14      // Wind control
#define POSHOLD_RATE_D         0.053     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  2.5
#define NAV_I                  0.33      // Wind control
#define NAV_D                  0.083      //
#define NAV_IMAX               20        // degrees

// *************************************** end GPS common variables and defines ******************************************************************

extern volatile uint8_t spekFrameFlags;
extern volatile uint32_t spekTimeLast;
extern uint8_t spekFrameDone;

void annexCode();
void go_disarm();

#endif /* MULTIWII_H_ */
