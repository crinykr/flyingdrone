#ifndef CONFIG_H_
#define CONFIG_H_

/**************************    The type of multicopter    ****************************/
#define QUADX

/****************************    Motor minthrottle    *******************************/
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
 This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1064 // special ESC (simonk)
//#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
#define MINTHROTTLE 1150 // (*) (**)

/****************************    Motor maxthrottle    *******************************/
/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/****************************    Mincommand          *******************************/
/* this is the value for the ESCs when they are not armed
 in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
#define MINCOMMAND  1000

/**********************************  I2C speed for old WMP config (useless config for other sensors)  *************/
#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

/**********************************  constant loop time  ******************************/
#define LOOP_TIME 2800

/***************************    Combined IMU Boards    ********************************/
#define GY_86           // Chinese 10 DOF with  MPU6050 HMC5883L MS5611, LLC

/********************************  PID Controller *********************************/
#define PID_CONTROLLER 1

/* NEW: not used anymore for servo coptertypes  <== NEEDS FIXING - MOVE TO WIKI */
#define YAW_DIRECTION 1
//#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction

#define ONLYARMWHENFLAT //prevent the copter from arming when the copter is tilted

/********************************    ARM/DISARM    *********************************/
/* optionally disable stick combinations to arm/disarm the motors.
 * In most cases one of the two options to arm/disarm via TX stick is sufficient */
#define ALLOW_ARM_DISARM_VIA_TX_YAW

/***********************          Cam Stabilisation             ***********************/
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms

/***********************          Airplane                       ***********************/
#define FLAPPERON_EP   { 1500, 1700 } // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio.
#define FLAPPERON_INVERT { -1, 1 }    // Change direction om flapperons { Wing1, Wing2 }

/***********************      Common for Heli & Airplane         ***********************/
#define YAW_COLL_PRECOMP 10           // (*) proportional factor in 0.1. Higher value -> higher precomp effect. value of 10 equals no/neutral effect
#define YAW_COLL_PRECOMP_DEADBAND 120 // (*) deadband for collective pitch input signal around 0-pitch input value

/***********************          Heli                           ***********************/
/* Channel to control CollectivePitch */
#define COLLECTIVE_PITCH      THROTTLE

/* Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch */
#define COLLECTIVE_RANGE { 80, 0, 80 }// {Min%, ZeroPitch offset from 1500, Max%}.
#define YAWMOTOR                 0       // If a motor is used as YAW Set to 1 else set to 0.

/* Servo mixing for heli 120
 {Coll,Nick,Roll} */
#define SERVO_NICK   { +10, -10,  0 }
#define SERVO_LEFT   { +10, +5, +10 }
#define SERVO_RIGHT  { +10, +5, -10 }

/* Limit Maximum controll for Roll & Nick  in 0-100% */
#define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }

/****************************    PPM Sum Reciver    ***********************************/
#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba

/*******************************    SBUS RECIVER    ************************************/
#define SBUS_MID_OFFSET 988 //SBUS Mid-Point at 1500

/******                Serial com speed    *********************************/
/* This is the speed of the serial interfaces */
#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
 it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000

/********                          Failsafe settings                 ********************/
/* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels)
 the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC is avaliable),
 PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
 for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed,
 and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
//#define FAILSAFE                                // uncomment  to activate the failsafe function
#define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
#define FAILSAFE_DETECT_TRESHOLD  985

/**************************************************************************************/
/***********************                  GPS                **************************/
/**************************************************************************************/
#define GPS_BAUD   57600       // GPS_BAUD will override SERIALx_COM_SPEED for the selected port
#define GPS_LED_INDICATOR
#define NAV_CONTROLS_HEADING       1    //(**)
#define NAV_TAIL_FIRST             0    //(**)
#define NAV_SET_TAKEOFF_HEADING    1    //(**)
#define MAG_DECLINATION  4.02f   //(**)
#define GPS_LEAD_FILTER               //(**)
#define GPS_WP_RADIUS              100      //(**)
#define SAFE_WP_DISTANCE           500      //(**)
#define MAX_NAV_ALTITUDE           100     //(**)
#define NAV_SPEED_MIN              100    // cm/sec //(**)
#define NAV_SPEED_MAX              400    // cm/sec //(**)
#define NAV_SLOW_NAV               0      //(**)
#define CROSSTRACK_GAIN            .4     //(**)
#define NAV_BANK_MAX 3000                 //(**)
#define RTH_ALTITUDE               15        //(**)
#define WAIT_FOR_RTH_ALT           1         //(**)
#define NAV_TAKEOVER_BARO          1         //(**)
#define IGNORE_THROTTLE            1         //(**)
#define FENCE_DISTANCE      600
#define LAND_SPEED          100

/******************************   Display settings   ***********************************/
#define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board
#define LCD_MENU_PREV 'p'
#define LCD_MENU_NEXT 'n'
#define LCD_VALUE_UP 'u'
#define LCD_VALUE_DOWN 'd'
#define LCD_MENU_SAVE_EXIT 's'
#define LCD_MENU_ABORT 'x'

/********************************************************************/
/****           battery voltage monitoring                       ****/
/********************************************************************/
#define VBATSCALE       131 // (*) (**) change this value if readed Battery voltage is different than real voltage
#define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBATLEVEL_WARN1 107 // (*) (**) 10,7V
#define VBATLEVEL_WARN2  99 // (*) (**) 9.9V
#define VBATLEVEL_CRIT   93 // (*) (**) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT          16 // Avoid beeping without any battery
#define VBAT_OFFSET       0 // offset in 0.1Volts, gets added to voltage value  - useful for zener diodes

#define VBAT_CELLS_NUM 0 // set this to the number of cells you monitor via analog pins
#define VBAT_CELLS_PINS {A0, A1, A2, A3, A4, A5 } // set this to the sequence of analog pins
#define VBAT_CELLS_OFFSETS {0, 50, 83, 121, 149, 177 } // in 0.1 volts, gets added to voltage value  - useful for zener diodes
#define VBAT_CELLS_DIVS { 75, 122,  98, 18, 30, 37 } // divisor for proportional part according to resistors - larger value here gives smaller voltage

/********************************************************************/
/****           powermeter (battery capacity monitoring)         ****/
/********************************************************************/
#define PSENSORNULL 510 /* (*) hard only: set to analogRead() value for zero current; for I=0A my sensor
                                   gives 1/2 Vss; that is approx 2.49Volt; */
#define PINT2mA 132     /* (*) hard: one integer step on arduino analog translates to mA (example 4.9 / 37 * 1000) ;
                                   soft: use fictional value, start with 100.
                                   for hard and soft: larger PINT2mA will get you larger value for power (mAh equivalent) */

/********************************************************************/
/****           altitude hold                                    ****/
/********************************************************************/
/* defines the neutral zone of throttle stick during altitude hold, default setting is
 +/-50 uncommend and change the value below if you want to change it. */
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE    50

/********************************************************************/
/****           board naming                                     ****/
/********************************************************************/
#define BOARD_NAME "MultiWii   V-.--"
#define NO_FLASH_CHECK

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/
#define VBAT_PRESCALER 16 // set this to 8 if vbatscale would exceed 255

/**************************************************************************************/
/***********************     motor, servo and other presets     ***********************/
/**************************************************************************************/
#define MIDRC 1500

/***********************         Servo Refreshrates            ***********************/
#define SERVO_RFR_50HZ
#define MEGA_HW_PWM_SERVOS
#define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode for mega and 32u4

/********************************************************************/
/****           ESCs calibration                                 ****/
/********************************************************************/
#define ESC_CALIB_LOW  MINCOMMAND
#define ESC_CALIB_HIGH 2000
//#define ESC_CALIB_CANNOT_FLY  // uncomment to activate

#define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
#define LCD_TELEMETRY_AUTO_FREQ  967// to step to next telemetry page 967 <=> 3s
#define PSENSOR_SMOOTH 16           // len of averaging vector for smoothing the PSENSOR readings; should be power of 2; set to 1 to disable
#define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
#define RSSI_SMOOTH 16              // len of averaging vector for smoothing the RSSI readings; should be power of 2; set to 1 to disable

/**************************    WMP power pin     *******************************/
#define DISABLE_POWER_PIN

#endif /* CONFIG_H_ */

