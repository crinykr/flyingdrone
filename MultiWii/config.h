#ifndef CONFIG_H_
#define CONFIG_H_

/** Motor minthrottle **/
#define MINTHROTTLE 1150

/** Motor maxthrottle **/
#define MAXTHROTTLE 1850

/** Mincommand **/
#define MINCOMMAND 1000

/** constant loop time **/
#define LOOP_TIME 2800

/** Flying stuff **/
#define YAW_DIRECTION 1

/** Common for Heli & Airplane **/
#define YAW_COLL_PRECOMP 10           // (*) proportional factor in 0.1. Higher value -> higher precomp effect. value of 10 equals no/neutral effect
#define YAW_COLL_PRECOMP_DEADBAND 120 // (*) deadband for collective pitch input signal around 0-pitch input value

/** PPM Sum Reciver **/
#define SERIAL_SUM_PPM ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 // For Robe/Hitec/Futaba

/** Serial com speed **/
#define SERIAL0_COM_SPEED 115200

/** Failsafe settings **/
#define FAILSAFE_DELAY 10 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example

/** GPS **/
#define MAG_DECLINATION 4.02f

/** battery voltage monitoring **/
#define VBATNOMINAL    126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBAT_CELLS_NUM 0 // set this to the number of cells you monitor via analog pins

/** altitude hold **/
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 50

/** motor, servo and other presets **/
#define MIDRC 1500

#endif /* CONFIG_H_ */

