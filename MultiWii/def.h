#ifndef DEF_H_
#define DEF_H_

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
#define PROMINI

/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#define SERVO_RATES      {30,30,100,100,100,100,100,100}
#define DYNBAL 0
#define FLAP 0
#define TRI_SERVO  6

#define NUMBER_MOTOR     4

#if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT))&& defined(CAMTRIG)
#define SEC_SERVO_FROM   1 // use servo from 1 to 3
#define SEC_SERVO_TO     3
#else
#if defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)
// if A0 and A1 is taken by motors, we can use A2 and 12 for Servo tilt
#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
#define SEC_SERVO_FROM   3 // use servo from 3 to 4
#define SEC_SERVO_TO     4
#else
#define SEC_SERVO_FROM   1 // use servo from 1 to 2
#define SEC_SERVO_TO     2
#endif
#endif
#if defined(CAMTRIG)
#define SEC_SERVO_FROM   3 // use servo 3
#define SEC_SERVO_TO     3
#endif
#endif

#if defined(SIRIUS_AIR) || defined(SIRIUS_AIR_GPS)
#define RCAUX2PIND17
#endif

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
#if !defined(MONGOOSE1_0)
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);
#endif
#if !defined(RCAUXPIN8)
#if !defined(MONGOOSE1_0)
#define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
#if NUMBER_MOTOR >4
#undef PILOTLAMP
#endif
#if defined PILOTLAMP && NUMBER_MOTOR <5
#define    PL_PIN_ON            PORTB |= 1;
#define    PL_PIN_OFF           PORTB &= ~1;
#else
#define BUZZERPIN_ON            PORTB |= 1;
#define BUZZERPIN_OFF           PORTB &= ~1;
#endif
#endif
#else
#define BUZZERPIN_PINMODE          ;
#define BUZZERPIN_ON               ;
#define BUZZERPIN_OFF              ;
#define RCAUXPIN
#endif
#if !defined(RCAUXPIN12) && !defined(DISABLE_POWER_PIN)
#define POWERPIN_PINMODE           pinMode (12, OUTPUT);
#define POWERPIN_ON                PORTB |= 1<<4;
#define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
#else
#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;
#endif
#if defined(RCAUXPIN12)
#define RCAUXPIN
#endif
#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#if !defined(MONGOOSE1_0)
#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#endif
#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
#define RX_SERIAL_PORT             0
//RX PIN assignment inside the port //for PORTD
#define THROTTLEPIN                2
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     6
#define AUX1PIN                    7
#define AUX2PIN                    0 // optional PIN 8 or PIN 12
#define AUX3PIN                    1 // unused
#define AUX4PIN                    3 // unused

#define PCINT_PIN_COUNT            5
#define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
#define PCINT_RX_PORT              PORTD
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PIND
#define V_BATPIN                   A3    // Analog PIN 3
#define PSENSORPIN                 A2    // Analog PIN 2

#if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
#define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0;
#define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0);
#define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1;
#define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1);
#else
#define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
#define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
#define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
#define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
#endif
#define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
#define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
#define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
#define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);

#define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // TILT_PITCH - WING left
#define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
#define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
#define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // TILT_ROLL  - WING right
#define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
#define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
#define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // CAM TRIG  - alt TILT_PITCH
#define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
#define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
#if !defined(MONGOOSE1_0)
#define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
#define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
#define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
#endif
#define SERVO_5_PINMODE            pinMode(11,OUTPUT); // BI LEFT
#define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
#define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);
#define SERVO_6_PINMODE            pinMode(3,OUTPUT);  // TRI REAR - BI RIGHT
#define SERVO_6_PIN_HIGH           PORTD|= 1<<3;
#define SERVO_6_PIN_LOW            PORTD &= ~(1<<3);
#define SERVO_7_PINMODE            pinMode(10,OUTPUT); // new
#define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
#define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
#define SERVO_8_PINMODE            pinMode(9,OUTPUT); // new
#define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
#define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
#endif

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/
#if defined(GY_86)
#define MPU6050
#define HMC5883
#define MS561101BA
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
#define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
#undef INTERNAL_I2C_PULLUPS
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(BMA280) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(LSM330) || defined(MMA8451Q)
#define ACC 1
#else
#define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
#define MAG 1
#else
#define MAG 0
#endif

#if defined(ITG3200) || defined(ITG3050) || defined(L3G4200D) || defined(MPU6050) || defined(LSM330) || defined(MPU3050) || defined(WMP)
#define GYRO 1
#else
#define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
#define BARO 1
#else
#define BARO 0
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS)
#define GPS 1
#else
#define GPS 0
#endif

#if defined(USE_MSP_WP)
#define NAVCAP 1
#else
#define NAVCAP 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(I2C_GPS_SONAR)
#define SONAR 1
#else
#define SONAR 0
#endif

#if defined(EXTENDED_AUX_STATES)
#define EXTAUX 1
#else
#define EXTAUX 0
#endif

#if defined(RX_RSSI_CHAN)
#define RX_RSSI
#endif

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#define MULTITYPE 3

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

//all new Special RX's must be added here
//this is to avoid confusion :)
// Spektrum Satellite
#define BIND_CAPABLE 0  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#define RC_CHANS 12

#define DISPLAY_2LINES
#define DISPLAY_COLUMNS 16

/**************************************************************************************/
/***************               override defaults                   ********************/
/**************************************************************************************/

/***************               pin assignments ?  ********************/
#ifdef OVERRIDE_V_BATPIN
#undef V_BATPIN
#define V_BATPIN OVERRIDE_V_BATPIN
#endif
#ifdef OVERRIDE_PSENSORPIN
#undef PSENSORPIN
#define PSENSORPIN OVERRIDE_PSENSORPIN
#endif
#ifdef OVERRIDE_LEDPIN_PINMODE
#undef LEDPIN_PINMODE
#undef LEDPIN_TOGGLE
#undef LEDPIN_OFF
#undef LEDPIN_ON
#define LEDPIN_PINMODE OVERRIDE_LEDPIN_PINMODE
#define LEDPIN_TOGGLE  OVERRIDE_LEDPIN_TOGGLE
#define LEDPIN_OFF     OVERRIDE_LEDPIN_OFF
#define LEDPIN_ON      OVERRIDE_LEDPIN_ON
#endif
#ifdef OVERRIDE_BUZZERPIN_PINMODE
#undef BUZZERPIN_PINMODE
#undef BUZZERPIN_ON
#undef BUZZERPIN_OFF
#define BUZZERPIN_PINMODE OVERRIDE_BUZZERPIN_PINMODE
#define BUZZERPIN_ON      OVERRIDE_BUZZERPIN_ON
#define BUZZERPIN_OFF     OVERRIDE_BUZZERPIN_OFF
#endif

#endif /* DEF_H_ */
