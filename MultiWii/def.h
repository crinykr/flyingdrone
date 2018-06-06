#ifndef DEF_H_
#define DEF_H_

/** motor and servo numbers **/
#define DYNBAL       0
#define FLAP         0
#define NUMBER_MOTOR 4

/** atmega328P (Promini) **/
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_TOGGLE              PINB |= 1<<5; // switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);

#define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
#define BUZZERPIN_ON               PORTB |= 1;
#define BUZZERPIN_OFF              PORTB &= ~1;

#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5; // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); // PIN 0
//RX PIN assignment inside the port //for PORTD

/** GY86 - IMU Orientations and Sensor definitions **/
#define MPU6050
#define HMC5883
#define MS561101BA
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}

/** Sensor Type definitions **/
#define ACC 1
#define MAG 1
#define GYRO 1
#define BARO 1
#define GPS 0
#define NAVCAP 0
#define SONAR 0
#define EXTAUX 0

/** Multitype decleration for the GUI's **/
#define MULTITYPE 3

/** Some unsorted "chain" defines **/
#define BIND_CAPABLE 0  // Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module.
#define RC_CHANS 12

#endif /* DEF_H_ */
