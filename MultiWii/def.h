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

/**************************   atmega328P (Promini)  ************************************/
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);

#define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
#define BUZZERPIN_ON            PORTB |= 1;
#define BUZZERPIN_OFF           PORTB &= ~1;

#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;

#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;

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

#define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
#define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
#define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
#define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);

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

#define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
#define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
#define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);

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

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/
#define MPU6050
#define HMC5883
#define MS561101BA
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
#define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
#undef INTERNAL_I2C_PULLUPS

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/
#define ACC 1
#define MAG 1
#define GYRO 1
#define BARO 1
#define GPS 0
#define NAVCAP 0
#define SONAR 0
#define EXTAUX 0

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#define MULTITYPE 3

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/
#define BIND_CAPABLE 0  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#define RC_CHANS 12

#define DISPLAY_2LINES
#define DISPLAY_COLUMNS 16

#endif /* DEF_H_ */
