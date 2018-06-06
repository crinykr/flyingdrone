#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//range id [50-99] won't be assigned and can therefore be used for any custom multiwii fork without further MSP id conflict
#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t CURRENTPORT = 0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateCommand(uint8_t c);

static uint8_t read8()
{
	return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff;
}
static uint16_t read16()
{
	uint16_t t = read8();
	t += (uint16_t) read8() << 8;
	return t;
}
static uint32_t read32()
{
	uint32_t t = read16();
	t += (uint32_t) read16() << 16;
	return t;
}

static void serialize8(uint8_t a)
{
	SerialSerialize(CURRENTPORT, a);
	checksum[CURRENTPORT] ^= a;
}
static void serialize16(int16_t a)
{
	serialize8((a) & 0xFF);
	serialize8((a >> 8) & 0xFF);
}
static void serialize32(uint32_t a)
{
	serialize8((a) & 0xFF);
	serialize8((a >> 8) & 0xFF);
	serialize8((a >> 16) & 0xFF);
	serialize8((a >> 24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s)
{
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	checksum[CURRENTPORT] = 0; // start calculating a new checksum
	serialize8(s);
	serialize8(cmdMSP[CURRENTPORT]);
}

static void headSerialReply(uint8_t s)
{
	headSerialResponse(0, s);
}

static void headSerialError()
{
	headSerialResponse(1, 0);
}

static void tailSerialReply()
{
	serialize8(checksum[CURRENTPORT]);
	UartSendData(CURRENTPORT);
}

static void serializeNames(PGM_P s)
{
	headSerialReply(strlen_P(s));
	for (PGM_P c = s; pgm_read_byte(c); c++)
		serialize8(pgm_read_byte(c));
	tailSerialReply();
}

static void __attribute__ ((noinline)) s_struct_w(uint8_t *cb, uint8_t siz)
{
	while (siz--)
		*cb++ = read8();
}

static void s_struct_partial(uint8_t *cb, uint8_t siz)
{
	while (siz--)
		serialize8(*cb++);
}

static void s_struct(uint8_t *cb, uint8_t siz)
{
	headSerialReply(siz);
	s_struct_partial(cb, siz);
	tailSerialReply();
}

static void mspAck()
{
	headSerialReply(0);
	tailSerialReply();
}

enum MSP_protocol_bytes
{
	IDLE, HEADER_START, HEADER_M, HEADER_ARROW, HEADER_SIZE, HEADER_CMD
};

void serialCom()
{
	uint8_t c, cc, port, state, bytesTXBuff;
	static uint8_t offset[UART_NUMBER];
	static uint8_t dataSize[UART_NUMBER];
	static uint8_t c_state[UART_NUMBER];
	uint32_t timeMax; // limit max time in this function in case of GPS

	timeMax = micros();
	for (port = 0; port < UART_NUMBER; port++)
	{
		CURRENTPORT = port;
#define RX_COND
		cc = SerialAvailable(port);
		while (cc--RX_COND)
		{
			bytesTXBuff = SerialUsedTXBuff(port); // indicates the number of occupied bytes in TX buffer
			if (bytesTXBuff > TX_BUFFER_SIZE - 50)
				return; // ensure there is enough free TX buffer to go further (50 bytes margin)
			c = SerialRead(port);
			state = c_state[port];
			if (state == IDLE)
			{
				if (c == '$')
					state = HEADER_START;
			}
			else if (state == HEADER_START)
			{
				state = (c == 'M') ? HEADER_M : IDLE;
			}
			else if (state == HEADER_M)
			{
				state = (c == '<') ? HEADER_ARROW : IDLE;
			}
			else if (state == HEADER_ARROW)
			{
				if (c > INBUF_SIZE)
				{  // now we are expecting the payload size
					state = IDLE;
					continue;
				}
				dataSize[port] = c;
				checksum[port] = c;
				offset[port] = 0;
				indRX[port] = 0;
				state = HEADER_SIZE; // the command is to follow
			}
			else if (state == HEADER_SIZE)
			{
				cmdMSP[port] = c;
				checksum[port] ^= c;
				state = HEADER_CMD;
			}
			else if (state == HEADER_CMD)
			{
				if (offset[port] < dataSize[port])
				{
					checksum[port] ^= c;
					inBuf[offset[port]++][port] = c;
				}
				else
				{
					if (checksum[port] == c) // compare calculated and transferred checksum
						evaluateCommand(cmdMSP[port]); // we got a valid packet, evaluate it
					state = IDLE;
					cc = 0; // no more than one MSP per port and per cycle
				}
			}
			c_state[port] = state;
		}
	}
}

void evaluateCommand(uint8_t c)
{
	uint32_t tmp = 0;

	switch (c)
	{
	// adding this message as a comment will return an error status for MSP_PRIVATE (end of switch), allowing third party tools to distinguish the implementation of this message
	//case MSP_PRIVATE:
	//  headSerialError();tailSerialReply(); // we don't have any custom msp currently, so tell the gui we do not use that
	//  break;
	case MSP_SET_RAW_RC:
		s_struct_w((uint8_t*) &rcSerial, 16);
		rcSerialCount = 50; // 1s transition
		break;
	case MSP_SET_PID:
		mspAck();
		s_struct_w((uint8_t*) &conf.pid[0].P8, 3 * PIDITEMS);
		break;
	case MSP_SET_BOX:
		mspAck();
		s_struct_w((uint8_t*) &conf.activate[0], CHECKBOXITEMS * 2);
		break;
	case MSP_SET_RC_TUNING:
		mspAck();
		s_struct_w((uint8_t*) &conf.rcRate8, 7);
		break;
	case MSP_SET_MISC:
		struct
		{
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t i, j, k, l;
		} set_misc;
		mspAck();
		s_struct_w((uint8_t*) &set_misc, 22);
		conf.minthrottle = set_misc.b;
		conf.mag_declination = set_misc.h;
		break;
	case MSP_MISC:
		struct
		{
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t i, j, k, l;
		} misc;
		misc.a = intPowerTrigger1;
		misc.b = conf.minthrottle;
		misc.c = MAXTHROTTLE;
		misc.d = MINCOMMAND;
		misc.e = 0;
		misc.f = 0;
		misc.g = 0;
		misc.h = conf.mag_declination;
		misc.i = 0;
		misc.j = 0;
		misc.k = 0;
		misc.l = 0;
		s_struct((uint8_t*) &misc, 22);
		break;
	case MSP_SET_HEAD:
		mspAck();
		s_struct_w((uint8_t*) &magHold, 2);
		break;
	case MSP_IDENT:
		struct
		{
			uint8_t v, t, msp_v;
			uint32_t cap;
		} id;
		id.v = VERSION;
		id.t = MULTITYPE;
		id.msp_v = MSP_VERSION;
		id.cap = (0 + BIND_CAPABLE) | DYNBAL << 2 | FLAP << 3 | NAVCAP << 4 | EXTAUX << 5 | ((uint32_t) NAVI_VERSION << 28); //Navi version is stored in the upper four bits;
		s_struct((uint8_t*) &id, 7);
		break;
	case MSP_STATUS:
		struct
		{
			uint16_t cycleTime, i2c_errors_count, sensor;
			uint32_t flag;
			uint8_t set;
		} st;
		st.cycleTime = cycleTime;
		st.i2c_errors_count = i2c_errors_count;
		st.sensor = ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4;
		if (f.ANGLE_MODE)
			tmp |= 1 << BOXANGLE;
		if (f.HORIZON_MODE)
			tmp |= 1 << BOXHORIZON;
		if (f.BARO_MODE)
			tmp |= 1 << BOXBARO;
		if (f.MAG_MODE)
			tmp |= 1 << BOXMAG;
		if (f.ARMED)
			tmp |= 1 << BOXARM;
		st.flag = tmp;
		st.set = global_conf.currentSet;
		s_struct((uint8_t*) &st, 11);
		break;
	case MSP_RAW_IMU:
		s_struct((uint8_t*) &imu, 18);
		break;
	case MSP_SERVO:
		s_struct((uint8_t*) &servo, 16);
		break;
	case MSP_SERVO_CONF:
		s_struct((uint8_t*) &conf.servoConf[0].min, 56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
		break;
	case MSP_SET_SERVO_CONF:
		mspAck();
		s_struct_w((uint8_t*) &conf.servoConf[0].min, 56);
		break;
	case MSP_MOTOR:
		s_struct((uint8_t*) &motor, 16);
		break;
	case MSP_ACC_TRIM:
		headSerialReply(4);
		s_struct_partial((uint8_t*) &conf.angleTrim[PITCH], 2);
		s_struct_partial((uint8_t*) &conf.angleTrim[ROLL], 2);
		tailSerialReply();
		break;
	case MSP_SET_ACC_TRIM:
		mspAck();
		s_struct_w((uint8_t*) &conf.angleTrim[PITCH], 2);
		s_struct_w((uint8_t*) &conf.angleTrim[ROLL], 2);
		break;
	case MSP_RC:
		s_struct((uint8_t*) &rcData, RC_CHANS * 2);
		break;
	case MSP_ATTITUDE:
		s_struct((uint8_t*) &att, 6);
		break;
	case MSP_ALTITUDE:
		s_struct((uint8_t*) &alt, 6);
		break;
	case MSP_ANALOG:
		s_struct((uint8_t*) &analog, 7);
		break;
	case MSP_RC_TUNING:
		s_struct((uint8_t*) &conf.rcRate8, 7);
		break;
	case MSP_PID:
		s_struct((uint8_t*) &conf.pid[0].P8, 3 * PIDITEMS);
		break;
	case MSP_PIDNAMES:
		serializeNames(pidnames);
		break;
	case MSP_BOX:
		s_struct((uint8_t*) &conf.activate[0], 2 * CHECKBOXITEMS);
		break;
	case MSP_BOXNAMES:
		serializeNames(boxnames);
		break;
	case MSP_BOXIDS:
		headSerialReply(CHECKBOXITEMS);
		for (uint8_t i = 0; i < CHECKBOXITEMS; i++)
			serialize8(pgm_read_byte(&(boxids[i])));
		tailSerialReply();
		break;
	case MSP_MOTOR_PINS:
		s_struct((uint8_t*) &PWM_PIN, 8);
		break;
	case MSP_RESET_CONF:
		if (!f.ARMED)
			LoadDefaults();
		mspAck();
		break;
	case MSP_ACC_CALIBRATION:
		if (!f.ARMED)
			calibratingA = 512;
		mspAck();
		break;
	case MSP_MAG_CALIBRATION:
		if (!f.ARMED)
			f.CALIBRATE_MAG = 1;
		mspAck();
		break;
	case MSP_EEPROM_WRITE:
		writeParams(0);
		mspAck();
		break;
	case MSP_DEBUG:
		s_struct((uint8_t*) &debug, 8);
		break;
	default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
		headSerialError();
		tailSerialReply();
		break;
	}
}

void SerialWrite16(uint8_t port, int16_t val)
{
	CURRENTPORT = port;
	serialize16(val);
	UartSendData(port);
}
