#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Sensors.h"

static void Device_Mag_getADC();
static void Baro_init();
static void Mag_init();
static void ACC_init();

// ITG3200 / ITG3205 / ITG3050 / MPU6050 / MPU3050 Gyro LPF setting
#define GYRO_DLPF_CFG   0 // Default settings LPF 256Hz/8000Hz sample

static uint8_t rawADC[6];

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
void i2c_init(void)
{
	I2C_PULLUPS_DISABLE
	TWSR = 0;                           // no prescaler => prescaler = 1
	TWBR = ((F_CPU / 400000) - 16) / 2; // set the I2C clock rate to 400kHz
	TWCR = 1 << TWEN;                   // enable twi module, no interrupt
	i2c_errors_count = 0;
}

void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr)
{
	TWCR = twcr;
	uint8_t count = 255;
	while (!(TWCR & (1 << TWINT)))
	{
		count--;
		if (count == 0)
		{ //we are in a blocking state => we don't insist
			TWCR = 0; //and we force a reset on TWINT register
			i2c_errors_count++;
			break;
		}
	}
}

void i2c_rep_start(uint8_t address)
{
	waitTransmissionI2C((1 << TWINT) | (1 << TWSTA) | (1 << TWEN)); // send REPEAT START condition and wait until transmission completed
	TWDR = address;                                                 // send device address
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));                // wail until transmission completed
}

void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_write(uint8_t data)
{
	TWDR = data; // send data to the previously addressed device
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
}

uint8_t i2c_readAck()
{
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
	return TWDR;
}

uint8_t i2c_readNak()
{
	waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
	uint8_t r = TWDR;
	i2c_stop();
	return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size)
{
	i2c_rep_start(add << 1);       // I2C write direction
	i2c_write(reg);                // register selection
	i2c_rep_start((add << 1) | 1); // I2C read direction
	uint8_t *b = buf;
	while (--size)
		*b++ = i2c_readAck();       // acknowledge all but the final byte
	*b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
	i2c_read_reg_to_buf(add, reg, rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
	i2c_rep_start(add << 1); // I2C write direction
	i2c_write(reg);          // register selection
	i2c_write(val);          // value to write in register
	i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
	uint8_t val;
	i2c_read_reg_to_buf(add, reg, &val, 1);
	return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common()
{
	static int16_t previousGyroADC[3] = { 0, 0, 0 };
	static int32_t g[3];
	uint8_t axis, tilt = 0;

	if (calibratingG > 0)
	{
		for (axis = 0; axis < 3; axis++)
		{
			if (calibratingG == 512)
			{ // Reset g[axis] at start of calibration
				g[axis] = 0;
			}
			g[axis] += imu.gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis] = g[axis] >> 9;
		}
		calibratingG--;
	}

	for (axis = 0; axis < 3; axis++)
	{
		imu.gyroADC[axis] -= gyroZero[axis];
		//anti gyro glitch, limit the variation between two consecutive readings
		imu.gyroADC[axis] = constrain(imu.gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
		previousGyroADC[axis] = imu.gyroADC[axis];
	}
}

// ****************
// ACC common part
// ****************
void ACC_Common()
{
	static int32_t a[3];

	if (calibratingA > 0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA == 511)
				a[axis] = 0;   // Reset a[axis] at start of calibration
			a[axis] += imu.accADC[axis];           // Sum up 512 readings
			global_conf.accZero[axis] = a[axis] >> 9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0)
		{
			global_conf.accZero[YAW] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
			conf.angleTrim[ROLL] = 0;
			conf.angleTrim[PITCH] = 0;
			writeGlobalSet(1); // write accZero in EEPROM
		}
	}
	imu.accADC[ROLL] -= global_conf.accZero[ROLL];
	imu.accADC[PITCH] -= global_conf.accZero[PITCH];
	imu.accADC[YAW] -= global_conf.accZero[YAW];
}

// ************************************************************************************************************
// BARO section
// ************************************************************************************************************
static void Baro_Common()
{
	static int32_t baroHistTab[BARO_TAB_SIZE];
	static uint8_t baroHistIdx;

	uint8_t indexplus1 = (baroHistIdx + 1);
	if (indexplus1 == BARO_TAB_SIZE)
		indexplus1 = 0;
	baroHistTab[baroHistIdx] = baroPressure;
	baroPressureSum += baroHistTab[baroHistIdx];
	baroPressureSum -= baroHistTab[indexplus1];
	baroHistIdx = indexplus1;
}

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct
{
	// sensor registers from the MS561101BA datasheet
	uint16_t c[7];
	uint32_t ut; //uncompensated T
	uint32_t up; //uncompensated P
	uint8_t state;
	uint16_t deadline;
} ms561101ba_ctx;

static void Baro_init()
{
	//reset
	i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
	delay(100);

	//read calibration data
	union
	{
		uint16_t val;
		uint8_t raw[2];
	} data;
	for (uint8_t i = 0; i < 6; i++)
	{
		i2c_rep_start(MS561101BA_ADDRESS << 1);
		i2c_write(0xA2 + 2 * i);
		i2c_rep_start((MS561101BA_ADDRESS << 1) | 1); //I2C read direction => 1
		data.raw[1] = i2c_readAck();  // read a 16 bit register
		data.raw[0] = i2c_readNak();
		ms561101ba_ctx.c[i + 1] = data.val;
	}
}

// read uncompensated temperature or pressure value: send command first
static void i2c_MS561101BA_UT_or_UP_Start(uint8_t reg)
{
	i2c_rep_start(MS561101BA_ADDRESS << 1);     // I2C write direction
	i2c_write(reg);  // register selection
	i2c_stop();
}

static void i2c_MS561101BA_UT_or_UP_Read(uint32_t* val)
{
	union
	{
		uint32_t val;
		uint8_t raw[4];
	} data;
	i2c_rep_start(MS561101BA_ADDRESS << 1);
	i2c_write(0);
	i2c_rep_start((MS561101BA_ADDRESS << 1) | 1);
	data.raw[2] = i2c_readAck();
	data.raw[1] = i2c_readAck();
	data.raw[0] = i2c_readNak();
	*val = data.val;
}

// use float approximation instead of int64_t intermediate values
// does not use 2nd order compensation under -15 deg
static void i2c_MS561101BA_Calculate()
{
	int32_t delt;

	float dT = (int32_t) ms561101ba_ctx.ut - (int32_t)((uint32_t) ms561101ba_ctx.c[5] << 8);
	float off = ((uint32_t) ms561101ba_ctx.c[2] << 16) + ((dT * ms561101ba_ctx.c[4]) / ((uint32_t) 1 << 7));
	float sens = ((uint32_t) ms561101ba_ctx.c[1] << 15) + ((dT * ms561101ba_ctx.c[3]) / ((uint32_t) 1 << 8));
	delt = (dT * ms561101ba_ctx.c[6]) / ((uint32_t) 1 << 23);
	baroTemperature = delt + 2000;
	if (delt < 0)
	{ // temperature lower than 20st.C
		delt *= 5 * delt;
		off -= delt >> 1;
		sens -= delt >> 2;
	}
	baroPressure = (((ms561101ba_ctx.up * sens) / ((uint32_t) 1 << 21)) - off) / ((uint32_t) 1 << 15);
}

//return 0: no data available, no computation ;  1: new value available   or   no new value but computation time
uint8_t Baro_update()
{                          // first UT conversion is started in init procedure
	uint32_t* rawValPointer;
	uint8_t commandRegister;

	if (ms561101ba_ctx.state == 2)
	{               // a third state is introduced here to isolate calculate() and smooth timecycle spike
		ms561101ba_ctx.state = 0;
		i2c_MS561101BA_Calculate();
		return 1;
	}
	if ((int16_t)(currentTime - ms561101ba_ctx.deadline) < 0)
		return 0; // the initial timer is not initialized, but in any case, no more than 65ms to wait.
	ms561101ba_ctx.deadline = currentTime + 10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms
	if (ms561101ba_ctx.state == 0)
	{
		Baro_Common();                              // moved here for less timecycle spike, goes after i2c_MS561101BA_Calculate
		rawValPointer = &ms561101ba_ctx.ut;
		commandRegister = MS561101BA_PRESSURE + OSR;
	}
	else
	{
		rawValPointer = &ms561101ba_ctx.up;
		commandRegister = MS561101BA_TEMPERATURE + OSR;
	}
	ms561101ba_ctx.state++;
	i2c_MS561101BA_UT_or_UP_Read(rawValPointer); // get the 24bit resulting from a UP of UT command request. Nothing interresting for the first cycle because we don't initiate a command in Baro_init()
	i2c_MS561101BA_UT_or_UP_Start(commandRegister);  // send the next command to get UP or UT value after at least 8.5ms
	return 1;
}

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
static float magGain[3] = { 1.0, 1.0, 1.0 };  // gain for each axis, populated at sensor init

uint8_t Mag_getADC()
{ // return 1 when news values are available, 0 otherwise
	static uint32_t t, tCal = 0;
	static int16_t magZeroTempMin[3], magZeroTempMax[3];
	uint8_t axis;

	if (currentTime < t)
		return 0; //each read is spaced by 100ms
	t = currentTime + 100000;
	Device_Mag_getADC();

	for (axis = 0; axis < 3; axis++)
	{
		imu.magADC[axis] = imu.magADC[axis] * magGain[axis];
		if (!f.CALIBRATE_MAG)
			imu.magADC[axis] -= global_conf.magZero[axis];
	}

	if (f.CALIBRATE_MAG)
	{
		if (tCal == 0) // init mag calibration
			tCal = t;
		if ((t - tCal) < 30000000)
		{ // 30s: you have 30s to turn the multi in all directions
			LEDPIN_TOGGLE
			;
			for (axis = 0; axis < 3; axis++)
			{
				if (tCal == t)
				{ // it happens only in the first step, initialize the zero
					magZeroTempMin[axis] = imu.magADC[axis];
					magZeroTempMax[axis] = imu.magADC[axis];
				}
				if (imu.magADC[axis] < magZeroTempMin[axis])
				{
					magZeroTempMin[axis] = imu.magADC[axis];
					SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
				}
				if (imu.magADC[axis] > magZeroTempMax[axis])
				{
					magZeroTempMax[axis] = imu.magADC[axis];
					SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
				}
				global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
			}
		}
		else
		{
			f.CALIBRATE_MAG = 0;
			tCal = 0;
			writeGlobalSet(1);
		}
	}

	return 1;
}

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

static int32_t xyz_total[3] = { 0, 0, 0 };  // 32 bit totals so they won't overflow.

static void getADC()
{
	i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
	MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]), ((rawADC[4] << 8) | rawADC[5]), ((rawADC[2] << 8) | rawADC[3]));
}

static uint8_t bias_collect(uint8_t bias)
{
	int16_t abs_magADC;

	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, bias);            // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
	for (uint8_t i = 0; i < 10; i++)
	{                               // Collect 10 samples
		i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
		delay(100);
		getADC();                                                  // Get the raw values in case the scales have already been changed.
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			abs_magADC = abs(imu.magADC[axis]);
			xyz_total[axis] += abs_magADC;                            // Since the measurements are noisy, they should be averaged rather than taking the max.
			if ((int16_t)(1 << 12) < abs_magADC)
				return false;         // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	return true;
}

static void Mag_init()
{
	bool bret = true;                // Error indicator

	// Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
	// The new gain setting is effective from the second measurement and on.
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
	delay(100);
	getADC();  //Get one sample, and discard it

	if (!bias_collect(0x010 + HMC_POS_BIAS))
		bret = false;
	if (!bias_collect(0x010 + HMC_NEG_BIAS))
		bret = false;

	if (bret) // only if no saturation detected, compute the gain. otherwise, the default 1.0 is used
		for (uint8_t axis = 0; axis < 3; axis++)
			magGain[axis] = 820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[axis];  // note: xyz_total[axis] is always positive

		// leave test mode
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 0x00); //Mode register             -- 000000 00    continuous Conversion Mode
	delay(100);
}

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board

static void Gyro_init()
{
	i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	delay(50);
	i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	i2c_writeReg(MPU6050_ADDRESS, 0x1A, GYRO_DLPF_CFG);    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
	// enable I2C bypass for AUX I2C
	i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);        //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
}

void Gyro_getADC()
{
	i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
	GYRO_ORIENTATION(((rawADC[0]<<8) | rawADC[1])>>2, // range: +/- 8192; +/- 2000 deg/sec
			((rawADC[2]<<8) | rawADC[3])>>2, ((rawADC[4]<<8) | rawADC[5])>>2);
	GYRO_Common();
}

static void ACC_init()
{
	i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	//note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
	//confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

	//at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
	//now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
	i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
	i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);        //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
	i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
	i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80 | MAG_ADDRESS);             //I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
	i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);             //I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
	i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
}

void ACC_getADC()
{
	i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
	ACC_ORIENTATION(((rawADC[0]<<8) | rawADC[1])>>3, ((rawADC[2]<<8) | rawADC[3])>>3, ((rawADC[4] << 8) | rawADC[5]) >> 3);
	ACC_Common();
}

static void Device_Mag_getADC()
{
	i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
	MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]), ((rawADC[4] << 8) | rawADC[5]), ((rawADC[2] << 8) | rawADC[3]));
}

void initS()
{
	i2c_init();
	if (GYRO)
		Gyro_init();
	if (BARO)
		Baro_init();
	if (MAG)
		Mag_init();
	if (ACC)
		ACC_init();
}

void initSensors()
{
	uint8_t c = 5;
	while (c)
	{ // We try several times to init all sensors without any i2c errors. An I2C error at this stage might results in a wrong sensor settings
		c--;
		initS();
		if (i2c_errors_count == 0)
			break; // no error during init => init ok
	}
}

