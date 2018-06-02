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
static uint8_t rawADC[6];

void i2c_init(void)
{

	(*(volatile uint8_t *) ((0x08) + 0x20)) &= ~(1 << 4);
	(*(volatile uint8_t *) ((0x08) + 0x20)) &= ~(1 << 5);

	(*(volatile uint8_t *) (0xB9)) = 0;
	(*(volatile uint8_t *) (0xB8)) = ((16000000L / 400000) - 16) / 2;
	(*(volatile uint8_t *) (0xBC)) = 1 << 2;
	i2c_errors_count = 0;
}

void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr)
{
	(*(volatile uint8_t *) (0xBC)) = twcr;
	uint8_t count = 255;
	while (!((*(volatile uint8_t *) (0xBC)) & (1 << 7)))
	{
		count--;
		if (count == 0)
		{
			(*(volatile uint8_t *) (0xBC)) = 0;

			i2c_errors_count++;
			break;
		}
	}
}

void i2c_rep_start(uint8_t address)
{
	waitTransmissionI2C((1 << 7) | (1 << 5) | (1 << 2));
	(*(volatile uint8_t *) (0xBB)) = address;
	waitTransmissionI2C((1 << 7) | (1 << 2));
}

void i2c_stop(void)
{
	(*(volatile uint8_t *) (0xBC)) = (1 << 7) | (1 << 2) | (1 << 4);

}

void i2c_write(uint8_t data)
{
	(*(volatile uint8_t *) (0xBB)) = data;
	waitTransmissionI2C((1 << 7) | (1 << 2));
}

uint8_t i2c_readAck()
{
	waitTransmissionI2C((1 << 7) | (1 << 2) | (1 << 6));
	return (*(volatile uint8_t *) (0xBB));
}

uint8_t i2c_readNak()
{
	waitTransmissionI2C((1 << 7) | (1 << 2));
	uint8_t r = (*(volatile uint8_t *) (0xBB));
	i2c_stop();
	return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size)
{
	i2c_rep_start(add << 1);
	i2c_write(reg);
	i2c_rep_start((add << 1) | 1);
	uint8_t *b = buf;
	while (--size)
		*b++ = i2c_readAck();
	*b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
	i2c_read_reg_to_buf(add, reg, rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
	i2c_rep_start(add << 1);
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
	uint8_t val;
	i2c_read_reg_to_buf(add, reg, &val, 1);
	return val;
}

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
			{
				g[axis] = 0;

			}
			g[axis] += imu.gyroADC[axis];
			gyroZero[axis] = g[axis] >> 9;
		}

		calibratingG--;
	}

	for (axis = 0; axis < 3; axis++)
	{
		imu.gyroADC[axis] -= gyroZero[axis];

		imu.gyroADC[axis] = (
				(imu.gyroADC[axis]) < (previousGyroADC[axis] - 800) ?
						(previousGyroADC[axis] - 800) : ((imu.gyroADC[axis]) > (previousGyroADC[axis] + 800) ? (previousGyroADC[axis] + 800) : (imu.gyroADC[axis])));

		previousGyroADC[axis] = imu.gyroADC[axis];
	}
}

void ACC_Common()
{
	static int32_t a[3];
	if (calibratingA > 0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA == 511)
				a[axis] = 0;
			a[axis] += imu.accADC[axis];
			global_conf.accZero[axis] = a[axis] >> 9;
		}
		if (calibratingA == 0)
		{
			global_conf.accZero[YAW] -= 512;
			conf.angleTrim[ROLL] = 0;
			conf.angleTrim[PITCH] = 0;
			writeGlobalSet(1);
		}
	}
	imu.accADC[ROLL] -= global_conf.accZero[ROLL];
	imu.accADC[PITCH] -= global_conf.accZero[PITCH];
	imu.accADC[YAW] -= global_conf.accZero[YAW];
}

static void Baro_Common()
{
	static int32_t baroHistTab[21];
	static uint8_t baroHistIdx;

	uint8_t indexplus1 = (baroHistIdx + 1);
	if (indexplus1 == 21)
		indexplus1 = 0;
	baroHistTab[baroHistIdx] = baroPressure;
	baroPressureSum += baroHistTab[baroHistIdx];
	baroPressureSum -= baroHistTab[indexplus1];
	baroHistIdx = indexplus1;
}

static struct
{

	uint16_t c[7];
	uint32_t ut;
	uint32_t up;
	uint8_t state;
	uint16_t deadline;
} ms561101ba_ctx;

static void Baro_init()
{

	i2c_writeReg(0x77, 0x1E, 0);
	delay(100);

	union
	{
		uint16_t val;
		uint8_t raw[2];
	} data;
	for (uint8_t i = 0; i < 6; i++)
	{
		i2c_rep_start(0x77 << 1);
		i2c_write(0xA2 + 2 * i);
		i2c_rep_start((0x77 << 1) | 1);
		data.raw[1] = i2c_readAck();
		data.raw[0] = i2c_readNak();
		ms561101ba_ctx.c[i + 1] = data.val;
	}
}

static void i2c_MS561101BA_UT_or_UP_Start(uint8_t reg)
{
	i2c_rep_start(0x77 << 1);
	i2c_write(reg);
	i2c_stop();
}

static void i2c_MS561101BA_UT_or_UP_Read(uint32_t* val)
{
	union
	{
		uint32_t val;
		uint8_t raw[4];
	} data;
	i2c_rep_start(0x77 << 1);
	i2c_write(0);
	i2c_rep_start((0x77 << 1) | 1);
	data.raw[2] = i2c_readAck();
	data.raw[1] = i2c_readAck();
	data.raw[0] = i2c_readNak();
	*val = data.val;
}

static void i2c_MS561101BA_Calculate()
{
	int32_t delt;

	float dT = (int32_t) ms561101ba_ctx.ut - (int32_t)((uint32_t) ms561101ba_ctx.c[5] << 8);
	float off = ((uint32_t) ms561101ba_ctx.c[2] << 16) + ((dT * ms561101ba_ctx.c[4]) / ((uint32_t) 1 << 7));
	float sens = ((uint32_t) ms561101ba_ctx.c[1] << 15) + ((dT * ms561101ba_ctx.c[3]) / ((uint32_t) 1 << 8));
	delt = (dT * ms561101ba_ctx.c[6]) / ((uint32_t) 1 << 23);
	baroTemperature = delt + 2000;
	if (delt < 0)
	{
		delt *= 5 * delt;
		off -= delt >> 1;
		sens -= delt >> 2;
	}
	baroPressure = (((ms561101ba_ctx.up * sens) / ((uint32_t) 1 << 21)) - off) / ((uint32_t) 1 << 15);
}

uint8_t Baro_update()
{
	uint32_t* rawValPointer;
	uint8_t commandRegister;

	if (ms561101ba_ctx.state == 2)
	{
		ms561101ba_ctx.state = 0;
		i2c_MS561101BA_Calculate();
		return 1;
	}
	if ((int16_t)(currentTime - ms561101ba_ctx.deadline) < 0)
		return 0;
	ms561101ba_ctx.deadline = currentTime + 10000;
	if (ms561101ba_ctx.state == 0)
	{
		Baro_Common();
		rawValPointer = &ms561101ba_ctx.ut;
		commandRegister = 0x40 + 0x08;
	}
	else
	{
		rawValPointer = &ms561101ba_ctx.up;
		commandRegister = 0x50 + 0x08;
	}
	ms561101ba_ctx.state++;
	i2c_MS561101BA_UT_or_UP_Read(rawValPointer);
	i2c_MS561101BA_UT_or_UP_Start(commandRegister);
	return 1;
}

static float magGain[3] = { 1.0, 1.0, 1.0 };

uint8_t Mag_getADC()
{
	static uint32_t t, tCal = 0;
	static int16_t magZeroTempMin[3], magZeroTempMax[3];
	uint8_t axis;

	if (currentTime < t)
		return 0;
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
		if (tCal == 0)
			tCal = t;
		if ((t - tCal) < 30000000)
		{
			(*(volatile uint8_t *) ((0x03) + 0x20)) |= 1 << 5;
			;
			for (axis = 0; axis < 3; axis++)
			{
				if (tCal == t)
				{
					magZeroTempMin[axis] = imu.magADC[axis];
					magZeroTempMax[axis] = imu.magADC[axis];
				}
				if (imu.magADC[axis] < magZeroTempMin[axis])
				{
					magZeroTempMin[axis] = imu.magADC[axis];
					alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_TOGGLE_1;
				}
				if (imu.magADC[axis] > magZeroTempMax[axis])
				{
					magZeroTempMax[axis] = imu.magADC[axis];
					alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_TOGGLE_1;
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

static int32_t xyz_total[3] = { 0, 0, 0 };

static void getADC()
{
	i2c_getSixRawADC(0x1E, 0x03);
	{
		imu.magADC[ROLL] = ((rawADC[0] << 8) | rawADC[1]);
		imu.magADC[PITCH] = ((rawADC[4] << 8) | rawADC[5]);
		imu.magADC[YAW] = -((rawADC[2] << 8) | rawADC[3]);
	}
}

static uint8_t bias_collect(uint8_t bias)
{
	int16_t abs_magADC;

	i2c_writeReg(0x1E, 0, bias);
	for (uint8_t i = 0; i < 10; i++)
	{
		i2c_writeReg(0x1E, 2, 1);
		delay(100);
		getADC();
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			abs_magADC = ((imu.magADC[axis]) > 0 ? (imu.magADC[axis]) : -(imu.magADC[axis]));
			xyz_total[axis] += abs_magADC;
			if ((int16_t)(1 << 12) < abs_magADC)
				return false;
		}
	}
	return true;
}

static void Mag_init()
{
	bool bret = true;

	i2c_writeReg(0x1E, 1, 2 << 5);
	i2c_writeReg(0x1E, 2, 1);
	delay(100);
	getADC();

	if (!bias_collect(0x010 + 1))
		bret = false;
	if (!bias_collect(0x010 + 2))
		bret = false;

	if (bret)
		for (uint8_t axis = 0; axis < 3; axis++)
			magGain[axis] = 820.0 * (+1.16) * 2.0 * 10.0 / xyz_total[axis];

	i2c_writeReg(0x1E, 0, 0x70);
	i2c_writeReg(0x1E, 1, 0x20);
	i2c_writeReg(0x1E, 2, 0x00);
	delay(100);
}

static void Gyro_init()
{
	i2c_writeReg(0x68, 0x6B, 0x80);
	delay(50);
	i2c_writeReg(0x68, 0x6B, 0x03);
	i2c_writeReg(0x68, 0x1A, 0);
	i2c_writeReg(0x68, 0x1B, 0x18);

	i2c_writeReg(0x68, 0x37, 0x02);
}

void Gyro_getADC()
{
	i2c_getSixRawADC(0x68, 0x43);
	{
		imu.gyroADC[ROLL] = ((rawADC[2] << 8) | rawADC[3]) >> 2;
		imu.gyroADC[PITCH] = -((rawADC[0] << 8) | rawADC[1]) >> 2;
		imu.gyroADC[YAW] = -((rawADC[4] << 8) | rawADC[5]) >> 2;
	}

	GYRO_Common();
}

static void ACC_init()
{
	i2c_writeReg(0x68, 0x1C, 0x10);

	i2c_writeReg(0x68, 0x6A, 0b00100000);
	i2c_writeReg(0x68, 0x37, 0x00);
	i2c_writeReg(0x68, 0x24, 0x0D);
	i2c_writeReg(0x68, 0x25, 0x80 | 0x1E);
	i2c_writeReg(0x68, 0x26, 0x03);
	i2c_writeReg(0x68, 0x27, 0x86);
}

void ACC_getADC()
{
	i2c_getSixRawADC(0x68, 0x3B);
	{
		imu.accADC[ROLL] = -((rawADC[0] << 8) | rawADC[1]) >> 3;
		imu.accADC[PITCH] = -((rawADC[2] << 8) | rawADC[3]) >> 3;
		imu.accADC[YAW] = ((rawADC[4] << 8) | rawADC[5]) >> 3;
	}

	ACC_Common();
}

static void Device_Mag_getADC()
{
	i2c_getSixRawADC(0x68, 0x49);

	{
		imu.magADC[ROLL] = ((rawADC[0] << 8) | rawADC[1]);
		imu.magADC[PITCH] = ((rawADC[4] << 8) | rawADC[5]);
		imu.magADC[YAW] = -((rawADC[2] << 8) | rawADC[3]);
	}
}

void initS()
{
	i2c_init();
	if (1)
		Gyro_init();
	if (1)
		Baro_init();
	if (1)
		Mag_init();
	if (1)
		ACC_init();
}

void initSensors()
{
	uint8_t c = 5;

	while (c)
	{
		c--;
		initS();
		if (i2c_errors_count == 0)
			break;
	}
}

