#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

void getEstimatedAttitude();

void computeIMU()
{
	uint8_t axis;
	static int16_t gyroADCprevious[3] = { 0, 0, 0 };
	static int16_t gyroADCinter[3];
	uint16_t timeInterleave = 0;

	ACC_getADC();
	getEstimatedAttitude();
	Gyro_getADC();
	for (axis = 0; axis < 3; axis++)
		gyroADCinter[axis] = imu.gyroADC[axis];
	timeInterleave = micros();
	annexCode();
	uint8_t t = 0;
	while ((int16_t)(micros() - timeInterleave) < 650)
		t = 1;
	Gyro_getADC();
	for (axis = 0; axis < 3; axis++) {
		gyroADCinter[axis] = imu.gyroADC[axis] + gyroADCinter[axis];
		imu.gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
		gyroADCprevious[axis] = gyroADCinter[axis] >> 1;
		if (!1)
			imu.accADC[axis] = 0;
	}
}

typedef struct
{
	int32_t X, Y, Z;
} t_int32_t_vector_def;

typedef struct
{
	uint16_t r26;
	int16_t X;
	uint16_t r28;
	int16_t Y;
	uint16_t r30;
	int16_t Z;
} t_int16_t_vector_def;

typedef union
{
	int32_t A32[3];
	t_int32_t_vector_def V32;
	int16_t A16[6];
	t_int16_t_vector_def V16;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x)
{
	float z = y;
	int16_t a;
	uint8_t c;

	c = ((y) > 0 ? (y) : -(y)) < ((x) > 0 ? (x) : -(x));
	if (c)
		z = z / x;
	else
		z = x / z;
	a = 2046.43 * (z / (3.5714 + z * z));
	if (c) {
		if (x < 0) {
			if (y < 0)
				a -= 1800;
			else
				a += 1800;
		}
	}
	else {
		a = 900 - a;
		if (y < 0)
			a -= 1800;
	}
	return a;
}

float InvSqrt(float x)
{
	union
	{
		int32_t i;
		float f;
	} conv;

	conv.f = x;
	conv.i = 0x5f1ffff9 - (conv.i >> 1);

	return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}

int32_t __attribute__ ((noinline)) mul(int16_t a, int16_t b)
{
	int32_t r;
	asm volatile ( "clr r26 \n\t" "mul %A1, %A2 \n\t" "movw %A0, r0 \n\t" "muls %B1, %B2 \n\t" "movw %C0, r0 \n\t" "mulsu %B2, %A1 \n\t" "sbc %D0, r26 \n\t" "add %B0, r0 \n\t" "adc %C0, r1 \n\t" "adc %D0, r26 \n\t" "mulsu %B1, %A2 \n\t" "sbc %D0, r26 \n\t" "add %B0, r0 \n\t" "adc %C0, r1 \n\t" "adc %D0, r26 \n\t" "clr r1 \n\t" : "=&r" (r) : "a" (a), "a" (b) : "r26" );
	return r;
}

void rotateV32(t_int32_t_vector *v, int16_t* delta)
{
	int16_t X = v->V16.X;
	int16_t Y = v->V16.Y;
	int16_t Z = v->V16.Z;

	v->V32.Z -= mul(delta[ROLL], X) + mul(delta[PITCH], Y);
	v->V32.X += mul(delta[ROLL], Z) - mul(delta[YAW], Y);
	v->V32.Y += mul(delta[PITCH], Z) + mul(delta[YAW], X);
}

static int16_t accZ = 0;

void getEstimatedAttitude()
{
	uint8_t axis;
	int32_t accMag = 0;
	float scale;
	int16_t deltaGyroAngle16[3];
	static t_int32_t_vector EstG = { 0, 0, (int32_t) 512 << 16 };
	static t_int32_t_vector EstM;
	static uint32_t LPFAcc[3];
	float invG;
	static int16_t accZoffset = 0;
	int32_t accZ_tmp = 0;
	static uint16_t previousT;
	uint16_t currentT = micros();

	scale = (currentT - previousT) * ((4 / 16.4 * 3.1415926535897932384626433832795 / 180.0 / 1000000.0) * 65536);
	previousT = currentT;
	for (axis = 0; axis < 3; axis++) {
		imu.accSmooth[axis] = LPFAcc[axis] >> 4;
		LPFAcc[axis] += imu.accADC[axis] - imu.accSmooth[axis];
		accMag += mul(imu.accSmooth[axis], imu.accSmooth[axis]);
		deltaGyroAngle16[axis] = imu.gyroADC[axis] * scale;
	}
	rotateV32(&EstG, deltaGyroAngle16);
	rotateV32(&EstM, deltaGyroAngle16);
	for (axis = 0; axis < 3; axis++) {
		if ((int16_t)(0.85 * 512 * 512 / 256) < (int16_t)(accMag >> 8) && (int16_t)(accMag >> 8) < (int16_t)(1.15 * 512 * 512 / 256))
			EstG.A32[axis] += (int32_t)(imu.accSmooth[axis] - EstG.A16[2 * axis + 1]) << (16 - 10);
		accZ_tmp += mul(imu.accSmooth[axis], EstG.A16[2 * axis + 1]);
		EstM.A32[axis] += (int32_t)(imu.magADC[axis] - EstM.A16[2 * axis + 1]) << (16 - 8);
	}
	if (EstG.V16.Z > (int16_t)(512 * 0.90631))
		f.SMALL_ANGLES_25 = 1;
	else
		f.SMALL_ANGLES_25 = 0;
	int32_t sqGX_sqGZ = mul(EstG.V16.X, EstG.V16.X) + mul(EstG.V16.Z, EstG.V16.Z);
	invG = InvSqrt(sqGX_sqGZ + mul(EstG.V16.Y, EstG.V16.Y));
	att.angle[ROLL] = _atan2(EstG.V16.X, EstG.V16.Z);
	att.angle[PITCH] = _atan2(EstG.V16.Y, InvSqrt(sqGX_sqGZ) * sqGX_sqGZ);
	att.heading = _atan2(mul(EstM.V16.Z, EstG.V16.X) - mul(EstM.V16.X, EstG.V16.Z), (EstM.V16.Y * sqGX_sqGZ - (mul(EstM.V16.X, EstG.V16.X) + mul(EstM.V16.Z, EstG.V16.Z)) * EstG.V16.Y) * invG);
	att.heading += conf.mag_declination;
	att.heading /= 10;
	accZ = accZ_tmp * invG;
	if (!f.ARMED) {
		accZoffset -= accZoffset >> 3;
		accZoffset += accZ;
	}
	accZ -= accZoffset >> 3;
}

uint8_t getEstimatedAltitude()
{
	int32_t BaroAlt;
	static float baroGroundTemperatureScale, logBaroGroundPressureSum;
	static float vel = 0.0f;
	static uint16_t previousT;
	uint16_t currentT = micros();
	uint16_t dTime;

	dTime = currentT - previousT;
	if (dTime < 25000)
		return 0;
	previousT = currentT;
	if (calibratingB > 0) {
		logBaroGroundPressureSum = log(baroPressureSum);
		baroGroundTemperatureScale = ((int32_t) baroTemperature + 27315) * (2 * 29.271267f);
		calibratingB--;
	}
	BaroAlt = (logBaroGroundPressureSum - log(baroPressureSum)) * baroGroundTemperatureScale;
	alt.EstAlt = (alt.EstAlt * 6 + BaroAlt) >> 3;
	int16_t error16 = ((AltHold - alt.EstAlt) < (-300) ? (-300) : ((AltHold - alt.EstAlt) > (300) ? (300) : (AltHold - alt.EstAlt)));
	if (((error16) > 0 ? (error16) : -(error16)) < 10)
		error16 = 0;
	else if (error16 > 0)
		error16 -= 10;
	else if (error16 < 0)
		error16 += 10;
	BaroPID = (((conf.pid[PIDALT].P8 * error16 >> 7)) < (-150) ? (-150) : (((conf.pid[PIDALT].P8 * error16 >> 7)) > (+150) ? (+150) : ((conf.pid[PIDALT].P8 * error16 >> 7))));
	errorAltitudeI += conf.pid[PIDALT].I8 * error16 >> 6;
	errorAltitudeI = ((errorAltitudeI) < (-30000) ? (-30000) : ((errorAltitudeI) > (30000) ? (30000) : (errorAltitudeI)));
	BaroPID += errorAltitudeI >> 9;
	if (((accZ) > 0 ? (accZ) : -(accZ)) < (512 >> 5))
		accZ = 0;
	else if (accZ > 0)
		accZ -= (512 >> 5);
	else if (accZ < 0)
		accZ += (512 >> 5);
	static int32_t lastBaroAlt;
	int16_t baroVel = mul((alt.EstAlt - lastBaroAlt), (1000000 / 25000));
	lastBaroAlt = alt.EstAlt;
	baroVel = ((baroVel) < (-300) ? (-300) : ((baroVel) > (300) ? (300) : (baroVel)));
	if (((baroVel) > 0 ? (baroVel) : -(baroVel)) < 10)
		baroVel = 0;
	else if (baroVel > 0)
		baroVel -= 10;
	else if (baroVel < 0)
		baroVel += 10;
	vel += accZ * (9.80665f / 10000.0f / 512) * dTime;
	vel = vel * 0.985f + baroVel * 0.015f;
	alt.vario = vel;
	if (((alt.vario) > 0 ? (alt.vario) : -(alt.vario)) < 5)
		alt.vario = 0;
	else if (alt.vario > 0)
		alt.vario -= 5;
	else if (alt.vario < 0)
		alt.vario += 5;
	BaroPID -= ((conf.pid[PIDALT].D8 * alt.vario >> 4) < (-150) ? (-150) : ((conf.pid[PIDALT].D8 * alt.vario >> 4) > (150) ? (150) : (conf.pid[PIDALT].D8 * alt.vario >> 4)));
	return 1;
}
