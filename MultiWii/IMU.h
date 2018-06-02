#ifndef IMU_H_
#define IMU_H_

#define BARO_TAB_SIZE   21

uint8_t getEstimatedAltitude();
void computeIMU();
int32_t mul(int16_t a, int16_t b);

#endif /* IMU_H_ */
