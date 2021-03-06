//
// Created by Lay on 3/20/2022.
//

#ifndef SOURCE_IMU_H
#define SOURCE_IMU_H
#include "MPU6050.h"

#define IMU_I2C_SDA 17  //need to be edited!
#define IMU_I2C_SCL 16


class IMU
{
private:
    MPU6050 imu;


public:
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float gx_error,gy_error,gz_error;

public:
    void init();
    void getGyroStaticError();
    void update();

    int16_t getAccelX();
    int16_t getAccelY();
    int16_t getAccelZ();

    int16_t getGyroX();
    int16_t getGyroY();
    int16_t getGyroZ();

};
extern IMU imu;

typedef struct
{
    float xv[3];
    float yv[3];
    float input;
    float output;
}Bw30HzLPFTypeDef;

typedef struct
{
    float xv[4];
    float yv[4];
    float input;
    float output;
}Bw50HzLPFTypeDef;

typedef struct
{
    Bw50HzLPFTypeDef GyroxLPF;
    Bw50HzLPFTypeDef GyroyLPF;
    Bw50HzLPFTypeDef GyrozLPF;
    Bw30HzLPFTypeDef AccxLPF;
    Bw30HzLPFTypeDef AccyLPF;
    Bw30HzLPFTypeDef AcczLPF;
}Filter6axisTypeDef;

extern Filter6axisTypeDef Filters;
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF);
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF);
void LPFUpdate6axis(float gyroGx,float gyroGy,float gyroGz,float ax,float ay,float az);


#endif //SOURCE_IMU_H
