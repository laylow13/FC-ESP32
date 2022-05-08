//
// Created by Lay on 3/20/2022.
//

#ifndef SOURCE_IMU_H
#define SOURCE_IMU_H
#include "MPU6050.h"
#include "ICM20689.h"
#include "ICM42605.h"
#include "BMI088.h"
#include "Wire.h"
#ifndef ESPFC
#ifndef IMU_I2C_SWAP_SEQUENCE
#define IMU_I2C_SDA 17  //need to be edited!
#define IMU_I2C_SCL 16
#else
#define IMU_I2C_SDA 16
#define IMU_I2C_SCL 17
#endif
#else
#define IMU_I2C_SDA 26  //need to be edited!
#define IMU_I2C_SCL 27
#endif
#define WINDOW_NUM 5
class IMU
{
private:
    MPU6050 mpu6050;//兼容mpu9250，6500 
    ICM20689 icm20689;
    ICM42605 icm42605;
    Bmi088 bmi088;
    float movingWin[6][WINDOW_NUM]={{0}};

public:
    double ax, ay, az;
    double gx, gy, gz;
    float gx_error=0,gy_error=0,gz_error=0,
            ax_error=0,ay_error=0,az_error=0;
    float gx_mWfilted,gy_mWfilted,gz_mWfilted,
            ax_mWfilted,ay_mWfilted,az_mWfilted;
    enum IMU_TYPE
    {
        IMU_MPU6050,
        IMU_MPU9250,
        IMU_MPU6500,
        IMU_ICM20689,
        IMU_ICM42605,
        IMU_BMI088
    };
    uint8_t imu_type;

public:
    void init();
    void getGyroStaticError();
    int8_t update();
    void movingWindowFil();
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
