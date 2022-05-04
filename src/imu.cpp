//
// Created by Lay on 3/20/2022.
//

#include "imu.h"
#include "Arduino.h"

#define STATIC_ERROR_SAMPLE_TIME 40000
#define IMU_SAMPLE_PERIOD 5 //5ms one time
#define GAINBtw50hz   (3.450423889e+02f)
#define GAINBtw30Hz   1.278738361e+02f

IMU imu;
Filter6axisTypeDef Filters;

void IMU::init()
{
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    Wire.setClock(400000);
    //没有检验whoami，为简易的适配其他IMU
    //TO DO:检验whoami
    uint8_t deviceId = imu.getDeviceID();
    Serial.println(deviceId);
    Serial.println("Imu find!");
    imu.initialize();
}

int8_t IMU::update()
{   int8_t res;
    res= imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    return res;

}

int16_t IMU::getAccelX()
{
    return ax;

}
int16_t IMU::getAccelY()
{
    return ay;
}

int16_t IMU::getAccelZ()
{
    return az;
}

int16_t IMU::getGyroX()
{
    return gx;
}

int16_t IMU::getGyroY()
{
    return gy;
}

int16_t IMU::getGyroZ()
{
    return gz;
}
void IMU::getGyroStaticError(void)
{
    int i=0;
    int gyrox_add=0;
    int gyroy_add=0;
    int gyroz_add=0;
    for(i=0;i<STATIC_ERROR_SAMPLE_TIME;i++)
    {
        this->update();
        delay(IMU_SAMPLE_PERIOD);
        gyrox_add+=gx;
        gyroy_add+=gy;
        gyroz_add+=gz;
    }
    gx_error = (float)(gyrox_add/STATIC_ERROR_SAMPLE_TIME);
    gy_error = (float)(gyroy_add/STATIC_ERROR_SAMPLE_TIME);
    gz_error = (float)(gyroz_add/STATIC_ERROR_SAMPLE_TIME);
}

//butterworth filter
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF)
{
    pLPF->xv[0] = pLPF->xv[1]; pLPF->xv[1] = pLPF->xv[2]; pLPF->xv[2] = pLPF->xv[3];
    pLPF->xv[3] = pLPF->input / GAINBtw50hz;
    pLPF->yv[0] = pLPF->yv[1]; pLPF->yv[1] = pLPF->yv[2]; pLPF->yv[2] = pLPF->yv[3];
    pLPF->yv[3] =   (pLPF->xv[0] + pLPF->xv[3]) + 3 * (pLPF->xv[1] + pLPF->xv[2])
                    + (  0.5320753683f * pLPF->yv[0]) + ( -1.9293556691f * pLPF->yv[1])
                    + (  2.3740947437f * pLPF->yv[2]);

    pLPF->output = pLPF->yv[3];
}

void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF)
{
    pLPF->xv[0] = pLPF->xv[1];
    pLPF->xv[1] = pLPF->xv[2];
    pLPF->xv[2] = pLPF->input / GAINBtw30Hz;
    pLPF->yv[0] = pLPF->yv[1];
    pLPF->yv[1] = pLPF->yv[2];
    pLPF->yv[2] = (pLPF->xv[0] + pLPF->xv[2]) + 2 * pLPF->xv[1]
                  + ( -0.7660066009f * pLPF->yv[0]) + (  1.7347257688f * pLPF->yv[1]);
    pLPF->output = pLPF->yv[2];
}
void LPFUpdate6axis(float gyroGx,float gyroGy,float gyroGz,float ax,float ay,float az)
{
    Filters.GyroxLPF.input = gyroGx;
    Filters.GyroyLPF.input = gyroGy;
    Filters.GyrozLPF.input = gyroGz;
    Butterworth50HzLPF(&Filters.GyroxLPF);
    Butterworth50HzLPF(&Filters.GyroyLPF);
    Butterworth50HzLPF(&Filters.GyrozLPF);

    Filters.AccxLPF.input = ax;
    Filters.AccyLPF.input = ay;
    Filters.AcczLPF.input = az;
    Butterworth30HzLPF(&Filters.AccxLPF);
    Butterworth30HzLPF(&Filters.AccyLPF);
    Butterworth30HzLPF(&Filters.AcczLPF);
}