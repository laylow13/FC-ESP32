//
// Created by Lay on 3/20/2022.
//

#include "imu.h"
#include "Arduino.h"

#define STATIC_ERROR_SAMPLE_NUM 2000
#define GAINBtw50hz (3.450423889e+02f)
#define GAINBtw30Hz 1.278738361e+02f

IMU imu;
Filter6axisTypeDef Filters;

void IMU::init() {
    imu_type = IMU_MPU6500;
    uint8_t deviceId;
    int sta;
    // bmi088 use SPI
    if (imu_type != IMU_BMI088) {
        Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
        Wire.setClock(400000);
    }
    switch (imu_type) {
        case IMU_MPU6050:
            deviceId = mpu6050.getDeviceID();
            Serial.println("IMU type: MPU6050");
            Serial.print("Imu Device ID:");
            Serial.println(deviceId, HEX);
            mpu6050.initialize();
            break;
        case IMU_MPU6500:
            deviceId = mpu6050.getDeviceID();
            Serial.println("IMU type: MPU6500");
            Serial.print("Imu Device ID:");
            Serial.println(deviceId, HEX);
            mpu6050.initialize();
            break;
        case IMU_MPU9250:
            deviceId = mpu6050.getDeviceID();
            Serial.println("IMU type: MPU9250");
            Serial.print("Imu Device ID:");
            Serial.println(deviceId, HEX);
            mpu6050.initialize();
            break;
        case IMU_ICM20689:
            icm20689.init(Wire, 0x68);
            icm20689.begin();
            Serial.println("IMU type: ICM20689");
            break;
        case IMU_ICM42605:
            icm42605.reset();
            delay(5);
            icm42605.init(AFS_2G, GFS_2000DPS, AODR_1000Hz, GODR_1000Hz);
            icm42605.reset();
            Serial.println("IMU type: ICM42605");
            Serial.print("Imu Device ID:");
            Serial.println(icm42605.getChipID(), HEX);
            break;
        case IMU_BMI088:
            bmi088.init(SPI, 12, 13);
            sta = bmi088.begin();
            if (sta < 0) {
                Serial.println("imu Initialization Error");
                Serial.println(sta);
            }
            break;
        default:
            break;
    }
    //没有检验whoami，为简易的适配其他IMU
    // TO DO:检验whoami
}

int8_t IMU::update() {
    int16_t rawAccx, rawAccy, rawAccz, rawGyrox, rawGyroy, rawGyroz;
    float gyroScale = 0.061;
    float r2d = 180.0 / 3.1415;
    switch (imu_type) {
        case IMU_MPU6050:
            mpu6050.getMotion6(&rawAccx, &rawAccy, &rawAccz, &rawGyrox, &rawGyroy, &rawGyroz);
            ax = (double) rawAccx;
            ay = (double) rawAccy;
            az = (double) rawAccz;
            gx = (double) rawGyrox * gyroScale;
            gy = (double) rawGyroy * gyroScale;
            gz = (double) rawGyroz * gyroScale;
            break;
        case IMU_MPU6500:
            mpu6050.getMotion6(&rawAccx, &rawAccy, &rawAccz, &rawGyrox, &rawGyroy, &rawGyroz);
            ax = (double) rawAccx;
            ay = (double) rawAccy;
            az = (double) rawAccz;
            gx = (double) rawGyrox * gyroScale;
            gy = (double) rawGyroy * gyroScale;
            gz = (double) rawGyroz * gyroScale;
            break;
        case IMU_MPU9250:
            mpu6050.getMotion6(&rawAccx, &rawAccy, &rawAccz, &rawGyrox, &rawGyroy, &rawGyroz);
            ax = (double) rawAccx;
            ay = (double) rawAccy;
            az = (double) rawAccz;
            gx = (double) rawGyrox * gyroScale;
            gy = (double) rawGyroy * gyroScale;
            gz = (double) rawGyroz * gyroScale;
            break;
        case IMU_ICM20689:
            icm20689.readSensor();
            ax = icm20689.getAccelX_mss();
            ay = icm20689.getAccelY_mss();
            az = icm20689.getAccelZ_mss();
            gx = icm20689.getGyroX_dps();
            gy = icm20689.getGyroY_dps();
            gz = icm20689.getGyroZ_dps();
            break;
        case IMU_ICM42605:
            int16_t rawData[7];
            icm42605.readData(rawData);
            ax = (double) rawData[1];
            ay = (double) rawData[2];
            az = (double) rawData[3];
            gx = (double) rawData[4] * gyroScale;
            gy = (double) rawData[5] * gyroScale;
            gz = (double) rawData[6] * gyroScale;
            break;
        case IMU_BMI088:
            bmi088.readSensor();
            ax = bmi088.getAccelX_mss();
            ay = bmi088.getAccelY_mss();
            az = bmi088.getAccelZ_mss();
            gx = bmi088.getGyroX_rads() * r2d;
            gy = bmi088.getGyroY_rads() * r2d;
            gz = bmi088.getGyroZ_rads() * r2d;
            break;
        default:
            break;
    }
    return 1;
}

void IMU::movingWindowFil() {
    float mW_temp[6]={0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < WINDOW_NUM; j++) {
            if (j < WINDOW_NUM - 1)
                movingWin[i][j] = movingWin[i][j + 1];
            else
                switch (i) {
                    case 0:
                        movingWin[i][j] = (float) gx - gx_error;
                        break;
                    case 1:
                        movingWin[i][j] = (float) gy - gy_error;
                        break;
                    case 2:
                        movingWin[i][j] = (float) gz - gz_error;
                        break;
                    case 3:
                        movingWin[i][j] = (float) ax - ax_error;
                        break;
                    case 4:
                        movingWin[i][j] = (float) ay - ay_error;
                        break;
                    case 5:
                        movingWin[i][j] = (float) az - az_error;
                        break;
                    default:
                        break;
                }
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < WINDOW_NUM; j++) {
            mW_temp[i]+=movingWin[i][j];
        }
    }
    gx_mWfilted= mW_temp[0] / WINDOW_NUM;
    gy_mWfilted= mW_temp[1] / WINDOW_NUM;
    gz_mWfilted= mW_temp[2] / WINDOW_NUM;
    ax_mWfilted= mW_temp[3] / WINDOW_NUM;
    ay_mWfilted= mW_temp[4] / WINDOW_NUM;
    az_mWfilted= mW_temp[5] / WINDOW_NUM;
}

void IMU::getGyroStaticError() {
    double gyrox_add = 0;
    double gyroy_add = 0;
    double gyroz_add = 0;
    Serial.println("Keep Still to Calibrate Gyro");
    delay(1000);
    for (int i = 0; i < STATIC_ERROR_SAMPLE_NUM; i++) {
        update();
        delay(1);
        gyrox_add += gx;
        gyroy_add += gy;
        gyroz_add += gz;
    }
    Serial.println("Calibrated!");
    gx_error = (float) (gyrox_add / STATIC_ERROR_SAMPLE_NUM);
    gy_error = (float) (gyroy_add / STATIC_ERROR_SAMPLE_NUM);
    gz_error = (float) (gyroz_add / STATIC_ERROR_SAMPLE_NUM);
}

// butterworth filter
void Butterworth50HzLPF(Bw50HzLPFTypeDef *pLPF) {
    pLPF->xv[0] = pLPF->xv[1];
    pLPF->xv[1] = pLPF->xv[2];
    pLPF->xv[2] = pLPF->xv[3];
    pLPF->xv[3] = pLPF->input / GAINBtw50hz;
    pLPF->yv[0] = pLPF->yv[1];
    pLPF->yv[1] = pLPF->yv[2];
    pLPF->yv[2] = pLPF->yv[3];
    pLPF->yv[3] = (pLPF->xv[0] + pLPF->xv[3]) + 3 * (pLPF->xv[1] + pLPF->xv[2]) + (0.5320753683f * pLPF->yv[0]) +
                  (-1.9293556691f * pLPF->yv[1]) + (2.3740947437f * pLPF->yv[2]);

    pLPF->output = pLPF->yv[3];
}

void Butterworth30HzLPF(Bw30HzLPFTypeDef *pLPF) {
    pLPF->xv[0] = pLPF->xv[1];
    pLPF->xv[1] = pLPF->xv[2];
    pLPF->xv[2] = pLPF->input / GAINBtw30Hz;
    pLPF->yv[0] = pLPF->yv[1];
    pLPF->yv[1] = pLPF->yv[2];
    pLPF->yv[2] = (pLPF->xv[0] + pLPF->xv[2]) + 2 * pLPF->xv[1] + (-0.7660066009f * pLPF->yv[0]) +
                  (1.7347257688f * pLPF->yv[1]);
    pLPF->output = pLPF->yv[2];
}

void LPFUpdate6axis(float gyroGx, float gyroGy, float gyroGz, float ax, float ay, float az) {
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