//
// Created by Lay on 3/20/2022.
//

#include "tasks.h"
#include "imu.h"


Mahony mahony;

void calc_att_init()
{
    imu.init();
    imu.gx_error=-248.00;
    imu.gy_error=114.00;
    imu.gz_error=-68.00;
    imu.getGyroStaticError();
    mahony.begin(200);
}
void task0()
{
    imu.update();
    float gyroScale = 0.061;
    LPFUpdate6axis((imu.gx-imu.gx_error)*gyroScale,(imu.gy-imu.gy_error)*gyroScale,
                   (imu.gz-imu.gz_error)*gyroScale,imu.ax,imu.ay,imu.az);
    mahony.updateIMU(Filters.GyroxLPF.output/57.3,Filters.GyroyLPF.output/57.3,Filters.GyrozLPF.output/57.3,
                     Filters.AccxLPF.output,Filters.AccyLPF.output,Filters.AcczLPF.output);
}
