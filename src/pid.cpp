//
// Created by Lay on 3/24/2022.
//

#include "pid.h"
PID anglePitch_ctrl,angleRoll_ctrl,angleYaw_ctrl;
PID angularVelPitch_ctrl,angularVelRoll_ctrl,angularVelYaw_ctrl;
void PID::init(float P,float I,float D,float destination,float T)
{
    kP=P;
    kI=I;
    kD=D;
    des=destination;
    period=T;
}
void PID::calculate(float currentData) {
    cur=currentData;
    errCur=des-cur;
    errInt += (errCur+errLast)/2*period;
    errDiff=(errCur-errLast)/period;
    output=kP*errCur+kI*errInt+kD*errDiff;
    errLast=errCur;
}
void PID::setDes(float newDes)
{
    des=newDes;
}

void PID::getParamList(char *str) {
}

PID angleX,angleY,angleZ;
PID angularVelX,angularVelY,angularVelZ;