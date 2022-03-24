//
// Created by Lay on 3/24/2022.
//

#ifndef SOURCE_PID_H
#define SOURCE_PID_H
class PID
{
public:
    float kP,kI,kD;
    float des,cur,output;
private:
    float period,errInt,errDiff,errLast,errCur;
public:
    void init(float kP,float kI,float kD,float des,float T);
    void calculate(float currentData);
    void setDes(float newDes);
};
extern PID anglePitch_ctrl,angleRoll_ctrl,angleYaw_ctrl;
extern PID angularVelPitch_ctrl,angularVelRoll_ctrl,angularVelYaw_ctrl;
#endif //SOURCE_PID_H
