//
// Created by Lay on 3/20/2022.
//

#include "tasks.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"


#define PID_CTRL_PERIOD 5 //:ms
#define UDP_PORT 4399
#ifndef EXT_WIFI_CONFIG
#define WIFI_SSID "lay"
#define WIFI_PASSWORD "12345678"
#endif


Mahony mahony;
AsyncUDP udp;
IPAddress remoteIP;

xQueueHandle serialPrintQueue = xQueueCreate(10, sizeof(serialPrintData_t));
xQueueHandle attDataQueue = xQueueCreate(10, sizeof(attFeedbackData_t));
xQueueHandle udpRecDataQueue = xQueueCreate(10, sizeof(AsyncUDPPacket));
xQueueHandle udpSendDataQueue = xQueueCreate(10, sizeof(char) * 300);
xQueueHandle paramChangeQueue = xQueueCreate(10, sizeof(paramChange_t));

xSemaphoreHandle paramUdpSend = xSemaphoreCreateBinary();
xSemaphoreHandle attUdpSend = xSemaphoreCreateBinary();
xSemaphoreHandle attUdpEnd = xSemaphoreCreateBinary();

//xQueueHandle udpSendQueue = xQueueCreate(10,sizeof());

void pid_ctrl_init() {
//    TODO:1.tune kP kI kD
    anglePitch_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
    angleRoll_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
    angleYaw_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
    angularVelPitch_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
    angularVelRoll_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
    angularVelYaw_ctrl.init(1, 0, 0, 0, PID_CTRL_PERIOD);
}

void att_calc_init() {
    imu.init();
//    -324.00,99.00,-96.00
//   -31.00 12.00 -8.00
    imu.gx_error = -76.00;
    imu.gy_error = 19.00;
    imu.gz_error = -45.00;
//    imu.getGyroStaticError();
    Serial.print(imu.gx_error);
    Serial.print(imu.gy_error);
    Serial.println(imu.gz_error);
    mahony.begin(200);
}

void onPacketCallBack(AsyncUDPPacket packet) {
    BaseType_t xHigherPriorityTaskWoken;
    udpRecData_t recData;
    packet.read(recData.data, 10);
    remoteIP = packet.remoteIP();
    xQueueSendFromISR(udpRecDataQueue, &recData, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR ();
}

uint8_t wifi_udp_init() {
    uint8_t count;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Connecting");
    while (!WiFi.isConnected()) {
        delay(500);
        Serial.println(".");
        count++;
        if(count>10)
        {
            Serial.println("connect failed");
            return 0;
        }
    }
    count=0;
    Serial.println("UDP initializng");
    while (!udp.listen(UDP_PORT)) //等待udp监听设置成功
    {
        Serial.println(".");
    }
    udp.onPacket(onPacketCallBack); //注册收到数据包事件
    return 1;
}

//udp资源守护任务
[[noreturn]] void udp_send_task(void *pvParameters) {
    char toSend[300];
    while (1) {
        xQueueReceive(udpSendDataQueue, toSend, portMAX_DELAY);
        udp.writeTo((uint8_t *) toSend, strlen(toSend), remoteIP, 45454);
    }
}

[[noreturn]] void udp_parser_data_task(void *pvParameters) {
    udpRecData_t recData;
    paramChange_t newParam;
    String parserStr;
    int index;
    while (1) {
        xQueueReceive(udpRecDataQueue, &recData, portMAX_DELAY);
        if (strcmp( (char *)recData.data , "connect")) {
            xSemaphoreGive(paramUdpSend);
            xSemaphoreGive(attUdpSend);
        } else if (strcmp((char *) recData.data,"end")) {
            xSemaphoreGive(attUdpEnd);
        } else {
            parserStr = (char *) recData.data;
            index = parserStr.indexOf(":");
            if (index == -1) {
                parserStr.substring(0, index - 1).toCharArray(newParam.param, 10);
                newParam.value = parserStr.substring(index + 1, parserStr.length() - 2).toFloat();
                xQueueSend(paramChangeQueue, &newParam, 3);
            }
        }
    }
}


[[noreturn]] void att_update_task(void *pvParameters) {
    TickType_t lastwaketime;
    float gyroScale = 0.061;
    attFeedbackData_t attData;
    bool udpSendFlag = false;
    serialPrintData_t toPrint;
    uint8_t count=0;
    float gx, gy, gz, gxFilted, gyFilted, gzFilted, axFilted, ayFilted, azFilted;
    while (1) {
        imu.update();
        gx = (imu.gx - imu.gx_error) * gyroScale;
        gy = (imu.gy - imu.gy_error) * gyroScale;
        gz = (imu.gz - imu.gz_error) * gyroScale;
        LPFUpdate6axis(gx, gy, gz, imu.ax, imu.ay, imu.az);
        gxFilted = Filters.GyroxLPF.output;
        gyFilted = Filters.GyroyLPF.output;
        gzFilted = Filters.GyrozLPF.output;
        axFilted = Filters.AccxLPF.output;
        ayFilted = Filters.AccyLPF.output;
        azFilted = Filters.AcczLPF.output;
        mahony.updateIMU(gxFilted, gyFilted, gzFilted, axFilted, ayFilted, azFilted);
        attData={gxFilted,gyFilted,gzFilted,mahony.getPitch(),mahony.getRoll(),mahony.getYaw()}; // deg & deg/s !!
        xQueueSendToFront(attDataQueue,(void *)&attData,1/portTICK_PERIOD_MS);
        if (xSemaphoreTake(attUdpSend, 0))
            udpSendFlag = true;
        else if (xSemaphoreTake(attUdpEnd, 0))
            udpSendFlag = false;
        if (udpSendFlag && count) {
            char attStr[60]={0};
            sprintf(attStr, "angle:0;pitchCur:%.2f;rollCur:%.2f;yawCur:%.2f;",
                    mahony.getPitch(), mahony.getRoll(), mahony.getYaw());
            xQueueSend(udpSendDataQueue, attStr, 1);
        }
        (count==0)? (count=1):(count=0);
        toPrint.type = "attitude";
        toPrint.data[0] = mahony.getPitch();
        toPrint.data[1] = mahony.getRoll();
        toPrint.data[2] = mahony.getYaw();
        xQueueSend(serialPrintQueue, (void *) &toPrint, 0 / portTICK_PERIOD_MS);
        vTaskDelayUntil(&lastwaketime, 5 / portTICK_PERIOD_MS);
    }
}

[[noreturn]] void serial_print_task(void *pvParameters) {
    serialPrintData_t dataToPrint;
    while (1) {
        xQueueReceive(serialPrintQueue, &dataToPrint, portMAX_DELAY);
        Serial.print(dataToPrint.type);
        Serial.print(":");
        Serial.print(dataToPrint.data[0]);
        Serial.print(",");
        Serial.print(dataToPrint.data[1]);
        Serial.print(",");
        Serial.println(dataToPrint.data[2]);
    }
}

[[noreturn]] void pid_calculate_task(void *pvParameters) {
    TickType_t lastwaketime;
    attFeedbackData_t attData;
    paramChange_t newParam;
    serialPrintData_t toPrint;
    char letter0,letter2;
    float angleYaw, angleRoll, anglePitch, angularVelYaw, angularVelRoll, angularVelPitch;
    while (1) {
        if(xQueueReceive(paramChangeQueue,&newParam,0)) {
            letter0=newParam.param[0];
            letter2=newParam.param[2];
            if(letter0=='o')
            {
                if(letter2=='p')
                {
                    if (strcmp(newParam.param,"okp1") == 0)
                        anglePitch_ctrl.kP=newParam.value;
                    else if(strcmp(newParam.param,"okp2") == 0)
                        angleRoll_ctrl.kP=newParam.value;
                    else if(strcmp(newParam.param,"okp3") == 0)
                        angleYaw_ctrl.kP=newParam.value;
                }
                else if(letter2=='i')
                {
                    if (strcmp(newParam.param,"oki1") == 0)
                        anglePitch_ctrl.kI=newParam.value;
                    else if(strcmp(newParam.param,"oki2") == 0)
                        angleRoll_ctrl.kI=newParam.value;
                    else if(strcmp(newParam.param,"oki3") == 0)
                        angleYaw_ctrl.kI=newParam.value;
                }
                else if(letter2=='d')
                {
                    if (strcmp(newParam.param,"okd1") == 0)
                        anglePitch_ctrl.kD=newParam.value;
                    else if(strcmp(newParam.param,"okd2") == 0)
                        angleRoll_ctrl.kD=newParam.value;
                    else if(strcmp(newParam.param,"okd3") == 0)
                        angleYaw_ctrl.kD=newParam.value;
                }
            }
            // TODO：这tm写的是什么玩意
            else if(letter0=='i')
            {
                if(letter2=='p')
                {
                    if (strcmp(newParam.param,"ikp1") == 0)
                        angularVelPitch_ctrl.kP=newParam.value;
                    else if(strcmp(newParam.param,"ikp2") == 0)
                        angularVelRoll_ctrl.kP=newParam.value;
                    else if(strcmp(newParam.param,"ikp3") == 0)
                        angularVelYaw_ctrl.kP=newParam.value;
                }
                else if(letter2=='i')
                {
                    if (strcmp(newParam.param,"iki1") == 0)
                        angularVelPitch_ctrl.kI=newParam.value;
                    else if(strcmp(newParam.param,"iki2") == 0)
                        angularVelRoll_ctrl.kI=newParam.value;
                    else if(strcmp(newParam.param,"iki3") == 0)
                        angularVelYaw_ctrl.kI=newParam.value;
                }
                else if(letter2=='d')
                {
                    if (strcmp(newParam.param,"ikd1") == 0)
                        angularVelPitch_ctrl.kD=newParam.value;
                    else if(strcmp(newParam.param,"ikd2") == 0)
                        angularVelRoll_ctrl.kD=newParam.value;
                    else if(strcmp(newParam.param,"ikd3") == 0)
                        angularVelYaw_ctrl.kD=newParam.value;
                }
            }
            toPrint.type="pid param";
            toPrint.data[0]=anglePitch_ctrl.kP;
            toPrint.data[1]= anglePitch_ctrl.kI;
            toPrint.data[2]= anglePitch_ctrl.kD;
            xQueueSend(serialPrintQueue,  &toPrint, 0 / portTICK_PERIOD_MS);
        }
        xQueueReceive(attDataQueue, &attData, 1);
        angularVelPitch = attData.gx * 0.01744;
        angularVelRoll = attData.gy * 0.01744;
        angularVelYaw = attData.gz * 0.01744;
        anglePitch = attData.pitch * 0.01744;  //deg to rad
        angleRoll = attData.roll * 0.01744;
        angleYaw = attData.yaw * 0.01744;

        anglePitch_ctrl.calculate(anglePitch);
        angularVelPitch_ctrl.setDes(anglePitch_ctrl.output);
        angularVelPitch_ctrl.calculate(angularVelPitch);

        angleRoll_ctrl.calculate(angleRoll);
        angularVelRoll_ctrl.setDes(angleRoll_ctrl.output);
        angularVelRoll_ctrl.calculate(angularVelRoll);

        angleYaw_ctrl.calculate(angleYaw);
        angularVelYaw_ctrl.setDes(angleYaw_ctrl.output);
        angularVelYaw_ctrl.calculate(angularVelYaw);

        motor_pwm_calculate(angularVelPitch_ctrl.output, angularVelRoll_ctrl.output, angularVelYaw_ctrl.output);
        motor_set_speed();

        if (xSemaphoreTake(paramUdpSend, 0)) {
            char paramList[300]={0};
            sprintf(paramList, "param;okp1:%.2f;oki1:%.2f;okd1:%.2f;ikp1:%.2f;iki1:%.2f;ikd1:%.2f;"
                               "okp2:%.2f;oki2:%.2f;okd2:%.2f;ikp2:%.2f;iki2:%.2f;ikd2:%.2f;"
                               "okp3:%.2f;oki3:%.2f;okd3:%.2f;ikp3:%.2f;iki3:%.2f;ikd3:%.2f;",
                    anglePitch_ctrl.kP, anglePitch_ctrl.kI, anglePitch_ctrl.kD,
                    angularVelPitch_ctrl.kP, angularVelPitch_ctrl.kI, angularVelPitch_ctrl.kD,
                    angleRoll_ctrl.kP, angleRoll_ctrl.kI, angleRoll_ctrl.kD,
                    angularVelRoll_ctrl.kP, angularVelRoll_ctrl.kI, angularVelRoll_ctrl.kD,
                    angleYaw_ctrl.kP, angleYaw_ctrl.kI, angleYaw_ctrl.kD,
                    angularVelYaw_ctrl.kP, angularVelYaw_ctrl.kI, angularVelYaw_ctrl.kD);
            xQueueSend(udpSendDataQueue, paramList, 3);
        }
        vTaskDelayUntil(&lastwaketime, 5 / portTICK_PERIOD_MS);
    }
}

void nonRtosTask() {
    float gyroScale = 0.061;
    imu.update();
    LPFUpdate6axis((imu.gx - imu.gx_error) * gyroScale, (imu.gy - imu.gy_error) * gyroScale,
                   (imu.gz - imu.gz_error) * gyroScale, imu.ax, imu.ay, imu.az);
    mahony.updateIMU(Filters.GyroxLPF.output / 57.3, Filters.GyroyLPF.output / 57.3, Filters.GyrozLPF.output / 57.3,
                     Filters.AccxLPF.output, Filters.AccyLPF.output, Filters.AcczLPF.output);
    Serial.println(mahony.getYaw());
}

