//
// Created by Áõ¼Î¿¡ on 25-6-2.
//

#ifndef CTRBOARD_H7_ALL_ONENET_TASK_H
#define CTRBOARD_H7_ALL_ONENET_TASK_H


void ESP8266_Init(void);
char* generateMqttPublishCommand1(float temp,float humi,float shine,float CO1,float alcohol);
char* generateMqttPublishCommand2(float jiaquan,float yiquan,float wumai,float leida,float ziwaixian);


#endif //CTRBOARD_H7_ALL_ONENET_TASK_H
