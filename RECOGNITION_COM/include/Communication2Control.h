//
// Created by User on 24. 10. 25.
//

#ifndef COMMUNICATION2CONTROL_H
#define COMMUNICATION2CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_COM_IP    ""
#define CONTROL_COM_PORT  1

typedef struct {
    char flag;
    float global_steer;
} RecvPacket;

typedef struct {
    char flag;
    float data1;
    float data2;
} SendPacket;

int init_communication();

int send_data(SendPacket packet);

RecvPacket recv_data();

int clear_communication();

#ifdef __cplusplus
}
#endif

#endif //COMMUNICATION2CONTROL_H
