//
// Created by User on 24. 10. 25.
//

#include "Socket_Thread.h"

DWORD WINAPI SocketThread(LPVOID lpParameter) {
    int recv_len;

    RecvPacket recv_data;

    ThreadParam *param = (ThreadParam *) lpParameter;
    SocketInfo *socket_info = param->socket_info;
    HANDLE mutex = param->mutex;

    while (param->running) {
        recv_len = IP_receive(socket_info, (char *) &recv_data, sizeof(RecvPacket));

        if (recv_len <= 0) continue;

        WaitForSingleObject(mutex, INFINITE);

        param->packet = recv_data;

        ReleaseMutex(mutex);
    }

    param->is_exit = 1;

    return 0;
}
