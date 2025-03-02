//
// Created by User on 24. 10. 25.
//

#ifndef SOCKET_THREAD_H
#define SOCKET_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Windows.h>
#include <communication.h>

#include "Communication2Control.h"

typedef struct _ThreadParam {
    SocketInfo *socket_info;
    HANDLE mutex;
    RecvPacket packet;
    char running;
    char is_exit;
} ThreadParam;

DWORD WINAPI SocketThread(LPVOID lpParameter);

#ifdef __cplusplus
}
#endif

#endif //SOCKET_THREAD_H
