//
// Created by User on 24. 10. 25.
//

#include "Communication2Control.h"

#include <communication.h>

#include "Socket_Thread.h"

static ThreadParam param;
static SocketInfo socket_info;

static HANDLE thread;

int init_communication() {
    socket_info = UDP_IPv4_client(CONTROL_COM_PORT, CONTROL_COM_IP, 1);
    if(socket_info.socket == INVALID_SOCKET) {
        print_socket_error("ERROR: Failed to open socket.");

        return -1;
    }


    param.socket_info = &socket_info;
    param.running = 1;
    param.is_exit = 0;

    return 0;
}

int send_data(SendPacket packet);

RecvPacket recv_data();

int clear_communication();

