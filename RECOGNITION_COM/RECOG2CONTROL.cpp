//
// Created by User on 2024-10-17.
//

#include <IP.h>
#include <UDP.h>
#include <Velodyne.h>

#include "CameraMainUtils.hpp"
#include "3D_Lidarfunction.hpp"
#include "LowerCamera/ComputePoints.hpp"
#include "UpperCamera/CameraUpper.hpp"

int main(){
    init_camera();
    SocketInfo socket =
        UDP_IPv4_client(5555,
        "192.168.1.21",1 );
    // float steer_cam = 0;
    while(1){
        Flag flag_send = upper_camera_processing();
        Output cmd_send= lower_camera_processing();
        // IP_send(socket,(char*)(&cmd_send),sizeof(Output));

        if (cv::waitKey(1) ==  27) break;

    }
    // clear_camera();

    return 0;
}
