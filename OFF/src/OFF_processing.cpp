//
// Created by LeeChanKeun on 2024-09-04.
//


#include <stdio.h>
#include <Windows.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "UST.h"
#include "circle_approx.hpp"
#include "OFF_processing.hpp"
#include "clustering.h"
#include "Communication.h"
#define PORT 55555
#define SERVER_ADDR "192.168.0.100"

typedef struct _comunication_packet {
    float steer_angle;
} Communication;

#define BUFFER_LEN sizeof(Communication)
static Communication comm;
static USTPolarData pol_data;
static USTRectData rec_data;

static SocketInfo socket_info;
static std::mutex mutx;
static std::thread recv_thread;

static bool running = true;

static float gpp_steer;

int init_OFF() {
    //    2D-LiDAR
    int flag = 0;

    // init UST
    while (1) {
        switch (init_UST()) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 0:
                flag = 1;
            default:
                break;
        }
        if (flag) {
            printf("init UST\n");
            break;
        }
    }
    socket_info = UDP_IPv4_client(PORT,SERVER_ADDR, 1);
    IP_send(socket_info, (char *) &comm, BUFFER_LEN);

    init_USTData(&pol_data, Polar);
    init_USTData(&rec_data, Rect);

    recv_thread = std::thread(receive_data);
    recv_thread.detach();

    Sleep(3000);

    printf("Initialize OFF Finished\n");
    return 0;
}


/*-----------------------------------------------------------------------------------*/
/* OFF Preprocessing                                                                 */
/*-----------------------------------------------------------------------------------*/

int OFF_processing() {
    //    out_array is 90~90 degree range array
    static float out_array[ORIGINAL_INDEX];
    static float median_array[ORIGINAL_INDEX];


    if (get_USTData(&pol_data, Polar)) {
        printf("ERROR in get_USTData\n");
        return 1;
    }

    median_filter(pol_data, median_array);


    circle_approx(pol_data, median_array, out_array);

    if (visualize_off(median_array, out_array)) return 1;

    return 0;
}

double OFF_processing_cluster(int visual_flag, int record_video_flag) {
    //    out_array is 90~90 degree range array
    static int len;
    int labels[ORIGINAL_INDEX] = {0,};
    double out_array[ORIGINAL_INDEX] = {0.};

    if (get_USTData(&pol_data, Polar) || get_USTData(&rec_data, Rect)) {
        printf("ERROR in get_USTData\n");
        return -50;
    }

    //DBSCAN
    int num_label = DBSCAN(rec_data.x + DEGREE_OFFSET, rec_data.y + DEGREE_OFFSET,
                           labels, ORIGINAL_INDEX, EPS_CLUSTER,
                           MINIMUM_DATAPOINTS, 1, NUM_LSH_TABLE);

    DBSCAN_processing(rec_data, labels, num_label, out_array);

    if (visual_flag) visual_cluster(rec_data, labels, out_array);

    float gpp_steer_local = 0.f;

    {
        std::lock_guard<std::mutex> lock(mutx);
        gpp_steer_local = gpp_steer;
    }

    comm.steer_angle = steer_command(out_array, gpp_steer_local);

    IP_send(socket_info, (char *) &comm, BUFFER_LEN);
    printf("Steer from Control = %f\t", gpp_steer);
    printf("Steer ang = %f\n", comm.steer_angle);
    if (cv::waitKey(1) == 27) return -50;

    return comm.steer_angle;
}

int clear_OFF() {
    running = false;
    clear_USTData(&pol_data, Polar);
    clear_USTData(&rec_data, Rect);
    clear_UST();
    clear_socket(socket_info);
    return 0;
}

void receive_data() {
    float gpp_steer_local = 0.f;

    while (running) {
        IP_receive(&socket_info,
                   (char *) (&gpp_steer_local),
                   sizeof(float));
        std::lock_guard<std::mutex> lock(mutx);
        gpp_steer = gpp_steer_local;
    }
}
