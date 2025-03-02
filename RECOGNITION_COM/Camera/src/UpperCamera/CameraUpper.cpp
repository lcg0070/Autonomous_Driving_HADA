/*****************************************************************************************/
/* Author: KHW1960                                                                       */
/* Date  : 2024-09-15                                                                    */
/*****************************************************************************************/

#include "UpperCamera/CameraUpper.hpp"
#include "UpperCamera/DeliveryProcessing.hpp"
#include "UpperCamera/TrafficLightProcessing.hpp"
#include "3D_Lidarfunction.hpp"
#include "VideoWriter.hpp"

YOLO SIGN("etc/sign_middle_150.engine");
YOLO LIGHT("etc/light_large_5th.engine");

static FrameCapture traffic;
static FrameCapture delivery;

// static?
static SIGN_Labels confirmed_delivery_state = SIGN_A;
// static SIGN_Labels confirmed_delivery_state = SIGN_B;

// static int delivery_spot = 3;

void init_upper_cam(cv::VideoCapture &front_upper_cam) {

    if (!front_upper_cam.isOpened()) {
        std::cerr << "Error: Could not open front_upper_camera" << std::endl;
    }

    front_upper_cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    front_upper_cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // socket_info = UDP_IPv4_client(PORT,SERVER_ADDR,1);

    traffic = FrameCapture("traffic", cv::Size(1280,720));
    delivery = FrameCapture("delivery", cv::Size(1280,720));
}

int delivery_mission(cv::Mat &front_upper_frame, YOLO::Object &target_info)
{

    static int detection_frame_count = 0;
    cv::Mat    delivery_frame        = front_upper_frame(cv::Range(80, front_upper_frame.rows),
                                                         cv::Range((front_upper_frame.cols >> 1) - 100,
                                                                   front_upper_frame.cols));


    int            interested_source[2] = {0};
    int            tol                  = 15;
    static int     delivery_stop_flag   = 0;
    static cv::Mat object_cord          = 500 * cv::Mat::ones(4, 1, CV_64F);

    Delivery_State checked_delivery_state;

    if(!SIGN.predict(delivery_frame))
    {
        detection_frame_count = 0;
        return -1;
    }

    std::vector<YOLO::Object> objects = SIGN.get_objects();
    std::vector<YOLO::Object> signs, signs_alp, signs_num;

    // classify_object(signs, signs_alp, signs_num, objects, delivery_frame);
    classify_object(signs, signs_alp, signs_num, objects);

    // check the alp and num in signs and save the detected information
    checked_delivery_state = verify_object(signs, signs_alp, signs_num, confirmed_delivery_state);
    if(checked_delivery_state.sign_num_idx == -1) {
        detection_frame_count = 0;

        return -1;
    }

    lock_object_info(signs_num[checked_delivery_state.sign_num_idx], confirmed_delivery_state,
                     target_info, detection_frame_count);
    if(!target_info.bbox.x) return -1;

    draw_sign_boxes(target_info, delivery_frame);

    interested_source[0] = (front_upper_frame.cols >> 1) - 100 + target_info.bbox.x + (target_info.bbox.width >> 1);
    interested_source[1] = 80 + target_info.bbox.y + (target_info.bbox.height >> 1);

    cv::circle(front_upper_frame, cv::Point(interested_source[0], interested_source[1]), 10, cv::Scalar(0, 0, 255),
               -1);

    cv::Mat lidar_uvcord;
    object_cord = velodyne_processing(interested_source, tol, lidar_uvcord);

    delivery_stop_flag = 0;
    //        3D lidar, camera calibartion
    if(object_cord.at<double>(0, 0) && std::abs(object_cord.at<double>(0, 0)) <= DELIVERY_STOP_THRESHOLD)
    {
        if(confirmed_delivery_state == SIGN_A)
        {
            delivery_stop_flag       = 1;
            confirmed_delivery_state = SIGN_B;
        }
        else if(confirmed_delivery_state == SIGN_B)
        {
            delivery_stop_flag = 2;
        }
    }

    cv::imshow("delivery_frame", delivery_frame);
    delivery.write(delivery_frame);
    return delivery_stop_flag;
}


int traffic_light_recognition(cv::Mat &front_upper_frame) {
    cv::Mat    traffic_light_frame        = front_upper_frame(cv::Range(front_upper_frame.rows/10, front_upper_frame.rows/10 + front_upper_frame.rows/2),
                                                         cv::Range(front_upper_frame.cols/3, front_upper_frame.cols/3 + front_upper_frame.cols/2));
    std::vector<YOLO::Object> objects = LIGHT.get_objects();

    static int lights_count[8] = {0}; // lights_count[N]: lightN count, lights_count[6]: full frame count, lights_count[7]: non object frame count

    if(lights_count[7] > 30) {
        lights_count[0] = 0;
        lights_count[1] = 0;
        lights_count[2] = 0;
        lights_count[3] = 0;
        lights_count[4] = 0;
        lights_count[5] = 0;
        lights_count[6] = 0;
    }

    if(!LIGHT.predict(traffic_light_frame))
    {
        lights_count[7]++;
        return -1;
    }

    lights_count[7] = 0;

    for(const YOLO::Object &object: objects) {
        draw_light_boxes(object, traffic_light_frame);

        // 정확도 확인
        if(object.label < 6) lights_count[object.label]++;
    }
    lights_count[6]++;

    if(lights_count[6] > 40) {
        int max_counted_light_idx = -1;

        for(int i = 0; i < 6; i++) {
            if(lights_count[i] < 30) continue;
            if(max_counted_light_idx == -1) max_counted_light_idx = i;

            if(lights_count[i] > lights_count[max_counted_light_idx]) max_counted_light_idx = i;
        }

        std::cout << "light number: " << max_counted_light_idx << std::endl;

        light_state(max_counted_light_idx, traffic_light_frame);

        lights_count[0] = 0;
        lights_count[1] = 0;
        lights_count[2] = 0;
        lights_count[3] = 0;
        lights_count[4] = 0;
        lights_count[5] = 0;
        lights_count[6] = 0;
    }

    cv::imshow("traffic_light_frame", traffic_light_frame);
    traffic.write(traffic_light_frame);
    return 0;
}

Flag upper_flag(cv::Mat &front_upper_frame, YOLO::Object &target_info) {
    Flag flag;

    flag.delivery =  delivery_mission(front_upper_frame, target_info);
    flag.traffic_light = traffic_light_recognition(front_upper_frame);

    return flag;
}