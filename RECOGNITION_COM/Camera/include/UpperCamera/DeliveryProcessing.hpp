//
// Created by jarry_goon on 24. 10. 18.
//

#ifndef DELIVERY_HPP
#define DELIVERY_HPP

#include <YOLO.cuh>

#define DETECTED_OBJECT_MINIMUM_SCORE 0.75 // should be set
#define OBJECT_MATCH_THRESHOLD 20
#define MIN_COUNT_FRAME 20

typedef enum SIGN_labels
{
    _,
    SIGN_1,
    SIGN_2,
    SIGN_3,
    SIGN_A,
    SIGN_B,
    SIGN_BOX,
} SIGN_Labels;

typedef struct Delivery_State
{
    char labels;
    int  sign_num_idx;
} Delivery_State;

void draw_sign_boxes(const YOLO::Object &sign_object, cv::Mat &frame);

void classify_object(std::vector<YOLO::Object> &signs, std::vector<YOLO::Object> &          signs_alp,
                     std::vector<YOLO::Object> &signs_num, const std::vector<YOLO::Object> &objects
);

Delivery_State verify_object(const std::vector<YOLO::Object> &signs, const std::vector<YOLO::Object> &signs_alp,
                             const std::vector<YOLO::Object> &signs_num,
                             SIGN_labels                      confirmed_delivery_state
);

void lock_object_info(const YOLO::Object &signs_num, SIGN_Labels confirmed_delivery_state,
                      YOLO::Object &      target_info, int &     detection_frame_count
);

#endif //DELIVERY_HPP
