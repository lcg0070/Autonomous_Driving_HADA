//
// Created by jarry_goon on 24. 10. 18.
//

#include "UpperCamera/DeliveryProcessing.hpp"
#include "UpperCamera/BoundingBoxProcessing.hpp"

static int delivery_spot  = 0;
static int sign_count     = 0;
static int sign_alp_count = 0;
static int sign_num_count = 0;

void draw_sign_boxes(const YOLO::Object &sign_object, cv::Mat &frame)
{
    cv::String sign_label;
    cv::Scalar sign_color;

    switch((SIGN_Labels)sign_object.label)
    {
        case SIGN_1:
            sign_label = "1";
            sign_color = cv::Scalar(0, 0, 255);
            break;
        case SIGN_2:
            sign_label = "2";
            sign_color = cv::Scalar(0, 165, 255);
            break;
        case SIGN_3:
            sign_label = "3";
            sign_color = cv::Scalar(0, 255, 255);
            break;
        case SIGN_A:
            sign_label = "A";
            sign_color = cv::Scalar(0, 255, 0);
            break;
        case SIGN_B:
            sign_label = "B";
            sign_color = cv::Scalar(255, 255, 0);
            break;
        case SIGN_BOX:
            sign_label = "Sign";
            sign_color = cv::Scalar(255, 0, 0);
            break;
        default:
            sign_label = "Unknown Sign";
            sign_color = cv::Scalar(255, 0, 0);
            break;
    }
    cv::rectangle(frame, cv::Rect(sign_object.bbox.x, sign_object.bbox.y, sign_object.bbox.width,
                                  sign_object.bbox.height), sign_color, 1, 8, 0);
    cv::putText(frame, sign_label, cv::Point(sign_object.bbox.x, sign_object.bbox.y - 10), cv::FONT_HERSHEY_SIMPLEX,
                0.5, sign_color, 1);
}

void classify_object(std::vector<YOLO::Object> &signs, std::vector<YOLO::Object> &          signs_alp,
                     std::vector<YOLO::Object> &signs_num, const std::vector<YOLO::Object> &objects
)
{
    sign_count     = 0;
    sign_alp_count = 0;
    sign_num_count = 0;

    for(const YOLO::Object &object: objects)
    {
        if(object.score <= DETECTED_OBJECT_MINIMUM_SCORE) continue;

        switch((SIGN_Labels)object.label)
        {
            case SIGN_BOX:
                signs.push_back(object);
                sign_count++;
                break;

            case SIGN_A:
            case SIGN_B:
                signs_alp.push_back(object);
                sign_alp_count++;
                break;

            case SIGN_1:
            case SIGN_2:
            case SIGN_3:
                signs_num.push_back(object);
                sign_num_count++;
                break;

            default:
                break;
        }
    }
}

Delivery_State verify_object(const std::vector<YOLO::Object> &signs, const std::vector<YOLO::Object> &signs_alp,
                             const std::vector<YOLO::Object> &signs_num,
                             SIGN_labels                      confirmed_delivery_state
)
{
    cv::Rect present_bbox;

    Delivery_State checked_delivery_state = {_, -1};

    for(int sign_idx = 0; sign_idx < sign_count; sign_idx++)
    {
        present_bbox = signs[sign_idx].bbox;

        // Alphabet
        for(int sign_alp_idx = 0; sign_alp_idx < sign_alp_count; sign_alp_idx++)
        {
            if(!bound_inner_check(present_bbox, signs_alp[sign_alp_idx].bbox) ||
               !signs_alp[sign_alp_idx].label == confirmed_delivery_state)
                continue;

            switch(confirmed_delivery_state)
            {
                case SIGN_A:
                    checked_delivery_state.labels = 'A';
                    break;
                case SIGN_B:
                    checked_delivery_state.labels = 'B';
                    break;

                default:
                    checked_delivery_state.labels = 0;
                    break;
            }

            // printf("signs_alp[sign_alp_idx].label: %c\n", checked_delivery_state.labels);
        }

        if(checked_delivery_state.labels == 0) continue;

        // Number
        for(int sign_num_idx = 0; sign_num_idx < sign_num_count; sign_num_idx++)
        {
            if(!bound_inner_check(present_bbox, signs_num[sign_num_idx].bbox)) continue;

            switch(confirmed_delivery_state)
            {
                case SIGN_B:
                    if(signs_num[sign_num_idx].label != delivery_spot)
                        break;

                case SIGN_A:
                    checked_delivery_state.sign_num_idx = sign_num_idx;
                    return checked_delivery_state;

                default:
                    break;
            }
        }
    }

    return checked_delivery_state;
}

void lock_object_info(const YOLO::Object &signs_num, SIGN_Labels confirmed_delivery_state,
                      YOLO::Object &      target_info, int &     detection_frame_count
)
{
    static YOLO::Object delivery_detection_in_frame;

    if(bbox_norm1(signs_num.bbox, delivery_detection_in_frame.bbox) <= OBJECT_MATCH_THRESHOLD)
    {
        delivery_detection_in_frame = signs_num;

        detection_frame_count++;
    }
    else if(bbox_norm1(signs_num.bbox, delivery_detection_in_frame.bbox) > OBJECT_MATCH_THRESHOLD)
    {
        delivery_detection_in_frame = signs_num;

        detection_frame_count = 1;
    }
    else detection_frame_count = 0;

    printf("detection_frame_count: %d\n", detection_frame_count);
    if(detection_frame_count >= MIN_COUNT_FRAME)
    {
        delivery_detection_in_frame = signs_num;

        if(confirmed_delivery_state == 'A')
        {
            delivery_spot = delivery_detection_in_frame.label;
            // confirmed_delivery_state = 'B';

            printf("state changed: %c\ndelivery spot: %d\n", confirmed_delivery_state, delivery_spot);
        }
        target_info = delivery_detection_in_frame;
    }
}
