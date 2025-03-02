//
// Created by dadan on 24. 10. 20.
//

#include "UpperCamera/TrafficLightProcessing.hpp"

void draw_light_boxes(const YOLO::Object &light_object, cv::Mat &frame)
{
    cv::String sign_label;
    cv::Scalar sign_color;

    switch((LIGHT_Labels)light_object.label)
    {
        case G4:
            sign_label = "G4";
        sign_color = cv::Scalar(0, 165, 255);
        break;
        case GL4:
            sign_label = "GL4";
        sign_color = cv::Scalar(0, 255, 255);
        break;
        case R4:
            sign_label = "R4";
        sign_color = cv::Scalar(0, 255, 0);
        break;
        case RL3:
            sign_label = "RL3";
        sign_color = cv::Scalar(255, 255, 0);
        break;
        case RL4:
            sign_label = "RL4";
        sign_color = cv::Scalar(255, 123, 0);
        break;
        case Y4:
            sign_label = "Y4";
        sign_color = cv::Scalar(255, 0, 0);
        break;
        default:
            sign_label = "Unknown Sign";
        sign_color = cv::Scalar(255, 0, 0);
        break;
    }
    cv::rectangle(frame, cv::Rect(light_object.bbox.x, light_object.bbox.y, light_object.bbox.width,
                                  light_object.bbox.height), sign_color, 1, 8, 0);
    cv::putText(frame, sign_label, cv::Point(light_object.bbox.x, light_object.bbox.y - 10), cv::FONT_HERSHEY_SIMPLEX,
                0.5, sign_color, 1);
}

void light_state(int light_label, cv::Mat &frame)
{
    cv::String sign_label;
    cv::Scalar sign_color;

    if(light_label == -1) {

        sign_label = "light_state: NONE";
        sign_color = cv::Scalar(255, 0, 0);
    }
    else {
        switch(light_label)
        {
            case 0: // G4:
                sign_label = "light_state: G4";
            sign_color = cv::Scalar(0, 165, 255);
            break;
            case 1: // GL4:
                sign_label = "light_state: GL4";
            sign_color = cv::Scalar(0, 255, 255);
            break;
            case 2: // R4:
                sign_label = "light_state: R4";
            sign_color = cv::Scalar(0, 255, 0);
            break;
            case 3: // RL3:
                sign_label = "light_state: RL3";
            sign_color = cv::Scalar(255, 255, 0);
            break;
            case 4: // RL4:
                sign_label = "light_state: RL4";
            sign_color = cv::Scalar(255, 123, 0);
            break;
            case 5: // Y4:
                sign_label = "light_state: Y4";
            sign_color = cv::Scalar(255, 0, 0);
            break;
            default:
                sign_label = "Unknown Light";
            sign_color = cv::Scalar(255, 0, 0);
            break;
        }
    }
    cv::putText(frame, sign_label, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.5, sign_color, 1);
}