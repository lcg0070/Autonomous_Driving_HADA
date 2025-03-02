/*****************************************************************************************/
/* Author: KHW1960                                                                       */
/* Date  : 2024-09-12                                                                    */
/*****************************************************************************************/

#include <opencv2/opencv.hpp>
#include "CameraMainUtils.hpp"
// #include "3D_Lidarfunction.hpp"

int main() {

    init_camera();
    // init_velodyne();

    while (true) {

        camera_processing();

        if (cv::waitKey(1) == 27) break;
    }

    // clear_velodyne();

    return 0;
}