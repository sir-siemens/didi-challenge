#include <object_detection/detection_pipeline.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "detection_pipeline");

    didi::DetectionPipeline detection_pipeline;

    ros::spin();

    return 0;
}
