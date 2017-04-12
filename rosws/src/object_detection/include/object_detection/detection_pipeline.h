#ifndef DETECTION_PIPELINE_H
#define DETECTION_PIPELINE_H
#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <object_detection/sensor.h>
#include <object_detection/radar.h>
#include <map>
#include <object_detection/vehicle_model.h>
namespace didi{


class DetectionPipeline
{
    public:
        DetectionPipeline();

        void mainLoop();


    private:
        std::vector< boost::shared_ptr<Sensor> > sensor_list_;

        // maintain a list of detected vehicle
        std::vector<Vehicle_model > detected_vehicle_list_;



};
}
#endif // DETECTION_PIPELINE_H
