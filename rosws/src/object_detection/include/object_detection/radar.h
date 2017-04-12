#ifndef RADAR_H
#define RADAR_H
#include <object_detection/sensor.h>
#include <object_detection/bbdetection_radar.h>
namespace didi{

class Radar : public Sensor
{
public:
    Radar();

    virtual void detect_vehicle(std::vector <Vehicle_model> &vehicle_list);

    virtual double compute_weight( Particle p) ;

    void cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    ros::Subscriber sub_;
    BBDetection_Radar rader_detector_;
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth_ptr_;
};

}
#endif // RADAR_H
