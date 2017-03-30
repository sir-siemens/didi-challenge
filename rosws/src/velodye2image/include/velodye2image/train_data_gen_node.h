#ifndef TRAIN_DATA_GEN_NODE_H
#define TRAIN_DATA_GEN_NODE_H
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodye2image/convert_pc.h>
#include <sstream>
namespace didi{


class TrainDataGenNode
{
public:
    TrainDataGenNode();

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);



private:
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr_;
    Convert_pc cpc_;
    ros::Subscriber sub_;
};
}
#endif // TRAIN_DATA_GEN_NODE_H
