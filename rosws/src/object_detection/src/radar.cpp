#include "object_detection/radar.h"

namespace didi{

Radar::Radar() {
    ros::NodeHandle n;
    sub_ = n.subscribe("/radar/points", 1,
                            &Radar::cloud_callback, this);

}


void Radar::detect_vehicle(std::vector <Vehicle_model> &vehicle_list) {
    // detect vehicle from the current configuration
    vehicle_list.resize(cloud_depth_ptr_->points.size());
    for ( size_t i = 0; i < cloud_depth_ptr_->points.size(); i++ ){
        Vehicle_model vm;
        vm.pose_.x = cloud_depth_ptr_->points[i].x;
        vm.pose_.y = cloud_depth_ptr_->points[i].y;
        vm.boundingbox_.x = rader_detector_.target_length();
        vm.boundingbox_.y = rader_detector_.target_width();
        // TODO determine
        // vm.boundingbox_.z = rader_detector_;
        vehicle_list[i] = vm;
    }
}

double Radar::compute_weight( Particle p) {
    geometry_msgs::Pose2D ego_T_target;

    ego_T_target.x = p.pose_.x;
    ego_T_target.y = p.pose_.y;
    ego_T_target.theta = p.pose_.theta;
    return rader_detector_.measurement_model(cloud_depth_ptr_, ego_T_target);
}

void Radar::cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_ptr_ = msg;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_ptr_, pcl_pc);
    cloud_depth_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2( pcl_pc,  *cloud_depth_ptr_);
    // ROS_INFO("cloud size %d", (int)cloud_depth_ptr->points.size());
}



}
