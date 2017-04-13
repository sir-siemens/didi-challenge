#include "object_detection/radar.h"

namespace didi{

Radar::Radar() {
    ros::NodeHandle n;
    sub_ = n.subscribe("/radar/points", 1,
                            &Radar::cloud_callback, this);
    radar_cloud_recieved_ = false;
    // set detection range to 100
    radar_max_ = 100;
}


void Radar::detect_vehicle(std::vector <Vehicle_model> &vehicle_list) {
    if (radar_cloud_recieved_ == false) {
        return;
    }
    // detect vehicle from the current configuration
    // vehicle_list.resize(cloud_depth_ptr_->points.size());
    for ( size_t i = 0; i < cloud_depth_ptr_->points.size(); i++ ){
        Vehicle_model vm;
        geometry_msgs::Point radar_point;
        // here x cooridnate + 0.5 target vehicle length is a prior that target vehicle
        // is drive in the same direction as ego vehicle
        radar_point.x = cloud_depth_ptr_->points[i].x+0.5*rader_detector_.target_length();
        radar_point.y = cloud_depth_ptr_->points[i].y;

        geometry_msgs::Point vehicle_positionInego_baselink = rader_detector_.transform_radar_ptInEgo_baselink(radar_point);

        vm.pose_.x = vehicle_positionInego_baselink.x;
        vm.pose_.y = vehicle_positionInego_baselink.y;
        vm.boundingbox_.x = rader_detector_.target_length();
        vm.boundingbox_.y = rader_detector_.target_width();
        // TODO determine
        // vm.boundingbox_.z = rader_detector_;
        if (vm.pose_.x < radar_max_) {
            vehicle_list.push_back(vm);
        }
    }
}

double Radar::compute_weight( Particle p) {
    geometry_msgs::Pose2D ego_T_target;
    ego_T_target.x = p.pose_.x;
    ego_T_target.y = p.pose_.y;
    ego_T_target.theta = p.pose_.theta;
    return rader_detector_.measurement_model_simple(cloud_depth_ptr_, ego_T_target);
}

void Radar::cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    radar_cloud_recieved_ = true;
    cloud_ptr_ = msg;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_ptr_, pcl_pc);
    cloud_depth_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2( pcl_pc,  *cloud_depth_ptr_);
    // ROS_INFO("cloud size %d", (int)cloud_depth_ptr->points.size());
}



}
