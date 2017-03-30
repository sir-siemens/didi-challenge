#include "velodye2image/train_data_gen_node.h"

namespace didi{


TrainDataGenNode::TrainDataGenNode() {
    cpc_.init();
    ros::NodeHandle n;
    sub_ = n.subscribe("/kitti/velo/pointcloud", 1,
                            &TrainDataGenNode::callback, this);
}

void TrainDataGenNode::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("cloud recieved");
    cloud_ptr_ = msg;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_ptr_, pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // point cloud containts only depth
    pcl::fromPCLPointCloud2( pcl_pc,  *cloud_depth_ptr);
    ROS_INFO("cloud size %d", (int)cloud_depth_ptr->points.size());
    std::vector< cv::Mat > height_images;
    cpc_.convert_pc_density_img(cloud_depth_ptr,
                                height_images);

    for (size_t i = 0; i < height_images.size() ;i++) {
        // header
        std::stringstream file_name;


        file_name<<msg->header.stamp<<"_"<<i<<".png";

        // write image to the disk
        cv::imwrite(file_name.str(), height_images[i]);
    }
}

}







int main(int argc, char **argv) {

    ros::init(argc, argv, "training_data_generator");

    didi::TrainDataGenNode train_gen;

    ros::spin();
    return 0;
}
