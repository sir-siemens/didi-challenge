#include "velodye2image/convert_pc.h"

namespace didi {

Convert_pc::Convert_pc() {

}


void Convert_pc::init() {
    // manually define the resolution to be 0.1 for each pixel
    resolution_ = 0.1;
    height_meter_ = 50;
    width_meter_ = 80;
    img_height_ = height_meter_/resolution_;
    img_width_ = width_meter_/resolution_;

    // parameter to be adjusted
    max_car_height_ = 2.5;
    velodye_to_ground_ =  1.733;
    min_filter_ground_ = -0.1;

    double height_interval = 0.2;

    double current_height = 0;
    do
    {

        height_threshold_map_.push_back(current_height);
        current_height+=height_interval;

    } while (current_height <  max_car_height_);

    ROS_INFO("height_threshold_ size %d", (int) height_threshold_map_.size());



}

 void Convert_pc::convert_pc_density_img(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                    std::vector< cv::Mat > &images_array        ) {

    //

    for (size_t i = 0; i < height_threshold_map_.size();i++) {
        cv::Mat image = cv::Mat::zeros(img_height_,
                                       img_width_,
                                       CV_8UC1);
        images_array.push_back(image);
    }


    // iterate all the point through the point cloud
    for (size_t i = 0; i < input_cloud->points.size(); i++) {
        double x = input_cloud->points[i].x;
        double y = input_cloud->points[i].y;
        double z_ground = input_cloud->points[i].z + velodye_to_ground_ ;
        int img_x = x / resolution_ +  width_meter_/resolution_ / 2.0  ;
        int img_y = -y / resolution_ +  height_meter_/resolution_ / 2.0  ;

        if (img_x > 0 && img_x < img_width_ && img_y > 0 && img_y < img_height_
                && z_ground > min_filter_ground_  && z_ground < max_car_height_  ) {

            // compute height index
            for (size_t k = 0; k < height_threshold_map_.size()-1; k++ ) {
                if ( z_ground > height_threshold_map_[k] && z_ground < height_threshold_map_[k+1]) {
                   images_array[k].at<uchar>(img_y, img_x) = 255;
                   break;
                }
            }
        }
    }

    ROS_INFO("conversion done");
}


}
