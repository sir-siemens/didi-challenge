#ifndef CONVERT_PC_H
#define CONVERT_PC_H
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace didi {

class Convert_pc
{
public:
    Convert_pc();

    /**
      Init image parameters
      */
    void init();

    void convert_pc_density_img(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           std::vector< cv::Mat > &images_array  );


private:
    cv::Mat density_image_;
    int img_height_;
    int img_width_;
    double resolution_;
    double height_meter_;
    double width_meter_;
    double max_car_height_;
    double velodye_to_ground_;
    double min_filter_ground_;
    std::vector<double> height_threshold_map_;
};

}
#endif // CONVERT_PC_H
