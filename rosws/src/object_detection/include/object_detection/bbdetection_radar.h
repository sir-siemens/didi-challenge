#ifndef BBDETECTION_RADAR_H
#define BBDETECTION_RADAR_H
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <geometry_msgs/Transform.h>
#include <object_detection/transforms.h>
#include <tf/transform_datatypes.h>
namespace didi {

class BBDetection_Radar
{
public:
    BBDetection_Radar();

    /**
      return the probability of p(z|x),
      given vehicle pose,
      compute desired measurement z and evaluate with the true measurement
      **/
    double measurement_model( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                              geometry_msgs::Pose2D ego_T_target);

    /**
      A simple measurement model which assumes the vehicle is modelled as a circle
      */
    double measurement_model_simple(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                                    geometry_msgs::Pose2D ego_T_target );

    std::vector<geometry_msgs::Pose2D> vehicles_proposal(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud);


    /**
      Set the bounding box corner of the vehicle
      */
    void setBBCorner(double length, double width);

    /**
      find the nearest radar point in the cloud and
      */
    double find_correspondence_and_prob(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                                             geometry_msgs::Point desired_measurement);

    double target_width() { return target_w_;}
    double target_length() { return target_l_;}

    // transform a point detected from radar link to base link
    geometry_msgs::Point transform_radar_ptInEgo_baselink(geometry_msgs::Point pInradar);


private:

    // parameters
    float target_w_;
    float target_l_;
    float fov_;
    double param_sigma_gaussian_;

    // outscribe radius
    double vehicle_radius_;
    //
    geometry_msgs::Transform base_linkTradar_ego_;
    geometry_msgs::Transform radarTbaselink_ego_;
    // corner point of a vehicle
    geometry_msgs::Point fl_;
    geometry_msgs::Point fr_;
    geometry_msgs::Point bl_;
    geometry_msgs::Point br_;

    // helper functions

    bool intersect(geometry_msgs::Point line1_seg_pt1,
                   geometry_msgs::Point line1_seg_pt2,
                   geometry_msgs::Point line2_seg_pt1,
                   geometry_msgs::Point line2_seg_pt2,
                   geometry_msgs::Point &intersection);

    double cross2d(Eigen::Vector2d v, Eigen::Vector2d w);



};
}
#endif // BBDETECTION_RADAR_H
