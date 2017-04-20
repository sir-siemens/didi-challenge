#ifndef BBDETECTION_RADAR_H
#define BBDETECTION_RADAR_H
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <geometry_msgs/Transform.h>
#include <object_detection/transforms.h>
#include <tf/transform_datatypes.h>
#include <radar_driver/RadarTracks.h>
#include <object_detection/vehicle_model.h>

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


    double measurement_model_simple(radar_driver::RadarTracks filtered_radar_measurement,
                                    geometry_msgs::Pose2D ego_T_target);


    double measurement_model_advanced(const radar_driver::RadarTracks &filtered_tracks,
                                      const std::vector< std::vector<int> >  &coorespondence_lookup_table,
                                      Particle p);


    std::vector<geometry_msgs::Pose2D> vehicles_proposal(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud);


    static geometry_msgs::Point radar_track2position( radar_driver::Track track) ;

    /**
      Set the bounding box corner of the vehicle
      */
    void setBBCorner(double length, double width);

    /**
      find the nearest radar point in the cloud and
      */
    double find_correspondence_and_prob(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                                             geometry_msgs::Point desired_measurement);

    radar_driver::Track find_correspondence(geometry_msgs::Point desired_measurement);


    double target_width() { return target_w_;}
    double target_length() { return target_l_;}

    // transform a point detected from radar link to base link
    geometry_msgs::Point transform_radar_ptInEgo_baselink(geometry_msgs::Point pInradar);

    /**
      Set initial paramter
      */
    void setParam(double angle_resolution,
                  double fov_degree,
                  double range_resolution,
                  double range_max) {
        angle_resolution_ = angle_resolution;
        fov_degree_ = fov_degree;
        range_resolution_ = range_resolution;
        range_max_ = range_max;
    }

    /**
      Compute the index for look up
      angle : in degree
      range: in meter
      */
    void compute_index_in_lookuptable( double angle, double range, int& angle_index, int& range_index );




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

    double angle_resolution_; // 0.5 degree
    double fov_degree_ ;
    double range_resolution_;
    double range_max_;

};
}
#endif // BBDETECTION_RADAR_H
