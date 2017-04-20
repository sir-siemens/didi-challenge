#ifndef RADAR_H
#define RADAR_H
#include <object_detection/sensor.h>
#include <object_detection/bbdetection_radar.h>
#include <radar_driver/RadarTracks.h>
#include <visualization_msgs/Marker.h>

namespace didi{

class Radar : public Sensor
{
public:
    Radar();

    virtual void detect_vehicle(std::vector <Vehicle_model> &vehicle_list);

    virtual double compute_weight( Particle p) ;

    virtual void preprocessing();

    void cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg);

    void radar_tracks_callback(const radar_driver::RadarTracks::ConstPtr &msg);

    // generate a table for correspondence finding
    void generate_lookup_table();

    void compute_index_in_lookuptable( double angle, double range, int& angle_index, int& range_index );

    radar_driver::RadarTracks preprocessing(radar_driver::RadarTracks radar_raw);

    void visualize_radar_tracks(radar_driver::RadarTracks radar_msg, ros::Publisher &pub);

    /**

        Merge radar measurement close to each other

      */

    bool merge_tracks(radar_driver::RadarTracks &radar_raw );


private:
    ros::Subscriber sub_;
    ros::Subscriber radar_track_sub_;
    BBDetection_Radar rader_detector_;
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth_ptr_;
    ros::Publisher radar_vis_;
    bool radar_cloud_recieved_;
    double radar_max_;
    double merge_radar_dist_threshold_;
    radar_driver::RadarTracks radar_tracks_;
    radar_driver::RadarTracks filtered_tracks_;
    std::vector< std::vector<int> >  coorespondence_lookup_table;
    // parameters
    double angle_resolution_; // 0.5 degree
    double fov_ ;
    double range_resolution_;
    double angle_window_;
    double range_window_;

};

}
#endif // RADAR_H
