#include "object_detection/radar.h"

namespace didi{

Radar::Radar() {
    ros::NodeHandle n;
    sub_ = n.subscribe("/radar/points", 1,
                            &Radar::cloud_callback, this);
    radar_track_sub_ = n.subscribe("/radar/tracks", 1,
                                   &Radar::radar_tracks_callback, this);

    radar_vis_ = n.advertise<visualization_msgs::Marker>(
                "radar_measurement", 1);

    radar_cloud_recieved_ = false;
    // set detection range to 100
    radar_max_ = 100;
    merge_radar_dist_threshold_ = 5;

    angle_resolution_= 0.5; // 0.5 degree
    fov_ = 60;
    range_resolution_ = 1; // 1 m

    angle_window_ = 5 ;// +-5 degree
    range_window_  = 5 ;// +-5 m

    rader_detector_.setParam( angle_resolution_,
              fov_,
              range_resolution_,
              radar_max_);
}


void Radar::detect_vehicle(std::vector <Vehicle_model> &vehicle_list) {
    if (radar_cloud_recieved_ == false) {
        return;
    }


    // detect vehicle from the current configuration
    // vehicle_list.resize(cloud_depth_ptr_->points.size());
    for ( size_t i = 0; i < filtered_tracks_.tracks.size(); i++ ){
        Vehicle_model vm;
        // geometry_msgs::Point radar_point;
        // here x cooridnate + 0.5 target vehicle length is a prior that target vehicle
        // is drive in the same direction as ego vehicle
        geometry_msgs::Point  radar_point =  BBDetection_Radar::radar_track2position(filtered_tracks_.tracks[i]);

        radar_point.x +=  0.5 * rader_detector_.target_length();

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
    return rader_detector_.measurement_model_advanced(radar_tracks_, coorespondence_lookup_table, p);
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

void Radar::radar_tracks_callback(const radar_driver::RadarTracks::ConstPtr &msg) {
    radar_tracks_ = *msg;
    radar_cloud_recieved_ = true;
    visualize_radar_tracks(radar_tracks_, radar_vis_);
}

void Radar::preprocessing() {
    ROS_INFO("Preprocessing radar measurement ");
    filtered_tracks_ = preprocessing(radar_tracks_);
    generate_lookup_table();
}

void Radar::compute_index_in_lookuptable(double angle, double range, int& angle_index, int& range_index ) {
    int num_angle = fov_ / angle_resolution_;
    int num_range = radar_max_ / range_resolution_;
    double k = (double) num_angle / fov_;
    double b = k * (-0.5 * fov_);
    double k_range = (double) num_range / radar_max_;
    angle_index = k * (angle) + b;
    range_index = k_range * (range);
}


void Radar::generate_lookup_table() {
    // generate correspondence look up table
    int num_angle = fov_ / angle_resolution_;
    int num_range = radar_max_ / range_resolution_;
    // index 0 is angle 1 is range
    coorespondence_lookup_table.resize( num_angle , std::vector<int>(num_range , -1));

    // update correspondence map
    for (size_t i = 0; i < filtered_tracks_.tracks.size(); i++) {
        // compute index
        double range = filtered_tracks_.tracks[i].range;
        double angle = filtered_tracks_.tracks[i].angle;

        double k = (double) num_angle / fov_;
        double b = k * 0.5*fov_;
        double k_range = (double) num_range / radar_max_;

        if ( range < radar_max_ && fabs(angle) < 0.5 * fov_) {
            int angle_index = k * (angle) + b;
            int range_index = k_range * (range);
            // update the quadratic window
            int angle_begin = angle_index -  angle_window_/angle_resolution_;
            int angle_end   = angle_index +  angle_window_/angle_resolution_;
            int range_begin = range_index -  range_window_/range_resolution_;
            int range_end   = range_index +  range_window_/range_resolution_;

            if (angle_begin < 0 ) {
                angle_begin = 0;
            }
            if (angle_begin >= num_angle) {
                angle_begin = num_angle;
            }
            if (angle_end < 0 ) {
                angle_end = 0;
            }
            if (angle_end >= num_angle) {
                angle_end = num_angle;
            }

            if (range_begin < 0) {
                range_begin = 0;
            }
            if (range_begin >= num_range) {
                range_begin = num_range;
            }

            if (range_end < 0) {
                range_end = 0;
            }

            if (range_end >= num_range) {
                range_end = num_range;
            }

            for (int j = angle_begin ; j < angle_end ;j++) {
                for (int k = range_begin; k < range_end; k++) {
                    coorespondence_lookup_table[j][k] = i;
                }
            }
        }
    }
}

radar_driver::RadarTracks Radar::preprocessing(radar_driver::RadarTracks radar_raw) {
    bool merge_required = false;
    ROS_INFO("before preprocessing %d ",(int) radar_raw.tracks.size());
    do {

       merge_required =  merge_tracks( radar_raw  );

    } while (merge_required);
    ROS_INFO("after preprocessing %d", (int)radar_raw.tracks.size());
    return radar_raw;
}

void Radar::visualize_radar_tracks(radar_driver::RadarTracks radar_msg, ros::Publisher &pub) {

    for (size_t i = 0; i < radar_msg.tracks.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "radar";
        marker.header.stamp = radar_msg.header.stamp;
        marker.ns = "radar";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = BBDetection_Radar::radar_track2position( radar_msg.tracks[i]);
        double angle = atan2( radar_msg.tracks[i].late_rate,radar_msg.tracks[i].rate);
        double v_abs = sqrt (pow(radar_msg.tracks[i].late_rate, 2) + pow(radar_msg.tracks[i].rate, 2));

        marker.pose.orientation = tf::createQuaternionMsgFromYaw( angle);
        marker.scale.x = v_abs;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        pub.publish( marker );
    }
}


bool Radar:: merge_tracks(radar_driver::RadarTracks &radar_raw ) {
    for (size_t i = 0; i < radar_raw.tracks.size();i++) {
        for (size_t j = i + 1; j < radar_raw.tracks.size(); j++) {
            // compare i j to determine if a merge required
            geometry_msgs::Point track_i = BBDetection_Radar::radar_track2position(radar_raw.tracks[i]);
            geometry_msgs::Point track_j = BBDetection_Radar::radar_track2position(radar_raw.tracks[j]);
            double dist_quad = pow ((track_i.x - track_j.x) ,2 ) +  pow ((track_i.y - track_j.y) ,2 );
            if (  dist_quad < merge_radar_dist_threshold_) {
                // merge i and j and delete j
                radar_raw.tracks[i].range = 0.5 * (radar_raw.tracks[i].range + radar_raw.tracks[j].range);
                radar_raw.tracks[i].angle = 0.5 * (radar_raw.tracks[i].angle + radar_raw.tracks[j].angle);

                if (fabs( radar_raw.tracks[i].rate)  > fabs(radar_raw.tracks[j].rate )) {
                    radar_raw.tracks[i].rate = radar_raw.tracks[i].rate;
                } else {
                    radar_raw.tracks[i].rate = radar_raw.tracks[j].rate;
                }
                if (fabs( radar_raw.tracks[i].late_rate)  > fabs(radar_raw.tracks[j].late_rate) ) {
                    radar_raw.tracks[i].late_rate = radar_raw.tracks[i].late_rate;
                } else {
                    radar_raw.tracks[i].late_rate = radar_raw.tracks[j].late_rate;
                }
                // delete j
                radar_raw.tracks.erase(radar_raw.tracks.begin() + j);
                return true;
            }
        }
    }
    return false;
}

void visualize_radar_tracks(radar_driver::RadarTracks radar_msg);


}
