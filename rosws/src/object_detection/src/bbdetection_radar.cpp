#include "object_detection/bbdetection_radar.h"

namespace didi {

BBDetection_Radar::BBDetection_Radar()
{
    // initialize some parameter
    target_w_ = 2.0;
    target_l_ = 5.0;
    fov_ =  M_PI/3.0;
    param_sigma_gaussian_ = 2.0;
    vehicle_radius_ = 2;
    base_linkTradar_ego_.translation.x = 3.8;
    base_linkTradar_ego_.rotation.w = 1.0;
    radarTbaselink_ego_ = inverseTransform(base_linkTradar_ego_);
    setBBCorner(target_l_,target_w_);


    //

}

void BBDetection_Radar::setBBCorner(double length, double width) {
    fl_.x = length/2.0;
    fl_.y = width/2.0;
    fr_.x = length/2.0;
    fr_.y = -width/2.0;
    bl_.x = -length/2.0;
    bl_.y = width/2.0;
    br_.x = -length/2.0;
    br_.y = -width/2.0;
}

radar_driver::Track BBDetection_Radar::find_correspondence(geometry_msgs::Point desired_measurement) {
    radar_driver::Track coorespond;
    // read correspondence from the lookup table


    return coorespond;
}


double BBDetection_Radar::find_correspondence_and_prob(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                                         geometry_msgs::Point desired_measurement) {
    double min_distance = 10000;
    for (size_t i = 0; i < input_radar_cloud->points.size();i++) {
        double radar_x = input_radar_cloud->points[i].x;
        double radar_y = input_radar_cloud->points[i].y;

        double distance_quad = (radar_x - desired_measurement.x) * (radar_x - desired_measurement.x)
                + (radar_y - desired_measurement.y) * (radar_y - desired_measurement.y);

        double distance = sqrt(distance_quad);
        if (distance <  min_distance) {
            min_distance = distance;
        }
    }

    // return probability of gaussian N(0, 3m)
    double prob = 1.0 / (param_sigma_gaussian_ * sqrt(2*M_PI) )  * exp( -0.5* pow(min_distance/param_sigma_gaussian_ ,2));
    return prob;
}



double BBDetection_Radar::measurement_model_simple(radar_driver::RadarTracks filtered_radar_measurement,
                                geometry_msgs::Pose2D ego_T_target) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    radar_cloud->points.resize(filtered_radar_measurement.tracks.size());
    for (size_t i = 0 ; i < filtered_radar_measurement.tracks.size(); i++) {
        double range = filtered_radar_measurement.tracks[i].range;
        double angle = -filtered_radar_measurement.tracks[i].angle / 180.0 * M_PI;
        radar_cloud->points[i].x = range * cos(angle);
        radar_cloud->points[i].y = range * sin(angle);
    }
    return measurement_model_simple(radar_cloud, ego_T_target);



}

double BBDetection_Radar::measurement_model_advanced(const radar_driver::RadarTracks &filtered_tracks,
                                                     const std::vector< std::vector<int> >  &coorespondence_lookup_table,
                                                     Particle p) {


    geometry_msgs::Pose2D ego_T_target;
    ego_T_target.x = p.pose_.x;
    ego_T_target.y = p.pose_.y;
    ego_T_target.theta = p.pose_.theta;
    // compute desired measurement z^

    // transform target in radar link
    geometry_msgs::Transform egoTtarget_tr;
    egoTtarget_tr.translation.x = ego_T_target.x;
    egoTtarget_tr.translation.y = ego_T_target.y;
    egoTtarget_tr.rotation = tf::createQuaternionMsgFromYaw (ego_T_target.theta);

    geometry_msgs::Transform radarTtarget = multiplyTransformMsg(radarTbaselink_ego_,
                                                                 egoTtarget_tr) ;
    // determine if target vehicle in fov if not return 0 or a very small number
    float yaw_angle = atan2(radarTtarget.translation.y, radarTtarget.translation.x);
    if ( fabs(yaw_angle) > fov_/2.0 ) {
        return 0.0;
    }
    // assume vehicle is a circle with radius
    double t =  1 -   vehicle_radius_ / sqrt( pow(radarTtarget.translation.x,2) + pow(radarTtarget.translation.y,2) );
    if (t > 0.0 && t < 1.0) {
        double desired_measurement_x = t * radarTtarget.translation.x;
        double desired_measurement_y = t * radarTtarget.translation.y;
        geometry_msgs::Point desired_measurement;
        desired_measurement.x = desired_measurement_x;
        desired_measurement.y = desired_measurement_y;
        double desired_range = sqrt (desired_measurement_x * desired_measurement_x + desired_measurement_y * desired_measurement_y);
        double desired_angle = -atan2(desired_measurement_y, desired_measurement_x); // minus because in look up table positive axis of radar measurement is inversed
        // compute prob w1: position w2: orientation w3: velocity
        double desired_angle_degree = desired_angle * 180 / M_PI;

        int angle_index = -1;
        int range_index = -1;
        compute_index_in_lookuptable(  desired_angle_degree, desired_range , angle_index, range_index);
        // TODOradar_driver::Track correspondence =
        bool correspondence_found = false;
        radar_driver::Track correspondence_track;
        if (angle_index >= 0 && angle_index < (int)fov_degree_ / angle_resolution_ &&
                range_index >= 0 && range_index < (int)range_max_/ range_resolution_) {
            int track_index = coorespondence_lookup_table[angle_index][range_index] ;
            if (track_index != -1) {
                // found correspondence
                if (track_index >= filtered_tracks.tracks.size() ) {
                    ROS_ERROR("track index is bigger than track size");
                } else {
                    correspondence_track = filtered_tracks.tracks[track_index];
                    correspondence_found = true;
                }
            }
        }

        // return find_correspondence_and_prob(input_radar_cloud, desired_measurement);
        if (correspondence_found) {
            // compute p1 p2 p3
            // euclidean distance between desired point and
            geometry_msgs::Point measurement = radar_track2position(correspondence_track);

            double distance = sqrt ( pow(( measurement.x - desired_measurement.x),2 ) + pow( (measurement.y - desired_measurement.y),2 ) );

            double prob = 1.0 / (param_sigma_gaussian_ * sqrt(2*M_PI) )  * exp( -0.5* pow(distance/param_sigma_gaussian_ ,2));

            // double v_x = p.velocity_.linear.x;

            // double v_measurementInbaselink   = correspondence_track.rate

            return prob;

        } else {
            return 0.0;
        }
    } else {
        return 0;
    }



}


double BBDetection_Radar::measurement_model_simple(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud,
                                geometry_msgs::Pose2D ego_T_target ) {
    // transform target in radar link
    geometry_msgs::Transform egoTtarget_tr;
    egoTtarget_tr.translation.x = ego_T_target.x;
    egoTtarget_tr.translation.y = ego_T_target.y;
    egoTtarget_tr.rotation = tf::createQuaternionMsgFromYaw (ego_T_target.theta);


    geometry_msgs::Transform radarTtarget = multiplyTransformMsg(radarTbaselink_ego_,
                                                                 egoTtarget_tr) ;
    // determine if target vehicle in fov if not return 0 or a very small number
    float yaw_angle = atan2(radarTtarget.translation.y, radarTtarget.translation.x);
    if ( fabs(yaw_angle) > fov_/2.0 ) {
        return 0.0;
    }
    // assume vehicle is a circle with radius

    double t =  1 -   vehicle_radius_ / sqrt( pow(radarTtarget.translation.x,2) + pow(radarTtarget.translation.y,2) );
    if (t > 0.0 && t < 1.0) {
        double desired_measurement_x = t * radarTtarget.translation.x;
        double desired_measurement_y = t * radarTtarget.translation.y;
        geometry_msgs::Point desired_measurement;
        desired_measurement.x = desired_measurement_x;
        desired_measurement.y = desired_measurement_y;
        return find_correspondence_and_prob(input_radar_cloud, desired_measurement);
    } else {
        return 0;
    }
}


double BBDetection_Radar::measurement_model( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                          geometry_msgs::Pose2D ego_T_target
                                             ) {
    // transform target in radar link
    geometry_msgs::Transform egoTtarget_tr;
    egoTtarget_tr.translation.x = ego_T_target.x;
    egoTtarget_tr.translation.y = ego_T_target.y;
    egoTtarget_tr.rotation = tf::createQuaternionMsgFromYaw (ego_T_target.theta);


    geometry_msgs::Transform radarTtarget = multiplyTransformMsg(radarTbaselink_ego_,
                                                                 egoTtarget_tr) ;
    // determine if target vehicle in fov if not return 0 or a very small number
    float yaw_angle = atan2(radarTtarget.translation.y, radarTtarget.translation.x);
    if ( fabs(yaw_angle) > fov_/2.0 ) {
        return 0.0;
    }

    // compute the corner point in radar coordiate
    geometry_msgs::Point flIntarget;
    flIntarget.x = fl_.x;
    flIntarget.y = fl_.y;
    geometry_msgs::Point flInradar = transformPoint(radarTtarget, flIntarget );

    geometry_msgs::Point frIntarget;
    frIntarget.x = fr_.x;
    frIntarget.y = fr_.y;
    geometry_msgs::Point frInradar = transformPoint(radarTtarget, frIntarget );

    geometry_msgs::Point blIntarget;
    blIntarget.x = bl_.x;
    blIntarget.y = bl_.y;
    geometry_msgs::Point blInradar = transformPoint(radarTtarget, blIntarget );

    geometry_msgs::Point brIntarget;
    brIntarget.x = br_.x;
    brIntarget.y = br_.y;
    geometry_msgs::Point brInradar = transformPoint(radarTtarget, brIntarget );

    // compute the intersection of bounding box to radar ray
    // line segment 1 pt_origin- pt_targetInrader
    geometry_msgs::Point pt_origin;
    geometry_msgs::Point pt_targetInrader;
    pt_targetInrader.x = radarTtarget.translation.x;
    pt_targetInrader.y = radarTtarget.translation.y;

    // check intersection
    geometry_msgs::Point intersection_fl_fr;
    bool check1 = intersect(pt_origin,pt_targetInrader,
                            flInradar, frInradar, intersection_fl_fr);
    geometry_msgs::Point intersection_fr_br;
    bool check2 = intersect(pt_origin,pt_targetInrader,
                            frInradar, brInradar, intersection_fr_br);
    geometry_msgs::Point intersection_br_bl;
    bool check3 = intersect(pt_origin,pt_targetInrader,
                            brInradar, blInradar, intersection_br_bl);
    geometry_msgs::Point intersection_bl_fl;
    bool check4 = intersect(pt_origin,pt_targetInrader,
                            blInradar, flInradar, intersection_bl_fl);
    // ROS_DEBUG("check 1 %b", check1);
    // ROS_DEBUG("check 2 %b", check2);
    // ROS_DEBUG("check 3 %b", check3);
    // ROS_DEBUG("check 4 %b", check4);

    if (check1) {
        // find correspondence
        return find_correspondence_and_prob(input_cloud,intersection_fl_fr);
    }
    if (check2) {
        // find correspondence
        return find_correspondence_and_prob(input_cloud,intersection_fr_br);
    }
    if (check3) {
        return find_correspondence_and_prob(input_cloud,intersection_br_bl);
    }
    if (check4) {
        return find_correspondence_and_prob(input_cloud,intersection_bl_fl);
    }
    return 0.0;

}



std::vector<geometry_msgs::Pose2D> BBDetection_Radar::vehicles_proposal(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_radar_cloud) {
    std::vector<geometry_msgs::Pose2D> particles;


    return particles;
}


bool BBDetection_Radar::intersect(geometry_msgs::Point line1_seg_pt1,
                                  geometry_msgs::Point line1_seg_pt2,
                                  geometry_msgs::Point line2_seg_pt1,
                                  geometry_msgs::Point line2_seg_pt2,
                                  geometry_msgs::Point &intersection) {
    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    Eigen::Vector2d p;
    p[0] = line1_seg_pt1.x;
    p[1] = line1_seg_pt1.y;
    Eigen::Vector2d r;
    r[0] = line1_seg_pt2.x - line1_seg_pt1.x;
    r[1] = line1_seg_pt2.y - line1_seg_pt1.y;
    Eigen::Vector2d q;
    q[0] = line2_seg_pt1.x;
    q[1] = line2_seg_pt1.y;
    Eigen::Vector2d s;
    s[0] = line2_seg_pt2.x - line2_seg_pt1.x;
    s[1] = line2_seg_pt2.y - line2_seg_pt1.y;

    // solving t and u
    // t = (q − p) × s / (r × s)
    // u = (q − p) × r / (r × s)

    double r_cross_s = cross2d(r,s);
    Eigen::Vector2d q_minus_p = q - p ;
    double q_minus_p_cross_s = cross2d (q_minus_p,s);
    double q_minus_p_cross_r = cross2d (q_minus_p,r);

    if ( r_cross_s != 0.0) {
        double t = q_minus_p_cross_s / r_cross_s;
        double u = q_minus_p_cross_r / r_cross_s;
        if ( t <= 1.0 && t >= 0.0 && u >= 0.0 && u <=1.0 ) {
            Eigen::Vector2d is =  p + t*r;
            intersection.x = is[0];
            intersection.y = is[1];
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

double BBDetection_Radar::cross2d(Eigen::Vector2d v, Eigen::Vector2d w) {
    return v[0] * w[1] - v[1] * w[0] ;
}


geometry_msgs::Point BBDetection_Radar::transform_radar_ptInEgo_baselink(geometry_msgs::Point pInradar) {
    geometry_msgs::Point pointInbaselink = transformPoint(base_linkTradar_ego_, pInradar);
    return pointInbaselink;

}

void BBDetection_Radar::compute_index_in_lookuptable( double angle, double range, int& angle_index, int& range_index ) {
    int num_angle = fov_degree_ / angle_resolution_;
    int num_range = range_max_ / range_resolution_;
    double k = (double) num_angle / fov_degree_;
    double b = k * (0.5 * fov_degree_);
    double k_range = (double) num_range / range_max_;
    angle_index = k * (angle) + b;
    range_index = k_range * (range);
}


geometry_msgs::Point BBDetection_Radar::radar_track2position( radar_driver::Track track) {
    double range = track.range;
    double angle = -track.angle / 180.0 * M_PI;
    geometry_msgs::Point p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    return p;
}

}


