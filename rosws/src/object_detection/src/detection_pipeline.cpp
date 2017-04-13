#include "object_detection/detection_pipeline.h"
namespace didi{

DetectionPipeline::DetectionPipeline() {
    // initialize sensor we use for detection
    boost::shared_ptr<Sensor> radar( new Radar());
    sensor_list_.push_back(radar);

    // TODO add two other modality
    //sensor_list_.push_back(camera);
    //sensor_list_.push_back(velodyne);
    // delta t
    detection_interval_ = 0.1;
    ros::NodeHandle n;
    particle_publisher_ = n.advertise<geometry_msgs::PoseArray>(
         "particles", 1);
    resampled_particle_publisher_ = n.advertise<geometry_msgs::PoseArray>(
                "resample_particles", 1);
    resampled_particle_after_motion_publisher_ = n.advertise<geometry_msgs::PoseArray>(
                "resample_particles_after_motion_update", 1);
    camera_sub_ = n.subscribe("/image_raw", 1 , &DetectionPipeline::trigger_detection_cb, this);

}

void DetectionPipeline::detect() {

    ros::WallTime begin = ros::WallTime::now();
    // 1. vehicle detection and propose new particles from the current measurement 500
    for (size_t i = 0; i < sensor_list_.size(); i++) {
        std::vector <Vehicle_model> vehicles;
        sensor_list_[i] ->detect_vehicle(vehicles);
        // TODO: merge with the current detected vehicles
        // TODO: currently we only add vehicles we should also be able to remove !!!
        // TODO: data association is not done currently
        if (detected_vehicle_list_.size() == 0 && vehicles.size() != 0 ) {
            for (size_t j = 0; j < vehicles.size(); j++ ) {
                detected_vehicle_list_.push_back(vehicles[j]);
            }
        }
    }
    // 2. create samples for each vehicle
    for (size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {
        // TODO: currently particle only have 3 dimension
        detected_vehicle_list_[i].add_particles(500);

    }
    //// DEBUG visualize particles
    visualize_particles(particle_publisher_);
    //////////////////////////////////////////

    ros::WallTime create_sample = ros::WallTime::now();
    ROS_INFO("Create initial sample takes %f second",  (create_sample - begin).toSec());

    // 3. compute weights for each particles
    for ( size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {

        for (size_t j = 0 ; j <  detected_vehicle_list_[i].particles_.size(); j++ ) {
            // TODO: combine weight for all the sensing modality w = 1/3 *w1 + 1/3 *w2 + 1/3 *w3
            double total_weight = 0;
            for (size_t k = 0; k < sensor_list_.size(); k++) {
                double weight =  sensor_list_[k]->compute_weight( detected_vehicle_list_[i].particles_[j]);
                total_weight = total_weight+=weight;
            }
            detected_vehicle_list_[i].particles_[j].weight_ = total_weight;
        }
        detected_vehicle_list_[i].normalize_weight();
    }

    ros::WallTime compute_weights = ros::WallTime::now();
    ROS_INFO("compute_weights takes %f second",  (compute_weights - create_sample).toSec());

    // 4. resampling 500 particles
    for ( size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {
        if (detected_vehicle_list_[i].all_weights_zero_ == false) {
            std::vector<int> particle_index = ParticleFilter::importance_sampling(detected_vehicle_list_[i].get_weights(),
                                                500 ,
                                                detected_vehicle_list_[i].max_weight_ );
            // update particles
            detected_vehicle_list_[i].update_particles(particle_index);
        }
    }
    ros::WallTime resample = ros::WallTime::now();
    ROS_INFO("resample takes %f second",  (resample - compute_weights).toSec());

    //// DEBUG visualize particles
    visualize_particles(resampled_particle_publisher_);
    //////////////////////////////////////////

    // 5. compute the mean and variance
    for ( size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {

        detected_vehicle_list_[i].compute_estimate();
    }
    // 6. visualize the result


    // 7. perform vehicle dynamics on particles and ego vehicle
    for ( size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {

        for (size_t j = 0 ; j < detected_vehicle_list_[i].particles_.size(); j++) {
            // update the particle pose motion prediction
            detected_vehicle_list_[i].particles_[j] = Vehicle_model::motion_model(detected_vehicle_list_[i].particles_[j],
                                                                                             detection_interval_);
            // update the particle pose due to ego motion
            geometry_msgs::Pose2D ego_pose;
            if (ego_velocity_.angular.z == 0) {
                ego_pose.x = ego_velocity_.linear.x * detection_interval_;
                ego_pose.y = 0;
            } else {
                Particle p;
                p.velocity_ = ego_velocity_;
                Particle p_aftermotion = Vehicle_model::motion_model (p , detection_interval_);
                ego_pose.x = p_aftermotion.pose_.x;
                ego_pose.y = p_aftermotion.pose_.y;
                ego_pose.theta = p_aftermotion.pose_.theta;
            }
            // transform all the particles to the new coordinate system
            geometry_msgs::Transform baseTbase_delta;
            baseTbase_delta.translation.x = ego_pose.x;
            baseTbase_delta.translation.y = ego_pose.y;
            baseTbase_delta.rotation = tf::createQuaternionMsgFromYaw (ego_pose.theta);
            geometry_msgs::Transform base_deltaTbase = inverseTransform(baseTbase_delta);
            geometry_msgs::Transform baseTparticle ;
            baseTparticle.translation.x = detected_vehicle_list_[i].particles_[j].pose_.x;
            baseTparticle.translation.y = detected_vehicle_list_[i].particles_[j].pose_.y;
            baseTparticle.rotation = tf::createQuaternionMsgFromYaw (detected_vehicle_list_[i].particles_[j].pose_.theta);
            // new particle pose
            geometry_msgs::Transform base_deltaTparticle = multiplyTransformMsg(base_deltaTbase, baseTparticle);
            // update the particle pose
            detected_vehicle_list_[i].particles_[j].pose_.x = base_deltaTparticle.translation.x;
            detected_vehicle_list_[i].particles_[j].pose_.y = base_deltaTparticle.translation.y;
            detected_vehicle_list_[i].particles_[j].pose_.theta = tf::getYaw(base_deltaTparticle.rotation);
        }
    }
    //// DEBUG visualize particles
    visualize_particles(resampled_particle_after_motion_publisher_);
    //////////////////////////////////////////
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration track_duration = end - begin;
    ROS_INFO("Detection takes %f second",  track_duration.toSec());
}

void DetectionPipeline::visualize_particles(ros::Publisher &pub) {
    geometry_msgs::PoseArray particles_proposal;
    particles_proposal.header.frame_id = "base_link";
    for (size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {
        geometry_msgs::PoseArray pa = detected_vehicle_list_[i].getParticlePoses();
        // insert to particles_proposal

        particles_proposal.poses.insert( particles_proposal.poses.end(), pa.poses.begin(), pa.poses.end() );
    }
    particles_proposal.header.stamp = current_time_;
    pub.publish(particles_proposal);
}

void DetectionPipeline::trigger_detection_cb (const sensor_msgs::Image::ConstPtr& msg) {
    // detection triggered
    current_time_ = msg->header.stamp;
    detect();
}


}
