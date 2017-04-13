#include "object_detection/vehicle_model.h"


namespace didi{

double Vehicle_model::linear_noise_ = 1.0;
double Vehicle_model::angular_noise_ = 5.0;
boost::random::mt19937 Vehicle_model::rng_;
Vehicle_model::Vehicle_model()   {

}

bool Vehicle_model::normalize_weight() {
    if (particles_.size() == 0) {
        ROS_WARN("particle size is zero");
        all_weights_zero_ = true;
        return false;
    }
    double max_weight = 0;
    double sum_weight = 0;
    for (int i = 0; i < particles_.size();i++) {
        sum_weight +=  particles_[i].weight_;
        if ( particles_[i].weight_ > max_weight) {
            max_weight = particles_[i].weight_;
        }
    }

    // normalize
    for (int i = 0; i < particles_.size();i++) {
        particles_[i].weight_ = particles_[i].weight_/max_weight;
    }

    // normalize max weight
    if (sum_weight == 0) {
        ROS_ERROR("Vehicle_model::normalize_weight():: sum weight is zero");
        all_weights_zero_ = true;
        return false;
    }
    max_weight_ = max_weight / sum_weight;
    all_weights_zero_ = false;
    return true;
}

std::vector<double> Vehicle_model::get_weights() {
    std::vector<double> weights_list(particles_.size());
    for (int i = 0 ; i < weights_list.size(); i++) {
        weights_list[i] = particles_[i].weight_;
    }
    return weights_list;
}

void Vehicle_model::update_particles(std::vector<int> particle_index) {
    std::vector< Particle > new_particle_set( particle_index.size());
    for (size_t i = 0; i < new_particle_set.size();i++){
        new_particle_set[i] = particles_[particle_index[i]];
    }

    // replace particles with new_particle_set
    particles_.resize(new_particle_set.size());
    for (size_t i = 0; i < particles_.size();i++ ) {
        particles_[i] = new_particle_set[i];
    }
}

void Vehicle_model::compute_estimate() {
    // compute the position and orientation
    double sum_x = 0;
    double sum_y = 0;
    double sum_delta_x_ = 0;
    double sum_delta_y_ = 0;

    for (size_t i = 0 ; i < particles_.size();i++ ){
        sum_x += particles_[i].pose_.x;
        sum_y += particles_[i].pose_.y;
        sum_delta_x_ +=  cos(particles_[i].pose_.theta);
        sum_delta_y_ +=  sin(particles_[i].pose_.theta);

    }
    pose_.x = sum_x/particles_.size();
    pose_.y = sum_y/particles_.size();
    pose_.theta = atan2(sum_delta_y_, sum_delta_x_);
}

Particle Vehicle_model::motion_model(Particle p, double theta_t) {
    // formula at https://people.eecs.berkeley.edu/~pabbeel/cs287-fa11/slides/velocity-motion-model.pdf

    boost::random::uniform_real_distribution<> linear_noise(-linear_noise_,linear_noise_);
    boost::random::uniform_real_distribution<> angular_noise(- angular_noise_ / 180.0 * M_PI,
                                                             angular_noise_ / 180.0 * M_PI);

    double v = p.velocity_.linear.x  +  linear_noise  (rng_);
    double w = p.velocity_.angular.z +  angular_noise(rng_);

    Particle p_after_motion;
    p_after_motion.pose_.x = p.pose_.x - v/w * sin (p.pose_.theta) + v/w * sin( p.pose_.theta + w * theta_t);
    p_after_motion.pose_.y = p.pose_.y + v/w * cos (p.pose_.theta) - v/w * cos( p.pose_.theta + w * theta_t);
    p_after_motion.pose_.theta = p.pose_.theta + w * theta_t;
    return p_after_motion;
}

void Vehicle_model::add_particles(int number ){
    // vector1.insert( vector1.end(), vector2.begin(), vector2.end() );
    std::vector<Particle> new_particles(number);
    for (size_t i = 0; i < number; i++ ) {
        Particle p;
        boost::random::uniform_real_distribution<> noise_x(-2,2);
        boost::random::uniform_real_distribution<> noise_y(-2,2);
        boost::random::uniform_real_distribution<> noise_yaw(-M_PI, M_PI);

        p.pose_.x = pose_.x + noise_x(rng_);
        p.pose_.y = pose_.y + noise_y(rng_);
        p.pose_.theta = pose_.theta + noise_yaw(rng_);
        new_particles[i] = p;
    }
    particles_.insert( particles_.end(), new_particles.begin(), new_particles.end() );
}


geometry_msgs::PoseArray Vehicle_model::getParticlePoses() {
    geometry_msgs::PoseArray pa;
    pa.poses.resize(particles_.size());
    for (size_t i = 0; i < particles_.size();i++) {
        geometry_msgs::Pose p;
        p.position.x = particles_[i].pose_.x;
        p.position.y = particles_[i].pose_.y;
        p.position.z = particles_[i].weight_;
        p.orientation = tf::createQuaternionMsgFromYaw(particles_[i].pose_.theta);
        pa.poses[i] = p;
    }
    return pa;
}






}
