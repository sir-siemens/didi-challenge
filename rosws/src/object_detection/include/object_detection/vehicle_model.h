#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <tf/transform_datatypes.h>

namespace didi{

struct Particle {
    geometry_msgs::Pose2D pose_;
    geometry_msgs::Vector3 dimension_;
    geometry_msgs::Twist velocity_;
    double weight_;
};

class Vehicle_model
{
public:
    Vehicle_model();

    // return expected pose when conduct a random noise
    static Particle motion_model(Particle p, double theta_t);


    // add particles based on the new detection
    void add_particles(int number);

    // parameter of a tracked vehicle
    geometry_msgs::Twist velocity_;
    geometry_msgs::Pose2D pose_;
    geometry_msgs::Vector3 boundingbox_;

    // normalize the current particles weights so that the weights summing up to 1
    bool normalize_weight();

    // update the member variable particles_ with the paritcle index
    void update_particles(std::vector<int> particle_index);

    // compute the estimate from the particles
    void compute_estimate();

    geometry_msgs::PoseArray getParticlePoses();

    std::vector<double> get_weights();
    std::vector< Particle > particles_;
    double max_weight_;
    static boost::random::mt19937 rng_;

    // parameter of motion noise
    static double linear_noise_;  // 1.0 m/s
    static double angular_noise_; // 5.0 grad

    bool all_weights_zero_;
};


}
#endif // VEHICLE_MODEL_H
