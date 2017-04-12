#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

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

    void create_particles(int number);

    geometry_msgs::Twist velocity_;
    geometry_msgs::Pose2D pose_;
    geometry_msgs::Vector3 boundingbox_;

    // particles which represent the vehicle distribution
    std::vector< Particle > particles_;



};
}
#endif // VEHICLE_MODEL_H
