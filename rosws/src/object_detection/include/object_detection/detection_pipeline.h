#ifndef DETECTION_PIPELINE_H
#define DETECTION_PIPELINE_H
#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <object_detection/sensor.h>
#include <object_detection/radar.h>
#include <map>
#include <object_detection/vehicle_model.h>
#include <object_detection/particlefilter.h>
#include <geometry_msgs/Transform.h>
#include <object_detection/transforms.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
namespace didi{


class DetectionPipeline
{
    public:
        DetectionPipeline();

        void detect();

        void timerCallback(const ros::TimerEvent&);

        void simulatetimerCallback(const ros::TimerEvent&);

        void ego_velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

        void visualize_particles(ros::Publisher &pub);
    private:
        std::vector< boost::shared_ptr<Sensor> > sensor_list_;

        // maintain a list of detected vehicle
        std::vector<Vehicle_model> detected_vehicle_list_;

        // ego vehicle information
        double detection_interval_;
        double simualte_vehicle_kinematics_interval_;

        geometry_msgs::Twist ego_velocity_;
        geometry_msgs::Pose2D ego_pose_;

        // visualization
        ros::Publisher particle_publisher_;
        ros::Publisher resampled_particle_publisher_;
        ros::Publisher resampled_particle_after_motion_publisher_;
        ros::Publisher ego_vehicle_visualizer;

        //
        ros::Subscriber camera_sub_;
        ros::Subscriber ego_vel_sub_;
        ros::Time current_time_;
        ros::Timer timer_;
        ros::Timer simulate_vehicle_dynamics_timer_;


        geometry_msgs::Pose2D simulate_vehicle_dyamics(geometry_msgs::Pose2D current_ego_pose,
                                                       geometry_msgs::Twist current_vel,
                                                       double theta_t);
        geometry_msgs::PoseArray poses_;


        // debug ego kinematic model
        void debug_ego_kmodel();

};
}
#endif // DETECTION_PIPELINE_H
