#ifndef TRANSFORMS_H
#define TRANSFORMS_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Dense>
namespace didi{

    Eigen::Quaterniond fromGeometryMsg(const geometry_msgs::Pose &rotation);

    geometry_msgs::Pose toGeometryMsg(const Eigen::Quaterniond  &quat) ;

    /**
     * @param bInc
     * @param aInb
     * @return aInc
     */
    geometry_msgs::Pose multiplyGeometryMsg(geometry_msgs::Pose bInc,
                                            geometry_msgs::Pose aInb);


    /**
     * @param cTb
     * @param bTa
     * @return cTa
     */
    geometry_msgs::Transform multiplyTransformMsg (geometry_msgs::Transform cTb,
                                                   geometry_msgs::Transform bTa);

    geometry_msgs::Point transformPoint(geometry_msgs::Pose aTb, geometry_msgs::Point pInb);

    geometry_msgs::Point transformPoint(geometry_msgs::Transform aTb, geometry_msgs::Point pInb);

    geometry_msgs::Pose TransformtoPoseMsg(geometry_msgs::Transform transform);

    geometry_msgs::Transform PosetoTransformMsg(geometry_msgs::Pose pose);

    geometry_msgs::Transform inverseTransform(geometry_msgs::Transform transform);

    geometry_msgs::Pose inverseRosGeometryMsg(geometry_msgs::Pose pose);

    void inverseTransformation(double &pos_x, double &pos_y, double &pos_z,
                               double &rot_x, double &rot_y, double &rot_z,
                               double &rot_w) ;

    Eigen::Matrix4d TranslationMatrix(double x, double y, double z);

    void GetTranslation(double &x, double &y, double &z,
                        const Eigen::Matrix4d &transform);
}
#endif // TRANSFORMS_H
