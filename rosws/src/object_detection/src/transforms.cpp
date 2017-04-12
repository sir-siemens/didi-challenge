#include "object_detection/transforms.h"

namespace didi{

Eigen::Quaterniond fromGeometryMsg(const geometry_msgs::Pose &rotation){
  Eigen::Quaterniond q;
  q.x() = rotation.orientation.x;
  q.y() = rotation.orientation.y;
  q.z() = rotation.orientation.z;
  q.w() = rotation.orientation.w;
  return q;
}


geometry_msgs::Pose toGeometryMsg(const Eigen::Quaterniond  &quat) {
  geometry_msgs::Pose p;
  p.orientation.x = quat.x();
  p.orientation.y = quat.y();
  p.orientation.z = quat.z();
  p.orientation.w = quat.w();
  return p;
}

geometry_msgs::Transform multiplyTransformMsg (geometry_msgs::Transform cTb,
                                               geometry_msgs::Transform bTa) {
    geometry_msgs::Pose bInc = TransformtoPoseMsg(cTb);
    geometry_msgs::Pose aInb = TransformtoPoseMsg(bTa);
    geometry_msgs::Pose aInc = multiplyGeometryMsg(bInc, aInb);
    return PosetoTransformMsg(aInc);
}

geometry_msgs::Pose TransformtoPoseMsg(geometry_msgs::Transform transform) {
    geometry_msgs::Pose p;
    p.position.x = transform.translation.x;
    p.position.y = transform.translation.y;
    p.position.z = transform.translation.z;
    p.orientation.x = transform.rotation.x;
    p.orientation.y = transform.rotation.y;
    p.orientation.z = transform.rotation.z;
    p.orientation.w = transform.rotation.w;
    return p;
}

geometry_msgs::Transform PosetoTransformMsg(geometry_msgs::Pose pose) {
   geometry_msgs::Transform tr;
   tr.translation.x = pose.position.x;
   tr.translation.y = pose.position.y;
   tr.translation.z = pose.position.z;
   tr.rotation.x = pose.orientation.x;
   tr.rotation.y = pose.orientation.y;
   tr.rotation.z = pose.orientation.z;
   tr.rotation.w = pose.orientation.w;
   return tr;
}

geometry_msgs::Transform inverseTransform(geometry_msgs::Transform transform) {
    geometry_msgs::Pose msg =  TransformtoPoseMsg(transform);
    geometry_msgs::Pose inverse_msg = inverseRosGeometryMsg(msg);
    return PosetoTransformMsg(inverse_msg);
}


geometry_msgs::Pose inverseRosGeometryMsg(geometry_msgs::Pose pose) {
  inverseTransformation(pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
  return pose;
}

void inverseTransformation(double &pos_x, double &pos_y, double &pos_z,
                           double &rot_x, double &rot_y, double &rot_z,
                           double &rot_w) {
  Eigen::Quaterniond q(rot_w, rot_x, rot_y, rot_z);
  Eigen::Isometry3d q_transform(q);

  Eigen::Matrix4d q_mat = q_transform.matrix();
  Eigen::Matrix4d t = TranslationMatrix(pos_x, pos_y, pos_z);
  Eigen::Matrix4d tf = t * q_mat;
  Eigen::Matrix4d tf_inv = tf.inverse();
  GetTranslation(pos_x, pos_y, pos_z, tf_inv);

  // normalize q
  q.normalize();
  Eigen::Quaterniond q_inverse = q.conjugate();
  rot_x = q_inverse.x();
  rot_y = q_inverse.y();
  rot_z = q_inverse.z();
  rot_w = q_inverse.w();
}


geometry_msgs::Pose multiplyGeometryMsg(geometry_msgs::Pose bInc,
                                        geometry_msgs::Pose aInb) {
    Eigen::Quaterniond q_bInc =  fromGeometryMsg(bInc);
    Eigen::Quaterniond q_aInb =  fromGeometryMsg(aInb);
    Eigen::Quaterniond q_aInc = q_bInc * q_aInb;
    q_aInc.normalize();

    Eigen::Quaterniond tba;
    tba.x() = aInb.position.x;
    tba.y() = aInb.position.y;
    tba.z() = aInb.position.z;
    tba.w() = 0.0;

    Eigen::Vector3d tcb;
    tcb(0) = bInc.position.x;
    tcb(1) = bInc.position.y;
    tcb(2) = bInc.position.z;

    Eigen::Quaterniond tba_rot = q_bInc * tba * q_bInc.inverse();
    Eigen::Vector3d tba_vec;
    tba_vec(0)=tba_rot.x();
    tba_vec(1)=tba_rot.y();
    tba_vec(2)=tba_rot.z();

    Eigen::Vector3d aInc_vec = tba_vec + tcb;

    geometry_msgs::Pose aInc;
    aInc.position.x = aInc_vec(0);
    aInc.position.y = aInc_vec(1);
    aInc.position.z = aInc_vec(2);
    aInc.orientation.x = q_aInc.x();
    aInc.orientation.y = q_aInc.y();
    aInc.orientation.z = q_aInc.z();
    aInc.orientation.w = q_aInc.w();
    return aInc;
}

geometry_msgs::Point transformPoint(geometry_msgs::Transform aTb, geometry_msgs::Point pInb) {
    geometry_msgs::Pose aTb_pose = TransformtoPoseMsg(aTb);
    return transformPoint(aTb_pose, pInb);
}


geometry_msgs::Point transformPoint(geometry_msgs::Pose aTb, geometry_msgs::Point pInb) {
  //return transformPoint (constructMat4FromGeometryMsg(aTb) ,  pInb);
  // pIna = R_aTb * t_pInb + t_aTb
  // R_aTb = q_aTb * q_tb *q_aTb-1 + t_aTb

  Eigen::Quaterniond q_aTb =  fromGeometryMsg(aTb);
  Eigen::Quaterniond q_pInb;
  q_pInb.x() = pInb.x;
  q_pInb.y() = pInb.y;
  q_pInb.z() = pInb.z;
  q_pInb.w() = 0.0;

  Eigen::Quaterniond tba_rot = q_aTb * q_pInb * q_aTb.inverse();
  Eigen::Vector3d tba_vec;
  tba_vec(0) = tba_rot.x();
  tba_vec(1) = tba_rot.y();
  tba_vec(2) = tba_rot.z();

  Eigen::Vector3d t_aTb;
  t_aTb(0) = aTb.position.x;
  t_aTb(1) = aTb.position.y;
  t_aTb(2) = aTb.position.z;

  Eigen::Vector3d pIna_vec = tba_vec + t_aTb;
  geometry_msgs::Point pIna;
  pIna.x = pIna_vec(0);
  pIna.y = pIna_vec(1);
  pIna.z = pIna_vec(2);
  return pIna;
}

Eigen::Matrix4d TranslationMatrix(double x, double y, double z) {
  Eigen::Matrix4d tf;
  tf << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
  return tf;
}

void GetTranslation(double &x, double &y, double &z,
                    const Eigen::Matrix4d &transform) {
  x = transform(0, 3);
  y = transform(1, 3);
  z = transform(2, 3);
}

}
