#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
import tf
import math
import tf.transformations as tf_tr
import numpy


class TF_gps():
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.sub_gps_front = rospy.Subscriber("/objects/capture_vehicle/front/gps/rtkfix", Odometry, self.callback_ego_front_gps)
        self.sub_gps_rear  = rospy.Subscriber("/objects/capture_vehicle/rear/gps/rtkfix", Odometry, self.callback_ego_rear_gps)
        self.sub_gps_target = rospy.Subscriber("/objects/obs1/rear/gps/rtkfix", Odometry, self.callback_target_gps)
           
        self.ego_front = None
        self.ego_rear = None  
        self.rearTbase_footprint = None
        self.get_transform_rearTbase_footprint()


    def get_transform_rearTbase_footprint(self):
        listener = tf.TransformListener()
        # listener.waitForTransform("/gps_antenna_rear", "/base_footprint", rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                listener.waitForTransform("/gps_antenna_rear", "/base_footprint", now, rospy.Duration(1))
                self.rearTbase_footprint = listener.lookupTransform("/gps_antenna_rear", "/base_footprint", now) 
                break
            except (tf.Exception):
                print ('wait for bag file to play')                
                continue
         


    def callback_ego_front_gps(self, msg):
        self.ego_front = msg


    def callback_ego_rear_gps(self, msg):
        self.ego_rear = msg

        # tf listener 
        
        if self.rearTbase_footprint!=None:
            if  self.ego_front != None:
                # compute orientation
                
                delta_y = self.ego_front.pose.pose.position.y - self.ego_rear.pose.pose.position.y
                delta_x = self.ego_front.pose.pose.position.x - self.ego_rear.pose.pose.position.x
                yaw = math.atan2(delta_y, delta_x)
                print "yaw :", yaw
                gpsTrear_t = (self.ego_rear.pose.pose.position.x,
                       self.ego_rear.pose.pose.position.y,
                       self.ego_rear.pose.pose.position.z)    
    
                gpsTrear_q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                
                
                # compute gpsTbase_footprint = gpsTrear_antenna * rear_antennaTbase_footprint
                (gpsTbase_footprint_t , gpsTbase_footprint_q ) = self.multiplyTF(gpsTrear_t,
                                                                                 gpsTrear_q,
                                                                                 self.rearTbase_footprint[0],
                                                                                 self.rearTbase_footprint[1])

                self.br.sendTransform(gpsTbase_footprint_t ,
                                      gpsTbase_footprint_q ,
                                      msg.header.stamp,
                                      "base_footprint",
                                      "gps")


    def callback_target_gps (self, msg):
        pos = (msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z)    
        self.br.sendTransform(pos,
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             msg.header.stamp,
                             "target_vehicle",
                             "gps")
        
    # return aTc_t, aTc_q
    def multiplyTF(self, aTb_t , aTb_q, bTc_t, bTc_q ):
        aTb_t_mat = tf.transformations.translation_matrix(aTb_t)
        aTb_q_mat   = tf.transformations.quaternion_matrix(aTb_q)
        aTb_mat = numpy.dot(aTb_t_mat, aTb_q_mat)

        bTc_t_mat = tf.transformations.translation_matrix(bTc_t)
        bTc_q_mat   = tf.transformations.quaternion_matrix(bTc_q)
        bTc_mat = numpy.dot(bTc_t_mat, bTc_q_mat)
        
        aTc_mat = numpy.dot(aTb_mat, bTc_mat)
        aTc_t = tf.transformations.translation_from_matrix(aTc_mat)
        aTc_q = tf.transformations.quaternion_from_matrix(aTc_mat)
        return (aTc_t, aTc_q)

if __name__ == '__main__':

    rospy.init_node('tf_gps_publisher', anonymous=False)
        
    tf_gps = TF_gps()

    rospy.spin()

