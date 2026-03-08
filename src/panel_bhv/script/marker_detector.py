#!/usr/bin/env python3

import py_trees
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import tf
from fiducial_msgs.msg import FiducialTransformArray

class Aruco_Detector(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, topic_name="/fiducial_transforms", topic_type=FiducialTransformArray):
        super().__init__(name, topic_name, topic_type, clearing_policy= py_trees.common.ClearingPolicy.NEVER)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.all_complete = False
        rospy.Subscriber(topic_name, topic_type, callback=self.callback)
        self.listener = tf.TransformListener()
        self.ids = [False]*4
        self.trans = [None]*4
        self.rots = [None]*4

    def callback(self, msg):
        try:
            for i in range(0, len(msg.transforms)):
                if(not msg.transforms[i].fiducial_id in[1,2,3,4]):
                    rospy.logwarn("Unwanted ArUco Marker")
                elif (msg.transforms[i].fiducial_id == 1):
                    self.ids[0] = msg.transforms[i].fiducial_id
                    (self.trans[0], self.rots[0]) = self.listener.lookupTransform('/base_link', "/fiducial_1", rospy.Time(0))    
                elif (msg.transforms[i].fiducial_id == 2):
                    self.ids[1] = msg.transforms[i].fiducial_id
                    (self.trans[1], self.rots[1]) = self.listener.lookupTransform('/base_link', "/fiducial_2", rospy.Time(0))    
                elif (msg.transforms[i].fiducial_id == 3):
                    self.ids[2] = msg.transforms[i].fiducial_id
                    (self.trans[2], self.rots[2]) = self.listener.lookupTransform('/base_link', "/fiducial_3", rospy.Time(0))    
                elif (msg.transforms[i].fiducial_id == 4):
                    self.ids[3] = msg.transforms[i].fiducial_id 
                    (self.trans[3], self.rots[3]) = self.listener.lookupTransform('/base_link', "/fiducial_4", rospy.Time(0))    
                self.blackboard.fiducials = ({"id":self.ids, "translation":self.trans, "rotation":self.rots})
        except Exception as e:
            rospy.logerr(e)
    def update(self):
        if not all(self.ids):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS