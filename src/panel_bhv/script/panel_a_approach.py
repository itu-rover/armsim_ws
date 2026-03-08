#!/usr/bin/env python3

import typing
import rospy
import py_trees
import tf
from math import pi, fabs,cos,dist, sin
import geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import copy

class Panel_A_Approach(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, move_group):
            super().__init__(name, move_group)
            self.move_group = move_group
            self.pose_goal = geometry_msgs.msg.Pose()
            self.blackboard = py_trees.blackboard.Blackboard()
            self.broadcaster = tf.TransformBroadcaster()
            self.goal_pos = []
            self.offset = [0, 0, 0]  # arm26 base is at world origin
            self.trans=[]
            self.rot=[]
            self.finished = False
            rospy.loginfo("Start")

    def state(self):
        rospy.loginfo("state started")
        #Approach Main button
        self.pose_goal.position.x=self.goal_pos[0][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[0][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[0][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0][0]
        self.pose_goal.orientation.y=self.rot[0][1]
        self.pose_goal.orientation.z=self.rot[0][2]
        self.pose_goal.orientation.w=self.rot[0][3]
           
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(0.1)

        #Press Main button
        self.create_goal()
        self.rotate()

        self.pose_goal.position.x=self.goal_pos[0][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[0][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[0][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0][0]
        self.pose_goal.orientation.y=self.rot[0][1]
        self.pose_goal.orientation.z=self.rot[0][2]
        self.pose_goal.orientation.w=self.rot[0][3]
           
        waypoints = []
        wpose = self.pose_goal
        wpose.position.x = wpose.position.x + 0.01
        wpose.position.z = wpose.position.z - 0.01
        wpose.position.y = wpose.position.y + 0.16

        waypoints.append(copy.deepcopy(wpose))

        self.broadcaster.sendTransform((wpose.position.x, wpose.position.y+0.16, wpose.position.z),
                                        (wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w),
                                        rospy.Time.now(),
                                        "cartesian_path",
                                        "base_link")
        
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.1, False)
        self.move_group.execute(plan, wait=True)            

        success =  True
        rospy.loginfo("state done")
        return success

    def create_goal(self):
        self.trans = self.blackboard.fiducials.get("translation")

        self.goal_pos = [[self.trans[0][0]+0.084, self.trans[0][1]-0.3, self.trans[0][2]]]

    def rotate(self):
        rospy.logwarn("Rotate")
        self.rot = self.blackboard.fiducials.get("rotation")
        try:
            rpy = euler_from_quaternion(self.rot[0])
            self.rot[0] = quaternion_from_euler(rpy[0], rpy[1], rpy[2]-pi/2)

            self.broadcaster.sendTransform((self.trans[0][0]+0.084, self.trans[0][1]-0.3, self.trans[0][2]),
                                            (self.rot[0][0], self.rot[0][1], self.rot[0][2], self.rot[0][3]),
                                            rospy.Time.now(),
                                            "main_switch",
                                            "base_link")
        except Exception as e:
            rospy.logwarn(e)

    def update(self):
        if not self.finished: 
            self.create_goal()
            self.rotate() 
            self.finished = self.state()
            return py_trees.common.Status.RUNNING
        else:
            self.broadcaster.sendTransform((self.trans[0][0]+0.084, self.trans[0][1], self.trans[0][2]),
                                            (self.rot[0][0], self.rot[0][1], self.rot[0][2], self.rot[0][3]),
                                            rospy.Time.now(),
                                            "main_switch",
                                            "base_link")
            return py_trees.common.Status.SUCCESS