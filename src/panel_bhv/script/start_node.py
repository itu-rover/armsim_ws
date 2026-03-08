#!/usr/bin/env python3

import py_trees
import moveit_commander
import sys
import rospy 
class Initiliase_Arm(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, group_name="arm26_arm"):
        super().__init__(name, group_name)
        self.group_name = group_name
        self.success = False
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group=moveit_commander.MoveGroupCommander(self.group_name)
        rospy.loginfo("Init")
    
    def update(self):
        
        if not self.success:
            self.move_group.set_named_target("look_panel")
            self.success = self.move_group.go(wait = True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.success = True
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
            