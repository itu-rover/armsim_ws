#!/usr/bin/env python3

import py_trees
import py_trees_ros 
import rospy
import functools
import sys

from start_node import Initiliase_Arm
from marker_detector import Aruco_Detector
from panel_a_approach import Panel_A_Approach

def create_root():
    root = py_trees.composites.Sequence("MOTHER")
    detection = py_trees.composites.Sequence("Detection")
    
    panel_look = Initiliase_Arm(name="Start panel")
    detect_marker = Aruco_Detector(name="Detect ArUco")
    first_approach = Panel_A_Approach(name="Panel_A_Approach", move_group=panel_look.move_group)

    root.add_child(panel_look)
    root.add_child(detection)
    detection.add_child(detect_marker)
    detection.add_child(first_approach)
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__=='__main__':

    rospy.init_node("test_node")

    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))

    if not behaviour_tree.setup(timeout=15):
        py_trees.console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    behaviour_tree.tick_tock(200)