#!/usr/bin/env python3

import rospy
import tf
from fiducial_msgs.msg import FiducialTransformArray
from scipy.spatial.transform import Rotation as R
from math import pi, fabs,cos,dist, sin
import sys
import moveit_commander
import geometry_msgs
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class Marker_Detection:
    def __init__(self):
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback=self.callback)
        moveit_commander.roscpp_initialize(sys.argv)
        self.group_name="arm26_arm"
        self.move_group=moveit_commander.MoveGroupCommander(self.group_name)

        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.trans=[]
        self.rot=[]
        self.pose_goal = geometry_msgs.msg.Pose()
        self.offset = [-0.00266295396494962, 0.465, 0.163]

    def go_to_pose_goal(self):
        self.pose_goal.position.x=self.trans[0]+self.offset[0]+0.084
        self.pose_goal.position.y=self.trans[1]-0.1+self.offset[1]
        self.pose_goal.position.z=self.trans[2]+self.offset[2]

        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
        self.move_group.set_pose_target(self.pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(self.pose_goal, current_pose, 0.01)
    

    def rotate(self):
        try:
            rpy = euler_from_quaternion(self.rot)
            self.rot = quaternion_from_euler(rpy[0], rpy[1], rpy[2]-pi/2)
        except Exception as e:
            rospy.logwarn(e)
    
    
    def create_goal(self):
        self.broadcaster.sendTransform((self.trans[0]+0.084, self.trans[1], self.trans[2]),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "main_switch",
                                            "base_link")
        
    def callback(self, msg):
        try:
            for n in range(0,len(msg.transforms)):
                if(msg.transforms[n].fiducial_id==1):
                    (self.trans, self.rot) = self.listener.lookupTransform('/base_link', '/fiducial_1', rospy.Time(0))
                    # rospy.loginfo("translation:{0} rotation{1}".format(self.trans, self.rot))
                    self.rotate()
                    self.create_goal()
                    self.broadcaster.sendTransform((self.trans[0], self.trans[1]-0.1, self.trans[2]),
                                                    (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                                    rospy.Time.now(),
                                                    "marker",
                                                    "base_link")
                    self.go_to_pose_goal()
        except:
            pass
if __name__=="__main__":    
    try:
        rospy.init_node("panel_detector")
        marker_det = Marker_Detection()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass