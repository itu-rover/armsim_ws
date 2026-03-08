#!/usr/bin/env python3

import rospy
import tf
from fiducial_msgs.msg import FiducialTransformArray
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
        self.pose_goal = geometry_msgs.msg.Pose()
        self.offset = [-0.00266295396494962, 0.465, 0.163]
        self.trans = []
        self.rot = []
        self.goal_pos = []
        self.points_created = False
        self.accomp = [False, False, False, False, False]

    def rotate(self):
        try:
            rpy = euler_from_quaternion(self.rot)
            self.rot = quaternion_from_euler(rpy[0], rpy[1], rpy[2]-pi/2)
        except Exception as e:
            rospy.logwarn(e)
    
    def state1(self):
        rospy.loginfo("state1 started")
        self.pose_goal.position.x=self.goal_pos[0][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[0][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[0][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
           
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        rospy.loginfo(success)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.accomp[0] = success
        rospy.loginfo("state1 done")
    
    def state2(self):
        rospy.loginfo("state2 started")
        self.pose_goal.position.x=self.goal_pos[1][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[1][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[1][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
        
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        rospy.loginfo(success)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.accomp[1] = success
        rospy.loginfo("state2 done")
        
    def state3(self):
        rospy.loginfo("state3 started")
        self.pose_goal.position.x=self.goal_pos[2][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[2][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[2][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
        
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        rospy.loginfo(success)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.accomp[2] = success
        rospy.loginfo("state3 done")

    def state4(self):
        rospy.loginfo("state4 started")
        self.pose_goal.position.x=self.goal_pos[3][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[3][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[3][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
        
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        rospy.loginfo(success)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.accomp[3] = success
        rospy.loginfo("state4 done")
        
    def state5(self):
        rospy.loginfo("state5 started")
        self.pose_goal.position.x=self.goal_pos[4][0]+self.offset[0]
        self.pose_goal.position.y=self.goal_pos[4][1]+self.offset[1]
        self.pose_goal.position.z=self.goal_pos[4][2]+self.offset[2]
        
        self.pose_goal.orientation.x=self.rot[0]
        self.pose_goal.orientation.y=self.rot[1]
        self.pose_goal.orientation.z=self.rot[2]
        self.pose_goal.orientation.w=self.rot[3]
        
        self.move_group.set_pose_target(self.pose_goal)
        
        success = self.move_group.go(wait=True)
        rospy.loginfo(success)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("state5 done")
        
    def goal_points(self):
        self.broadcaster.sendTransform((self.trans[0]+0.084, self.trans[1]-0.2, self.trans[2]),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "main_switch",
                                            "base_link")

        self.broadcaster.sendTransform((self.trans[0]+0.084, self.trans[1]-0.2, self.trans[2]-0.071),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "button_2",
                                            "base_link")
        
        self.broadcaster.sendTransform((self.trans[0], self.trans[1]-0.2, self.trans[2]-0.071),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "button_1",
                                            "base_link")
        
        self.broadcaster.sendTransform((self.trans[0], self.trans[1]-0.2, self.trans[2]-0.071*2),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "button_3",
                                            "base_link")
        
        self.broadcaster.sendTransform((self.trans[0]+0.084, self.trans[1]-0.2, self.trans[2]-0.071*2),
                                            (self.rot[0], self.rot[1], self.rot[2], self.rot[3]),
                                            rospy.Time.now(),
                                            "button_4",
                                            "base_link")
        self.goal_pos = [[self.trans[0]+0.084, self.trans[1]-0.1, self.trans[2]],
                         [self.trans[0], self.trans[1]-0.1, self.trans[2]-0.071],
                         [self.trans[0]+0.084, self.trans[1]-0.1, self.trans[2]-0.071],
                         [self.trans[0], self.trans[1]-0.1, self.trans[2]-0.071*2],
                         [self.trans[0]+0.084, self.trans[1]-0.1, self.trans[2]-0.071*2]
                         ]
        return True
    
    def callback(self, msg):
        try:
            for n in range(0,len(msg.transforms)):
                if(msg.transforms[n].fiducial_id==1):
                    (self.trans, self.rot) = self.listener.lookupTransform('/base_link', '/fiducial_1', rospy.Time(0))
                    # rospy.loginfo("translation:{0} rotation{1}".format(self.trans, self.rot))
                    self.rotate()
                    self.points_created = self.goal_points()   

        except:
            pass
    def main(self):
        while not rospy.is_shutdown():
            if self.points_created and not self.accomp[0]:
                rospy.loginfo("points created")
                self.state1()
            elif self.accomp[0] and not self.accomp[1]:
                self.state2()
            elif self.accomp[1] and not self.accomp[2]:
                self.state3()
            elif self.accomp[2] and not self.accomp[3]:
                self.state4()
            elif self.accomp[3] and not self.accomp[4]:
                self.state5()
            elif self.accomp[4]:
                rospy.loginfo("all states completed")
            else: 
                pass
            rospy.sleep(0.1)
            
if __name__=="__main__":    
    try:
        rospy.init_node("panel_detector")
        marker_det = Marker_Detection()
        marker_det.main()
    except rospy.ROSInterruptException:
        pass