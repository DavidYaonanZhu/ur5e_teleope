#!/usr/bin/env python

import rospy
# TF stuff
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from trac_ik_python.trac_ik import IK


class PR2Teleop(object):
    def __init__(self):
        urdf = rospy.get_param('/robot_description')
        self.ik_right = IK("base_link",
                           "ee_link",0.005,1e-5,"Distance",urdf)
        #self.ik_left = IK("torso_lift_link",
         #                 "l_wrist_roll_link")

        #self.left_command = rospy.Publisher('/l_arm_controller/command',
         #                                   JointTrajectory,
          #                                  queue_size=1)

        self.right_command = rospy.Publisher('/arm_controller/command',
                                             JointTrajectory,
                                             queue_size=1)

        #self.last_left_pose = None
        #self.left_pose = rospy.Subscriber('/left_controller_as_posestamped',
                                          #PoseStamped,
                                          #self.left_cb, queue_size=1)
        self.last_right_pose = None
        self.right_pose = rospy.Subscriber('/free_positioning/gripper_marker_pose',
                                           PoseStamped,
                                           self.right_cb, queue_size=1)

        rospy.sleep(2.0)

    def left_cb(self, msg):
        self.last_left_pose = msg

    def right_cb(self, msg):
        self.last_right_pose = msg

    def send_right_arm_goal(self, positions):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        jtp.velocities = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(0.1)   #default 0.4
        jt.points.append(jtp)
        # print("Goal: ")
        print(jt)
        self.right_command.publish(jt)
    '''
    def send_left_arm_goal(self, positions):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                          "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        jtp.velocities = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(0.1)
        jt.points.append(jtp)
        self.left_command.publish(jt)
        '''

    def run_with_ik(self):
        qinit = [0., 0., 0., 0., 0., 0.]
        x = y = z = 0.0
        rx = ry = rz = 0.0
        rw = 1.0
        bx = by = bz = 0.0   #default 0.02
        brx = bry = brz = 0.0  #default 0.5

        r = rospy.Rate(125) #default 4
        while not rospy.is_shutdown():
            ps = self.last_right_pose
            if ps is None:
                r.sleep()
                print("No last right pose...")
                continue
            x = self.last_right_pose.pose.position.x
            y = self.last_right_pose.pose.position.y
            z = self.last_right_pose.pose.position.z

            rx = self.last_right_pose.pose.orientation.x
            ry = self.last_right_pose.pose.orientation.y
            rz = self.last_right_pose.pose.orientation.z
            rw = self.last_right_pose.pose.orientation.w

            # rospy.loginfo("Got pose: " + str(ps))
            sol = None
            retries = 0
            while not sol and retries < 10:
                sol = self.ik_right.get_ik(qinit,
                                           x, y, z,
                                           rx, ry, rz, rw,
                                           bx, by, bz,
                                           brx, bry, brz)
                retries += 1
            if sol:
                print "Solution found: (" + str(retries) + " retries)"
                print sol

                self.send_right_arm_goal(sol)
                qinit = sol
            else:
                print "NO SOLUTION FOUND :("

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ur5e_ik_marker_teleope_py')
    nv = PR2Teleop()
    nv.run_with_ik()

