#!/usr/bin/env python

import rospy
import time
# TF stuff
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import *

from trac_ik_python.trac_ik import IK


init_pos1=[1.57079632, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0]
init_pos2=[0.698132, -1.02974, 1.90241, -0.837758, -0.837758, -3.141592]#default
init_pos3=[0.872665,-1.22173,2.00713,-0.785398,-0.698132,-3.14159] 
#init_pos4 is 18cm gripper+4cm wrist radious.  22cm in Z, 0.174465 0.402667 0.22214 
init_pos4=[0.698131,-1.18085,1.82886,-0.613092,-0.837756,-3.14159] 

#control loop rate
control_rate = 125



class PR2Teleop(object):
    def init_pose(self):
        #rospy.init_node("test_move", anonymous=True, disable_signals=True)

        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=init_pos4, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
        client.send_goal(g) #this is the initail pose of the robot
        time.sleep(2.0)
        print "Interrupting"
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise

    def __init__(self):
        urdf = rospy.get_param('/robot_description')
        self.ik_right = IK("base_link",
                           "ee_link",(1.0/control_rate)*0.4,1e-5,"Distance",urdf)
        print(1.0/control_rate)
        #self.ik_left = IK("torso_lift_link",
         #                 "l_wrist_roll_link")

        #self.left_command = rospy.Publisher('/l_arm_controller/command',
         #                                   JointTrajectory,
          #                                  queue_size=1)

        self.right_command = rospy.Publisher('/pos_based_pos_traj_controller/command',
                                             JointTrajectory,
                                             queue_size=1)

        #self.last_left_pose = None /pos_based_pos_traj_controller/command /arm_controller/command
        #self.left_pose = rospy.Subscriber('/left_controller_as_posestamped',
                                          #PoseStamped,
                                          #self.left_cb, queue_size=1)
        self.last_right_pose = init_pos4#None
        self.right_pose = rospy.Subscriber('/sigma7/sigma0/pose',
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
        jtp.time_from_start = rospy.Time(1.0/control_rate)   #default 0.4
        jt.points.append(jtp)
        # print("Goal: ")
        #print(jt)
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
        qinit = init_pos4#[0., 0., 0., 0., 0., 0.]
        x = y = z = 0.0
        rx = ry = rz = 0.0
        rw = 1.0
        bx = by = bz = 0.0   #default 0.02
        brx = bry = brz = 0.0  #default 0.5

        r = rospy.Rate(control_rate) #default 4
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
            start = time.clock()
            while not sol and retries < 10:
                sol = self.ik_right.get_ik(qinit,
                                           x, y, z,
                                           rx, ry, rz, rw,
                                           bx, by, bz,
                                           brx, bry, brz)
                retries += 1
            end = time.clock()
            print "Execution time:" + str(1000*(end-start)) + "ms"
            if sol:
                print "Solution found: (" + str(retries) + " retries)"
                #print sol

                self.send_right_arm_goal(sol)
                qinit = sol
            else:
                print "NO SOLUTION FOUND :("

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ur5e_ik_marker_teleope_py')
    nv = PR2Teleop()
    nv.init_pose()
    time.sleep(7.0) #wait for omega to be ready
    nv.run_with_ik()


