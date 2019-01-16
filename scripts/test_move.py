#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def init_pose():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=[1.57079632, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0], velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    d = 2.0
    g.trajectory.points = []
    for i in range(3):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move():
    t=0.05
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=[1.49999, -0.199993, -1.56999, -1.2678e-06, -3.01992e-07, 2.38419e-06], velocities=[0]*6, time_from_start=rospy.Duration(0.05)),
        JointTrajectoryPoint(positions=[1.49999, -0.2257, -1.47049, -0.0581187, 3.52126e-05, -0.0157037], velocities=[0]*6, time_from_start=rospy.Duration(0.1)),
        JointTrajectoryPoint(positions=[1.49999, -0.251406, -1.37099, -0.116236, 7.07273e-05, -0.0314098], velocities=[0]*6, time_from_start=rospy.Duration(0.15)),
        JointTrajectoryPoint(positions=[1.5, -0.277112, -1.27149, -0.174354, 0.000106242, -0.0471159], velocities=[0]*6, time_from_start=rospy.Duration(0.20)),
        JointTrajectoryPoint(positions=[1.5, -0.302818, -1.17199, -0.232471, 0.000141757, -0.062822], velocities=[0]*6, time_from_start=rospy.Duration(0.25)),
        JointTrajectoryPoint(positions=[1.5, -0.328524, -1.07249, -0.290589, 0.000177271, -0.0785281], velocities=[0]*6, time_from_start=rospy.Duration(0.30)),
        JointTrajectoryPoint(positions=[1.50001, -0.35423, -0.972991, -0.348706, 0.000212786, -0.0942342], velocities=[0]*6, time_from_start=rospy.Duration(0.35)),
        JointTrajectoryPoint(positions=[1.50001, -0.379936, -0.873492, -0.406824, 0.0002483, -0.10994], velocities=[0]*6, time_from_start=rospy.Duration(0.40)),
        JointTrajectoryPoint(positions=[1.50001, -0.405642, -0.773992, -0.464941, 0.000283815, -0.125646], velocities=[0]*6, time_from_start=rospy.Duration(0.45)),
        JointTrajectoryPoint(positions=[1.50001, -0.431349, -0.674493, -0.523058, 0.00031933, -0.141353], velocities=[0]*6, time_from_start=rospy.Duration(0.50))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move2():
    t=0.05
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=[1.50001, -0.431349, -0.674493, -0.523058, 0.00031933, -0.141353], velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        init_pose()
        #move1()
        #move_repeated()
        #move()
        #move_disordered()
        #move_interrupt()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
