#!/usr/bin/env python

import rospy
import actionlib
import threading

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from action_controller.msg import MultiDofFollowJointTrajectoryAction, MultiDofFollowJointTrajectoryActionFeedback, MultiDofFollowJointTrajectoryActionResult, MultiDofFollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist, Transform
from std_msgs.msg import Empty

class Controller:
    def __init__(self, node):
        self.node_ = node
        self.pub_topic = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.empty = Twist()
        self.lastPosition = Transform()
        self.toExecute = MultiDOFJointTrajectory()
        self.has_active_goal_ = False
        self.active_goal_ = None
        self.creato = 0
        self.action_server_ = actionlib.ActionServer(
            "multi_dof_joint_trajectory_action",
            MultiDofFollowJointTrajectoryAction,
            execute_cb=self.goalCB,
            auto_start=False)
        self.action_server_.register_preempt_callback(self.cancelCB)
        self.action_server_.start()
        rospy.loginfo("Node ready!")

    def cancelCB(self, gh):
        if self.active_goal_ == gh:
            # Stops the controller.
            if self.creato:
                rospy.loginfo("Stop thread")
                self.creato = 0
            self.pub_topic.publish(self.empty)

            # Marks the current goal as canceled.
            self.active_goal_.set_canceled()
            self.has_active_goal_ = False

    def goalCB(self, gh):
        if self.has_active_goal_:
            # Stops the controller.
            if self.creato:
                rospy.loginfo("Stop thread")
                self.creato = 0
            self.pub_topic.publish(self.empty)

            # Marks the current goal as canceled.
            self.active_goal_.set_canceled()
            self.has_active_goal_ = False

        self.active_goal_ = gh
        self.has_active_goal_ = True
        gh.set_accepted()
        self.toExecute = gh.get_goal().trajectory

        # controllore solo per il giunto virtuale Base
        if threading.Thread(target=self.executeTrajectory).start() == None:
            self.creato = 1
            rospy.loginfo("Thread for trajectory execution created")
        else:
            rospy.loginfo("Thread creation failed!")

    def executeTrajectory(self):
        if self.toExecute.joint_names[0] == "Base" and len(self.toExecute.points) > 0:
            for k in range(len(self.toExecute.points)):
                # ricavo cmd da effettuare
                punto = self.toExecute.points[k].transforms[0]
                eseguito = True
                if k != 0:
                    eseguito = self.publishTranslationComand(punto, False)
                    if k == len(self.toExecute.points) - 1:
                        if not eseguito:
                            self.publishTranslationComand(punto, True)
                        self.publishRotationComand(punto, False)
                else:
                    self.publishRotationComand(punto, True)
                self.pub_topic.publish(self.empty)
                # aggiorno start position
                if eseguito:
                    self.lastPosition.translation = punto.translation
                    self.lastPosition.rotation = punto.rotation

        self.active_goal_.set_succeeded()
        self.has_active_goal_ = False
        self.creato = 0

    def publishTranslationComand(self, punto, anyway):
        # creazione comando di traslazione
        cmd = Twist()
        cmd.linear.x = punto.translation.x - self.lastPosition.translation.x
        cmd.linear.y = punto.translation.y - self.lastPosition.translation.y
        cmd.linear.z = punto.translation.z - self.lastPosition.translation.z
        cmd.angular.x = cmd.angular.y = cmd.angular.z = 0

        if anyway or cmd.linear.x >= 0.5 or cmd.linear.y >= 0.5 or cmd.linear.z >= 0.5:
            self.printPositionInfo()
            self.printCmdInfo()
            self.pub_topic.publish(cmd)
            # tempo d'esecuzione
            rospy.sleep(1.0)
            return True
        return False

def publishRotationComand(self, punto, start):
    # comando di allineamento, permesse solo rotazioni sull'asse z
    cmd = Twist()
    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0
    # start = true --> devo tornare nell'orientazione 0
    # start = false --> devo arrivare al'orientazione punto.rotation.z
    cmd.angular.z = (0 - punto.rotation.z) if start else punto.rotation.z

    self.printCmdInfo()

    sleep = cmd.angular.z * 3.0  # tempo necessario a tornare nella giusta orientazione
    if sleep < 0:
        sleep = -sleep
    self.pub_topic.publish(cmd)
    rospy.sleep(sleep)
    cmd.angular.z = 0

def printPositionInfo(self):
    rospy.loginfo("Start Position: [{} {} {}] [{} {} {}]".format(self.lastPosition.translation.x,
                                                                  self.lastPosition.translation.y,
                                                                  self.lastPosition.translation.z,
                                                                  self.lastPosition.rotation.x,
                                                                  self.lastPosition.rotation.y,
                                                                  self.lastPosition.rotation.z))

def printCmdInfo(self):
    cmd = self.empty
    rospy.loginfo("cmd to execute: x: {} y: {} z: {} rX: {} rY: {} rZ: {}".format(cmd.linear.x, cmd.linear.y,
                                                                                   cmd.linear.z, cmd.angular.x,
                                                                                   cmd.angular.y, cmd.angular.z))

def main():
    rospy.init_node("my_controller_node")
    node = rospy.NodeHandle()
    controller = Controller(node)
    rospy.spin()

if name == "main":
    main()