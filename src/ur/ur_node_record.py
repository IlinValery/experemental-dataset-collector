#!/usr/bin/env python
import roslib
roslib.load_manifest('warevision_dataset_collector')
import rospy
import os

from robot_bridge import RobotBridge

from geometry_msgs.msg import TransformStamped
import numpy




class URNode(object):
    robot = None
    def __init__(self, robot_ip="192.168.2.15", rate = 2):

        rospy.init_node("ur_info_node", anonymous=True)
        self.pub = rospy.Publisher('ur_pose', TransformStamped, queue_size=10)
        self.rate = rospy.Rate(rate)
        self.robot_ip = robot_ip
        self.init_robot()
        self.frame_id = 0

    def init_robot(self):
        self.robot = RobotBridge(self.robot_ip)
        self.robot.connect()
        print("Connection status:" ,self.robot.is_connected())
        if self.robot.is_connected():
            self.initial_pos =  self.robot.get_tcp()
            print(self.initial_pos)
        else: 
            exit()


    def start_publisher(self):
        while not rospy.is_shutdown():
            tr = TransformStamped()
            tr.header.frame_id = str(self.frame_id)

            pose = self.robot.get_tcp()
            tr.transform.translation.x = pose[0] - self.initial_pos[0]
            tr.transform.translation.y = pose[1] - self.initial_pos[1]
            tr.transform.translation.z = pose[2] - self.initial_pos[2]

            tr.transform.rotation.x = pose[3] - self.initial_pos[3]
            tr.transform.rotation.y = pose[4] - self.initial_pos[4]
            tr.transform.rotation.z = pose[5] - self.initial_pos[5]
            tr.transform.rotation.w = 0.0

            self.pub.publish(tr)
            self.frame_id +=1
            self.rate.sleep()



if __name__ == '__main__':
    try:
        ur_pub = URNode()
        print("Start UR node position")
        ur_pub.start_publisher()
    except rospy.ROSInterruptException:
        print("Done.")
        pass
