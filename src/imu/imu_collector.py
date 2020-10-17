#!/usr/bin/env python3

import json
import rospy
import os
from sensor_msgs.msg import Imu
from datetime import datetime



def callback(msg: Imu, args):
    outfile = args[0]
    av = msg.angular_velocity
    lv = msg.linear_acceleration
    time = msg.header.stamp
    data = {"time": {"secs":time.secs, "nsecs": time.nsecs},"angular_velocity": {"x": av.x, "y": av.y, "z": av.z}, "linear_acceleration": {"x": lv.x, "y": lv.y, "z": lv.z}}
    json.dump(data, outfile)
    outfile.write(",\n")


if __name__=="__main__":
    # filename = datetime.now()
    # outfile = open("imus_{}.json".format(filename.timestamp()), 'w')
    outfile = open("imus_data.json", 'w')
    outfile.write("[")
    print("Start writing /imu to the file", os.path.realpath(outfile.name))
    
    rospy.init_node('imu_file_writer')
    rospy.Subscriber("/imu", Imu, callback, (outfile,))
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()
    outfile.write("{}]")
    outfile.close()
    print("Finished writing to the file", os.path.realpath(outfile.name))