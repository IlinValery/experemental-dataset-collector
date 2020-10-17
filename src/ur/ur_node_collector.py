#!/usr/bin/env python3

import json
import rospy
import os
from geometry_msgs.msg import TransformStamped
from datetime import datetime



def callback(msg: TransformStamped, args):
    outfile = args[0]
    translation = msg.transform.translation
    rotation = msg.transform.rotation
    time = msg.header.stamp
    data = {"time": {"secs":time.secs, "nsecs": time.nsecs},"translation": {"x": translation.x, "y": translation.y, "z": translation.z}, "rotation": {"x": rotation.x, "y": rotation.y, "z": rotation.z}}
    json.dump(data, outfile)
    outfile.write(",\n")


if __name__=="__main__":
    # filename = datetime.now()
    # outfile = open("imus_{}.json".format(filename.timestamp()), 'w')
    outfile = open("ur_data.json", 'w')
    outfile.write("[")
    print("Start writing /ur_pose to the file", os.path.realpath(outfile.name))
    
    rospy.init_node('ur_file_writer')
    rospy.Subscriber("/ur_pose", TransformStamped, callback, (outfile,))
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()
    outfile.write("{}]")
    outfile.close()
    print("Finished writing to the file", os.path.realpath(outfile.name))