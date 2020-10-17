#!/usr/bin/env python3

import json
import rospy
import os
from sensor_msgs.msg import Range
from datetime import datetime



def callback(msg: Range, args):
    outfile = args[0]
    range = msg.range 
    time = msg.header.stamp
    data = {"time": {"secs":time.secs, "nsecs": time.nsecs},"range": range}
    json.dump(data, outfile)
    outfile.write(",\n")


if __name__=="__main__":
    # filename = datetime.now()
    # outfile = open("imus_{}.json".format(filename.timestamp()), 'w')
    outfile = open("range_data.json", 'w')
    outfile.write("[")
    print("Start writing /range_data to the file", os.path.realpath(outfile.name))
    
    rospy.init_node('range_node_collector')
    rospy.Subscriber("/range_data", Range, callback, (outfile,))
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()
    outfile.write("{}]")
    outfile.close()
    print("Finished writing to the file", os.path.realpath(outfile.name))