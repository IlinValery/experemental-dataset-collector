#!/usr/bin/env python
import roslib
import os
import json
import cv2
from glob import glob
roslib.load_manifest('warevision_dataset_collector')
import rospy
from sensor_msgs.msg import Image
from cv_image_converter import CvImgConverter
from camera_parameters import PiCameraParameters


class PiCameraNode(object):
    def __init__(self, outfile, folder_path):
        print(folder_path)
        rospy.init_node("picamera_node_collector", anonymous=True)
        self.img_subscriber = rospy.Subscriber("pi_image", Image, self.callback, (self, outfile, folder_path))
        self.converter = CvImgConverter()

    def callback(self, msg, args):
        outfile = args[1]
        folder_path = args[2]

        time = msg.header.stamp
        file_path = "{0}/{1}_{2:010d}.png".format(folder_path, time.secs, time.nsecs)
        # image = msg.data
        image = self.converter.convert_to_cv_ndarray(msg)
        cv2.imwrite(file_path, image)

        data = {"time": {"secs":time.secs, "nsecs": time.nsecs}, "path": file_path}
        json.dump(data, outfile)
        outfile.write(",\n")
        

if __name__=="__main__":
    outfile = open("image_data.json", 'w')
    outfile.write("[")
    print("Start writing /imu to the file", os.path.realpath(outfile.name))

    folder_path = "images"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    else:
        files = glob('{}/*'.format(folder_path))
        for f in files:
            os.remove(f)

    picamera_node = PiCameraNode(outfile, folder_path)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()

    outfile.write("{}]")
    outfile.close()
    print("Finished writing to the file", os.path.realpath(outfile.name))

    

