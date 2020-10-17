#!/usr/bin/env python
import roslib
roslib.load_manifest('warevision_dataset_collector')
import rospy
from sensor_msgs.msg import Image
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv_image_converter import CvImgConverter
from camera_parameters import PiCameraParameters


class PiCameraNode(object):
    def __init__(self):
        rospy.init_node("picamera_node", anonymous=True)
        self.configs = PiCameraParameters()
        self.camera = PiCamera()
        self.converter = CvImgConverter()
        self.update_camera_parameters()
        self.capturer = PiRGBArray(self.camera, size=(self.configs.width, self.configs.height))
        self.img_publisher = rospy.Publisher("pi_image", Image, queue_size=1)
    
    def update_camera_parameters(self):
        self.camera.rotation = self.configs.rotation
        self.camera.resolution = (self.configs.width, self.configs.height)
        self.camera.framerate = self.configs.framerate

    def start_publisher(self):
        try:
            for frame in self.camera.capture_continuous(self.capturer, format="bgr", use_video_port=True):
                image = frame.array
                rosmsg = self.converter.convert_to_rosmsg(image)
                rosmsg.header.stamp = rospy.Time.now()
                rosmsg.header.frame_id = '0'
                self.img_publisher.publish(rosmsg)
                self.capturer.truncate(0)
        except KeyboardInterrupt:
            exit

if __name__=="__main__":
    picamera_node = PiCameraNode()
    picamera_node.start_publisher()
