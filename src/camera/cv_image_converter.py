from cv_bridge import CvBridge


class CvImgConverter(object):
    def __init__(self):
        self.cv_bridge = CvBridge()

    def convert_to_rosmsg(self, img):
        return self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        
    def convert_to_cv_ndarray(self, rosmsg):
        return self.cv_bridge.imgmsg_to_cv2(rosmsg, "passthrough")

    def convert_img2compressed_rosmsg(self, img):
        return self.cv_bridge.cv2_to_compressed_imgmsg(img, dst_format="png")

    def convert_compressed_rosmsg2cv_img(self, compressed_ros_img):
        return self.cv_bridge.compressed_imgmsg_to_cv2(compressed_ros_img)
