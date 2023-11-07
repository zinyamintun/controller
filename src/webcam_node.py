import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera(object):

    def __init__(self):
        self.height = 640
        self.width = 480
        self.fps = 10

        self.bridge = CvBridge()

        self.pubstr = "/shadowsense/camera/raw"
        self.rawpub = rospy.Publisher(self.pubstr, Image, queue_size=10)

        self.cap = cv2.VideoCapture(0)

    def run(self):
        if not self.cap.isOpened():
            print("Camera is not opened\n")

        ret, cv_image = self.cap.read()

        if not ret:
            print("Detect no Image\n")

        else:
            self.rawpub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

if __name__ == '__main__':
    rospy.init_node("Publish_camera")
    camera = Camera()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        camera.run()
        r.sleep()
