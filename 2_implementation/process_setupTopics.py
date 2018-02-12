#!/usr/bin/env python

import rospy
import sys
import time
import os
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import datetime
import argparse
import rosbag


################################################################################
# CLASS Image_Extractor
################################################################################

class Image_Extractor:

    def __init__(self, setup, imu, nb, rosbag, optimal):
        rospy.loginfo("Saving the Image # %s", str(nb))
        rospy.loginfo(
            "######################################################")
        # Instanciate to the correct size the store
        if setup:
            self.storage = [None] * 3
        else:
            self.storage = [None] * 4

        # Instanciate IMU storage

        self.storageIMU = [None]

        # Instanciate the bridge
        self.bridge = CvBridge()

        # Instanciate the Array of saved Images
        self.imageArray = []

        # Initialize all the triggers
        self.polarcamSaver = False
        self.kinectSaver = False
        self.NIRSaver = False
        self.IDSSaver = False
        self.IMUSaver = False

        # Instanciation of the time storage
        self.str_time = ""

        # Conditionnal Setup -- if true, 3 Camera setup else 4 Camera
        self.en_camSetup = setup

        # Conditionnal IMU -- if true enable IMU
        self.en_IMU = imu

        # Conditionnal rosbag -- if ture enable the creation of rosbag
        self.en_rosbag = rosbag

        try:
            if self.en_camSetup:
                # 3 Cameras
                self.topics = ['pleora_polarcam_driver/image_raw',
                               'kinect2/qhd/image_color_rect', 'camera/image_raw']
            else:
                # 4 Cameras
                if optimal:
                    self.topics = ['pleora_polarcam_driver/image_raw',
                                   'kinect2/qhd/image_mono_rect', 'camera/image_raw', '/ucamera/image_raw']
                else:
                    self.topics = ['pleora_polarcam_driver/image_raw',
                                   'kinect2/qhd/image_color_rect', 'camera/image_raw', '/ucamera/image_raw']

            # IMU Topic
            self.ImuTopic = '/imu_3dm_node/imu/data'

            # Call of the extractor
            self.extractor()
        except rospy.ROSInterruptException as e:
            print(e)

################################################

    # extractor
    # Extract messages from all the topics selected in the topics array
    #	+  array<st>
    #	+  empty array
    #
    #	- None
    def extractor(self):

        temporary = True  # Creation of a trigger frot he process

        while temporary:

            # Subscribers for each of the cameras
            sub_polar = rospy.Subscriber(
                self.topics[0], Image, self.callbackPolarcam)
            sub_kinect = rospy.Subscriber(
                self.topics[1], Image, self.callbackKinect2)
            sub_NIR = rospy.Subscriber(
                self.topics[2], Image, self.callbackNIR)
            if not self.en_camSetup:
                sub_IDS = rospy.Subscriber(
                    self.topics[3], Image, self.callbackIDS)

            sub_IMU = rospy.Subscriber(
                self.ImuTopic, Imu, self.callbackIMU)

            # Condition setup-wise
            if self.en_camSetup:
                cond = self.polarcamSaver and self.kinectSaver and self.NIRSaver
            else:
                cond = self.polarcamSaver and self.kinectSaver and self.NIRSaver and self.IDSSaver

            # Conditionnal, if all the trigger are on
            if cond:
                self.imageArray.append(self.storage)
                if not self.en_rosbag:
                    self.saver()
                    rospy.loginfo(
                        "######################################################")
                    return
                else:
                    rospy.loginfo(
                        "######################################################")
                    return self


################################################

    # saver
    # Save into images any image (cvMat) available in a list
    #	+  None
    #
    #	- None
    def saver(self):

        # Setup 3 or 4 Cameras Checker
        if self.en_camSetup:
            paths = ['Polarcam/', 'Kinect/',
                     'NIR/']  # 3 images
        else:
            paths = ['Polarcam/', 'Kinect/',
                     'NIR/', 'IDS/']  # 4 images

        # path to the saving folders
        Path = '../data/'

        # time stamp formatting for saving
        self.str_time = self.createTimeStamp(
            datetime.datetime.now())

        for p, i in zip(paths, self.imageArray[0]):
            # Create the path
            lastPath = Path + p

            # Save the image
            cv2.imwrite(lastPath.__add__(
                self.str_time + '.tiff'), i)


################################################

    # createTimeStamp
    # Convert time into a formatted string
    #	+  datetime.datetime
    #
    #	- string
    def createTimeStamp(self, time):
        year = str(time.year)
        month = str(time.month)
        day = str(time.day)
        hour = str(time.hour)
        minute = str(time.minute)
        second = str(time.second)
        micro = str(time.microsecond)

        combination = year + '-' + month + '-' + day + '_' + \
            hour + ':' + minute + ':' + second + '.' + micro

        return combination

################################################

    # converter
    # Convert any std_msgs.msg.Image into a cv Mat
    #	+  std_msgs.msg.Image
    #
    #	- cv.Mat
    def converter(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message)
            return cv_image

        except CvBridgeError as e:
            print(e)

################################################

    # callbackPolarcam
    # callback function for the polar camera
    #	+ sensor_msgs.msg.Image
    #	+ array of trigger and storage
    #
    #	- None
    def callbackPolarcam(self, data):
        if not self.polarcamSaver:
            rospy.loginfo("Saving the PolarCam Image")
            if not self.en_rosbag:
                self.storage[0] = self.converter(data)
            else:
                self.storage[0] = data
            self.polarcamSaver = True

################################################

    # callbackKinect2
    # callback function for the Kinect 2
    #	+ sensor_msgs.msg.Image
    #	+ array of trigger and storage
    #
    #	- None
    def callbackKinect2(self, data):

        if not self.kinectSaver:
            rospy.loginfo("Saving the Kinect Image")
            if not self.en_rosbag:
                self.storage[1] = self.converter(data)
            else:
                self.storage[1] = data
            self.kinectSaver = True

################################################

    # callbackNIR
    # callback function for the Near Infra-Red camera
    #	+ sensor_msgs.msg.Image
    #	+ array of trigger and storage
    #
    #	- None
    def callbackNIR(self, data):

        if not self.NIRSaver:
            rospy.loginfo("Saving the NIR Image")
            if not self.en_rosbag:
                self.storage[2] = self.converter(data)
            else:
                self.storage[2] = data
            self.NIRSaver = True

################################################

    # callbackIDS
    # callback function for the IDS Grayscale camera
    #	+ sensor_msgs.msg.Image
    #	+ array of trigger and storage
    #
    #	- None
    def callbackIDS(self, data):

        if not self.IDSSaver:
            rospy.loginfo("Saving the IDS Image")
            if not self.en_rosbag:
                self.storage[3] = self.converter(data)
            else:
                self.storage[3] = data
            self.IDSSaver = True

################################################

    # callbackIMU
    # callback function for the IDS Grayscale camera
    #	+ sensor_msgs.msg.IMU
    #	+ array of trigger and storage
    #
    #	- None
    def callbackIMU(self, data):

        if not self.IMUSaver:
            rospy.loginfo("Saving the IMU Data")
            if self.en_rosbag:
                self.storageIMU[0] = data
            self.IMUSaver = True


######################################################################
# END CLASS Image_Extractor
######################################################################


######################################################################
# FUNCTIONS
######################################################################

# create_rosbag
# creation of the rosbag from list of object containing the data
#	+ list[Object :: Image_Extractor]
#
#	- None
def create_rosbag(ObjectList, en_setup, en_imu):

    # Name of topics
    strContainer = ['Polar', 'Kinect', 'NIR', 'IDS']

    # Dynamic name of bag
    bag = rosbag.Bag('../data/bags/test' +
                     str(len(os.listdir('../data/bags/'))) + '.bag', 'w')

    try:
        # For all object and for all image array in object, push the bag
        for obs in range(0, len(ObjectList)):
            Stamp = rospy.Time.now()
            for im in range(0, len(ObjectList[obs].imageArray[0])):
                bag.write(
                    strContainer[im], ObjectList[obs].imageArray[0][im], Stamp)

            # imu enabled
            if en_imu:
                bag.write(
                    'IMU', ObjectList[obs].storageIMU[0], Stamp)

    finally:
        bag.close()


######################################################################
# END  FUNCTIONS
######################################################################

######################################################################
# PROCESS
######################################################################

if __name__ == "__main__":

    # Parser creation
    parser = argparse.ArgumentParser(
        description='Image Extraction from 4 Cameras')

    # all arguments available in command line
    parser.add_argument(
        "-c", "--camsetup", help="enable the 3 camera setup", action="store_true")

    parser.add_argument(
        "-i", "--imu", help="enable the IMU", action="store_true")

    parser.add_argument(
        "-s", "--snapshot", help="taking a snapshot", action="store_true")

    parser.add_argument("-n", "--number", type=int,
                        help="number of images required")

    parser.add_argument(
        "-f", "--fps", type=int, help="FPS value")

    parser.add_argument(
        "-b", "--rosbag", help="enable the creation of a bag instead of saving the images in folder", action="store_true")

    parser.add_argument(
        "-o", "--calibrate", help="optimal calibration parameters", action="store_true")

    # extract the arguments
    args = parser.parse_args()

    rospy.init_node('Extractor')  # Instanciate the node

    # Create object list in case of rosbag creation
    ObjectList = []

    if not args.snapshot:
        rospy.loginfo(
            "######################################################")
        rospy.loginfo(
            "Saving the Cameras output for %s images", str(args.number))
        rospy.loginfo(
            "Chosen FrameRate : %s", str(args.fps))
        rospy.loginfo(
            "######################################################")

    if args.calibrate:
                    # Sequence grabbing
        for nb in range(0, 100):
            ObjectList.append(Image_Extractor(
                False, True, nb, True, True))
            rospy.sleep(1.0 / float(10))
        # Ros bag Dynamic Creation
        rospy.loginfo(
            "######################################################")
        rospy.loginfo(
            "Creating a rosbag with recorded data")
        rospy.loginfo(
            "######################################################")
        create_rosbag(ObjectList, args.camsetup, args.imu)
    else:
        # if snapshot -- takes only one set of images
        if args.snapshot:
            unused = Image_Extractor(
                args.camsetup, args.imu, 0, args.rosbag, False)

        else:

            if args.rosbag:

                    # Sequence grabbing
                for nb in range(0, args.number):
                    ObjectList.append(Image_Extractor(
                        args.camsetup, args.imu, nb, args.rosbag, False))
                    rospy.sleep(1.0 / float(args.fps))
                # Ros bag Dynamic Creation
                rospy.loginfo(
                    "######################################################")
                rospy.loginfo(
                    "Creating a rosbag with recorded data")
                rospy.loginfo(
                    "######################################################")
                create_rosbag(
                    ObjectList, args.camsetup, args.imu)

            else:
                # Sequence grabbing without bag
                for nb in range(0, args.number):
                    unused = Image_Extractor(
                        args.camsetup, args.imu, nb, args.rosbag, False)
                    rospy.sleep(1.0 / float(args.fps))


######################################################################
# END PROCESS
######################################################################
