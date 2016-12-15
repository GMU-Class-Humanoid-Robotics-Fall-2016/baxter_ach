#! /usr/bin/env python


"""




"""


import rospy as rp
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
from baxterStructure import *
from baxter_interface import *
import ach
import skimage.morphology as morphology
import time

class baxterImageProcessing(object):

    def __init__(self):

        rp.init_node('baxterImageProcessing')
        # p = ach.Channel('baxterImage')

        # left_camera = CameraController('left_hand_camera')
        # left_camera.resolution = (640, 400)
        # left_camera.open()
        #
        # right_camera = CameraController('right_hand_camera')
        # right_camera.resolution = (640 , 400)
        # right_camera.open



        viewImage = rp.get_param('~view_image' , default=False)
        color = rp.get_param('~color_to_find' , default="Green") # "Green" "Blue" are the other options
        rate = rp.Rate(10)#rp.Rate(rp.get_param('run_rate' , default=10)) # Run Frequency

        self.bridge = cv_bridge.CvBridge()
        self.imageRightFound = False
        self.imageLeftFound = False
        self.subRightImage = rp.Subscriber('/cameras/right_hand_camera/image',Image,self._imageRightCallback)
        self.subLeftImage = rp.Subscriber('/cameras/left_hand_camera/image',Image,self._imageLeftCallback)
        # self.subHeadImage = rp.Subscriber('cameras/head_camera/image',Image,self._imageHeadCallback)
        img = IMAGE()


        # while not rp.is_shutdown():
        while 1:


            print self.imageRightFound , self.imageLeftFound

            if self.imageRightFound == True and self.imageLeftFound == True:# and self.imageHeadFound == True:

                try:
                    rows , cols , channs = self.imageRightData.shape

                    hsvImage = cv2.cvtColor(self.imageRightData,cv2.COLOR_RGB2HSV_FULL)




                    if color == "Red":
                        upperBounds = 90.
                        lowerBounds = 0.
                        print 'red'

                    elif color == "Green":
                        upperBounds = 180.
                        lowerBounds = 91.
                        print 'green'

                    elif color == "Blue":
                        upperBounds = 270.
                        lowerBounds = 181.
                        print 'blue'

                    else:

                        print "Color Error"


                    print "got right color"


                    set1 = np.array(np.where(hsvImage[:,:,0] < upperBounds))
                    set2 = np.array(np.where(hsvImage[:,:,0] > lowerBounds))
                    setInds1 = np.char.array(set1[0,] + '-' + np.char.array(set1[1,]))
                    setInds2 = np.char.array(set2[0,] + '-' + np.char.array(set2[1,]))
                    inds = np.where(np.in1d(setInds1,setInds2))[0]

                    binImg = np.zeros([np.shape(hsvImg)[0] , np.shape(hsvImg)[1]])
                    binImg[set1[0,inds],set2[1,inds]] = 1

                    binImg = morphology.dilation(morphology.erosion(binImg,selem=morphology.disk(1)),selem=morphology.disk(1))
                    cv2.imshow("Right_Hand", binImg)
                    inds = np.where(binImg == 1)

                    try:
                        meanInds = np.mean(inds)
                        img.vertical[RIGHT_ARM] = rows - meanInds[0]
                        img.horizontal[RIGHT_ARM] = cols - meanInds[1]


                    except:
                        img.vertical[RIGHT_ARM] = -1.
                        img.horizontal[RIGHT_ARM] = -1.

                except:
                    print "image acquisition error RIGHT side - baxterImageProcessing.py"

                try:
                    rows, cols, channs = self.imageLeftData.shape

                    hsvImage = cv2.cvtColor(self.imageLeftData, cv2.COLOR_RGB2HSV_FULL)

                    if color == "Red":
                        upperBounds = 90.
                        lowerBounds = 0.
                        print 'red'

                    elif color == "Green":
                        upperBounds = 180.
                        lowerBounds = 91.
                        print 'green'

                    elif color == "Blue":
                        upperBounds = 270.
                        lowerBounds = 181.
                        print 'blue'
                    else:

                        print "Color Error"

                    set1 = np.array(np.where(hsvImage[:, :, 0] < upperBounds))
                    set2 = np.array(np.where(hsvImage[:, :, 0] > lowerBounds))
                    setInds1 = np.char.array(set1[0,] + '-' + np.char.array(set1[1,]))
                    setInds2 = np.char.array(set2[0,] + '-' + np.char.array(set2[1,]))
                    inds = np.where(np.in1d(setInds1, setInds2))[0]

                    binImg = np.zeros([np.shape(hsvImg)[0], np.shape(hsvImg)[1]])
                    binImg[set1[0, inds], set2[1, inds]] = 1

                    binImg = morphology.dilation(morphology.erosion(binImg, selem=morphology.disk(1)),
                                                 selem=morphology.disk(1))
                    cv2.imshow("Left_Hand", binImg)
                    inds = np.where(binImg == 1)

                    try:
                        meanInds = np.mean(inds)
                        img.vertical[LEFT_ARM] = rows - meanInds[0]
                        img.horizontal[LEFT_ARM] = cols - meanInds[1]


                    except:
                        img.vertical[LEFT_ARM] = -1
                        img.horizontal[LEFT_ARM] = -1



                except:
                    print "image acquisition error RIGHT side - baxterImageProcessing.py"
            else:
                img.arm[LEFT_ARM].vertical = -1
                img.arm[LEFT_ARM].horizontal = -1
                img.arm[RIGHT_ARM].vertical = -1
                img.arm[RIGHT_ARM].horizontal = -1

            # p.put(img)

            time.sleep(.1)



    def _imageRightCallback(self,data):
        try:

            self.imageRightData = self.bridge.imgmsg_to_cv(data,"rgb8")
            print "got it"
            #
            # def get_img(msg):
            #     global camera_image
            #     camera_image = msg_to_cv(msg)
            #
            # def msg_to_cv(msg):
            #     return cv_bridge.CvBridge().imgmsg_to_cv(msg, desired_encoding='bgr8')

            self.imageRightFound = True
        except:
            self.imageRightFound = False

    def _imageLeftCallback(self,data):
        try:
            # CvBridge().imgmsg_to_cv(msg, desired_encoding='bgr8')  # True

            self.imageLeftData = self.bridge.imgmsg_to_cv(data,"rgb8")
            self.imageLeftFound = True
        except:
            self.imageLeftFound = False

    def _imageHeadCallback(self,data):
        try:
            self.imageHeadData = self.bridge.imgmsg_to_cv2(data,"rgb8")
            self.imageHeadFound = True
        except:
            self.imageHeadFound = False


if __name__ == "__main__":
    baxterImageProcessing()
