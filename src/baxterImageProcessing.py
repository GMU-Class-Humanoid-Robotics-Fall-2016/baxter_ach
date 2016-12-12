#! /usr/bin/env python


"""




"""


import rospy as rp
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
from baxterStructure import *
import ach
import skimage.morphology as morphology

class baxterImageProcessing(object):

    def __init__(self):

        rp.init_node('baxterImageProcessing')
        p = ach.Channel('baxterImage')

        viewImage = rp.get_param('~view_image' , default=False)
        color = rp.get_param('~color_to_find' , default="Red") # "Green" "Blue" are the other options
        rate = rp.Rate(rp.get_param('run_rate' , default=10)) # Run Frequency

        self.bridge = cv_bridge.CvBridge
        self.imageRightFound = False
        self.imageLeftFound = False
        self.subRightImage = rp.Subscriber('/cameras/right_hand_camera/image',Image,self._imageRightCallback)
        self.subLeftImage = rp.Subscriber('/cameras/left_hand_camera/image',Image,self._imageLeftCallback)
        self.subHeadImage = rp.Subscriber('cameras/head_camera/image',Image,self._imageHeadCallback)
        img = IMAGE()
        while not rp.is_shutdown():

            if self.imageRightFound == True and self.imageLeftFound == True:# and self.imageHeadFound == True:

                try:
                    rows , cols , channs = self.imageRightData.shape

                    hsvImage = cv2.cvtColor(self.imageRightData,cv2.COLOR_RGB2HSV_FULL)

                    if color == "Red":
                        upperBounds = 0.
                        lowerBounds = 0.
                        print 'red'

                    elif color == "Green":
                        upperBounds = 0.
                        lowerBounds = 0.
                        print 'green'

                    elif color == "Blue":
                        upperBounds = 0.
                        lowerBounds = 0.
                        print 'blue'

                    else:

                        print "Color Error"

                    set1 = np.array(np.where(hsvImage[:,:,0] < upperBounds))
                    set2 = np.array(np.where(hsvImage[:,:,0] > lowerBounds))
                    setInds1 = np.char.array(set1[0,] + '-' + np.char.array(set1[1,]))
                    setInds2 = np.char.array(set2[0,] + '-' + np.char.array(set2[1,]))
                    inds = np.where(np.in1d(setInds1,setInds2))[0]

                    binImg = np.zeros([np.shape(hsvImg)[0] , np.shape(hsvImg)[1]])
                    binImg[set1[0,inds],set2[1,inds]] = 1

                    binImg = morphology.dilation(morphology.erosion(binImg,selem=morphology.disk(1)),selem=morphology.disk(1))

                    inds = np.where(binImg == 1)

                    try:
                        meanInds = np.mean(inds)
                        img[RIGHT_ARM].vertical = rows - meanInds[0]
                        img[RIGHT_ARM].horizontal = cols - meanInds[1]


                    except:
                        img[RIGHT_ARM].vertical = []
                        img[RIGHT_ARM].horizontal = []

                except:
                    print "image acquisition error RIGHT side - baxterImageProcessing.py"

                try:
                    rows, cols, channs = self.imageLeftData.shape

                    hsvImage = cv2.cvtColor(self.imageLeftData, cv2.COLOR_RGB2HSV_FULL)

                    if color == "Red":
                        upperBounds = 0.
                        lowerBounds = 0.
                        print 'red'

                    elif color == "Green":
                        upperBounds = 0.
                        lowerBounds = 0.
                        print 'green'

                    elif color == "Blue":
                        upperBounds = 0.
                        lowerBounds = 0.
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

                    inds = np.where(binImg == 1)

                    try:
                        meanInds = np.mean(inds)
                        img[LEFT_ARM].vertical = rows - meanInds[0]
                        img[LEFT_ARM].horizontal = cols - meanInds[1]


                    except:
                        img[LEFT_ARM].vertical = []
                        img[LEFT_ARM].horizontal = []



                except:
                    print "image acquisition error RIGHT side - baxterImageProcessing.py"
            else:
                img[LEFT_ARM].vertical = []
                img[LEFT_ARM].horizontal = []
                img[RIGHT_ARM].vertical = []
                img[RIGHT_ARM].horizontal = []

            p.put(img)

            rate.sleep()



    def _imageRightCallback(self,data):
        try:
            self.imageRightData = self.bridge.imgmsg_to_cv2(data,"rgb8")
            self.imageRightFound = True
        except:
            self.imageRightFound = False

    def _imageLeftCallback(self,data):
        try:
            self.imageLeftData = self.bridge.imgmsg_to_cv2(data,"rgb8")
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
