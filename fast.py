# Sample script to try out FAST feature detector

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread('blox.jpg') # '<opencv_root>/samples/data/blox.jpg'

cv.imshow("Image", img)
cv.waitKey(0)

# Initiate FAST object with default values
fast = cv.FastFeatureDetector_create()

# Find and draw the keypoints
keypoints = fast.detect(img, None)
img2 = cv.drawKeypoints(img, keypoints, None, color=(255,0,0))

# Print all default params
print( "Threshold: {}".format(fast.getThreshold()) )
print( "nonmaxSuppression:{}".format(fast.getNonmaxSuppression()) )
print( "neighborhood: {}".format(fast.getType()) )
print( "Total Keypoints with nonmaxSuppression: {}".format(len(keypoints)) )

cv.imwrite('fast_true.png', img2)
cv.imshow("Image", img2)
cv.waitKey(0)

# Disable nonmaxSuppression
fast.setNonmaxSuppression(0)
keypoints = fast.detect(img, None)

print( "Total Keypoints without nonmaxSuppression: {}".format(len(keypoints)) )

img3 = cv.drawKeypoints(img, keypoints, None, color=(255,0,0))

cv.imwrite('fast_false.png', img3)
cv.imshow("Image", img3)
cv.waitKey(0)
