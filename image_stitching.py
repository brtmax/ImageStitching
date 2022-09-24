# 1. Detect keypoints in all of the images
# 2. Match the descriptors between the two images
# 3. Use RANSAC algorithm to estimate a homography matrix using matched descriptors
# 4. Apply warp transformation using the estimated homography matrix
import numpy as np
import cv2
import glob
import imutils