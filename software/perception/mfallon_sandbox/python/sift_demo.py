import cv2
import numpy as np

#img = cv2.imread('/home/mfallon/data/atlas/2014-04-21-dense-depth-drills/000000.ppm')
#img = cv2.imread('building.jpg')

img = cv2.imread('utah.jpg')


gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT()
kp = sift.detect(gray,None) # (image, mask)

img=cv2.drawKeypoints(gray,kp)

cv2.imwrite('sift_keypoints.jpg',img)
