import numpy as np
import cv2
from matplotlib import pyplot as plt

#img = cv2.imread('utah.jpg')
img = cv2.imread('/home/mfallon/data/atlas/2014-04-21-dense-depth-drills/000000.ppm')

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)



corners = cv2.goodFeaturesToTrack(gray,800,0.05,5) # img, nfeatures, quality level, quality, distance, 
corners = np.int0(corners)

for i in corners:
    x,y = i.ravel()
    cv2.circle(img,(x,y),3,255,-1)


cv2.imwrite('gftt_keypoints.jpg',img)


img_flip = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

plt.imshow(img_flip),plt.show()
