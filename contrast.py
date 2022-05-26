#!/usr/bin/env python

import cv2
import numpy as np

# load image as YUV (or YCbCR) and select Y (intensity)
# or convert to grayscale, which should be the same.
# Alternately, use L (luminance) from LAB.

# Cropped picture from Manual tuning
img = cv2.imread("cropped1.png")

#Cropped picture from ZDK
img2 = cv2.imread("cropped2.png")

#Black and white picture fro reference
img3 = cv2.imread("bw.png")


#Y = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)[:,:,0]

img_grey1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_grey2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
img_grey3 = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)
# compute min and max of Y
#min = np.min(img_grey)
#max = np.max(img_grey)

# compute contrast
#contrast = (max-min)/(max+min)
#print(min,max,contrast)
#filename = 'greyzs.jpg'
#cv2.imwrite(filename, img_grey)


contrast1 = img_grey1.std()
contrast2 = img_grey2.std()
contrast3 = img_grey3.std()
#print(img_grey)
print(contrast1,contrast2, contrast3)