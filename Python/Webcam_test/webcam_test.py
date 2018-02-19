import numpy as np
import matplotlib.pyplot as plt
from cv2 import *
import skimage.io
import skimage.segmentation

cam = VideoCapture(1)  # 0 -> index of camera
s, img = cam.read()
if s:  # frame captured without any errors
    imwrite("filename.jpg", img)  # save image



im = skimage.io.imread('filename.jpg', True)

# Threshold the image first then clear the border
im_clear = skimage.segmentation.clear_border(im < (100.0 / 255.0))

# Show image in figure and hold to place dots in
plt.figure()
plt.imshow(np.dstack([im, im, im]))

# Find coordinates of thresholded image
y, x = np.nonzero(im_clear)

# Find average
xmean = x.mean()
ymean = y.mean()

# Plot on figure
plt.plot(xmean, ymean, 'r.', markersize=14)

# Show image and make sure axis is removed
plt.axis('off')
plt.show()
