import cv2
import numpy as np
import matplotlib.pyplot as plt

# Charger l'image
image = cv2.imread('/home/petrus/duckietown-panel/packages/image.jpg')

image_cropped = image[90:160, 150:190, :]
cv2.imshow('image', image_cropped);
cv2.waitKey(0)
cv2.destroyAllWindows()
