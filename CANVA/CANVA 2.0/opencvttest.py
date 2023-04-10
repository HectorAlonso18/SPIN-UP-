import cv2
import imutils
from PIL import Image

img = Image.open("field.jpg")
img = img.rotate(angle=45)
img.save("field.jpg")
img.show()