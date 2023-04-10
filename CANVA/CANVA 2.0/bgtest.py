import turtle
from PIL import Image
import imutils
import cv2
import os

cancha = turtle.Screen()
cancha.bgpic("field.gif")

def change():
    img90 = cv2.imread("field.jpg")
    imutils.rotate(img90,75)
    cv2.imshow("aaa",img90)
    

    #cancha.bgpic("field.gif")
    #cancha.update()

def delete():
    os.remove("field.jpg")
    os.remove("fieldrotado.jpg")


turtle.listen()
turtle.onkey(change,"z")
turtle.onkey(delete,"q")
turtle.mainloop()