from multiprocessing.connection import wait
from operator import truediv
from pydoc import cli
from re import I
import turtle
import cv2
import imutils
from PIL import Image
import os
import math

cancha = turtle.Screen()  # Define la cancha como una pantalla que se va a mostrar
cancha.setup(width=460, height=460)  # Define las medidas de la cancha
cancha.bgpic("field.gif")


#Seleccionar (0,0)
coordenadas = ()
xcero = 0
ycero = 0
x = 0
y = 0
lock = 0
xs = []
ys = []

turtle.addshape('Robot.gif')

def coordenadas(x, y):
    global clicks, xcero, ycero, lock, xs, ys
    if lock == 0:
        coordenadas = (x, y)
        xcero = coordenadas[0]
        ycero = coordenadas[1]
        turtle.pensize(None)
        turtle.goto(xcero, ycero)
        turtle.pensize(5)
        lock = 1
    elif lock == 1:
        m= turtle.Turtle()
        m.penup
        m.goto(x,y)
        m.pendown()
        m.shape('Robot.gif')

        x_respecto = (x - xcero) * 0.313043
        y_respecto = (y - ycero) * 0.313043
        print("x: %.2f" % x_respecto, "y: %.2f" % y_respecto)
        turtle.goto(x, y)
        xs.append(x_respecto)
        ys.append(y_respecto)
        turtle.onscreenclick(print_cords, 2)
        lock = 2
    elif lock == 2:
        m= turtle.Turtle()
        m.penup
        x_respecto = (x - xcero) * 0.313043
        y_respecto = (y - ycero) * 0.313043
        
        print("MIRAR A ","x: %.2f" % x_respecto, "y: %.2f" % y_respecto)
 
        turtle.onscreenclick(print_cords, 2)   
        lock = 1   


def print_cords():
    print("X: ", xs)
    print("Y: ", ys)

def rotate():
    Image.open("field.gif").convert("RGB").save("field.jpg")
    img = Image.open("field.jpg")
    img = img.rotate(angle=90)
    img.save("field.jpg")
    
    Image.open("field.jpg").save("fieldnew.gif")
    cancha.bgpic("fieldnew.gif")
    cancha.update()


def delete():
    os.remove("field.jpg")
    os.remove("fieldrotado.jpg")
    os.remove("fieldnew.gif")




def TO_DEGREES(RADIANES):
    return (RADIANES*180)/3.141516

def reducir_angulo_0_360 (anguloGrados):
    while( not(anguloGrados >=0 and anguloGrados<=360)):
        if(anguloGrados<0):
            anguloGrados+=360

        if(anguloGrados>360):
            anguloGrados-=360    



        return anguloGrados


def get_angle(x1,y1,x2,y2):

    dy= y2-y1
    dx=x2-x1

    ANGLE= TO_DEGREES(math.atan2(dx,dy))

    return ANGLE

turtle.onscreenclick(coordenadas, 1)
turtle.listen()
turtle.onkey(print_cords,"p")
turtle.onkey(rotate,"z")
turtle.onkey(delete,"q")
turtle.mainloop()