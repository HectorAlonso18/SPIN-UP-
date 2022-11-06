from multiprocessing.connection import wait
from operator import truediv
from pydoc import cli
from re import I
import turtle

cancha = turtle.Screen()  # Define la cancha como una pantalla que se va a mostrar
cancha.setup(width=460, height=460)  # Define las medidas de la cancha
cancha.bgpic('field.gif')




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
        #print("Ceros", coordenadas)
        xcero = coordenadas[0]
        ycero = coordenadas[1]
        turtle.pensize(0)
        turtle.goto(xcero, ycero)
        turtle.pensize(5)
        lock = 1
        # print(lock)
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
        # turtle.write(str(x_respecto)+","+str(y_respecto))
        turtle.onscreenclick(print_cords, 2)


def print_cords(x, y):
    print("X: ", xs)
    print("Y: ", ys)


turtle.onscreenclick(coordenadas, 1)
turtle.onscreenclick(print_cords, 3)

turtle.mainloop()