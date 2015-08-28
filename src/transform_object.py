#!/usr/bin/env python
from Tkinter import *
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

def run():
   rospy.init_node('Publish_values', anonymous=True)
   root = Tk()
   app = Application(master=root)
   root.destroy()


class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master.pub = rospy.Publisher('slider_for_transformation', Float32MultiArray, queue_size = 1)
        self.grid(row=0,column=0,sticky=N+S+E+W)
        Grid.rowconfigure(master,0,weight=1)
        Grid.columnconfigure(master,0,weight=1)
        self.CreateWidgets()
        self.flag = 0
        self.Publish_values()

    def CreateWidgets(self):
        for rows in xrange(7):
            Grid.rowconfigure(self, rows, weight=1)
        for columns in xrange(1):
            Grid.columnconfigure(self, columns, weight=1)
        self.master.title("barrett hand control")
        self.master.slider_1 = Scale(self, from_=(-1*math.pi), to=math.pi, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_1.grid(row = 0, column = 0,sticky=N+S+E+W)
        self.master.slider_2 = Scale(self, from_=(-1* math.pi), to= (math.pi), resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_2.grid(row = 1, column = 0,sticky=N+S+E+W)
        self.master.slider_3 = Scale(self, from_=(-1*math.pi), to=math.pi, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_3.grid(row = 2, column = 0,sticky=N+S+E+W)
        self.master.slider_4 = Scale(self, from_=-2.5, to=2.5, resolution=0.0001,orient = HORIZONTAL) 
        self.master.slider_4.grid(row = 3, column = 0,sticky=N+S+E+W)
        self.master.slider_5 = Scale(self, from_=-2.5, to=2.5, resolution=0.0001,orient = HORIZONTAL) 
        self.master.slider_5.grid(row = 4, column = 0,sticky=N+S+E+W)
        self.master.slider_6 = Scale(self, from_=-2.5, to=2.5, resolution=0.0001,orient = HORIZONTAL) 
        self.master.slider_6.grid(row = 5, column = 0,sticky=N+S+E+W)
        self.master.button = Button(self, text = "Reset everything", command = self.reset_zero)
        self.master.button.grid(row = 7, column = 0, sticky = N+S+E+W)
        self.master.slider_4.set(-0.71670001745224)
        self.master.slider_5.set(-0.11140000075101852)
        self.master.slider_6.set(0.3)

    def reset_zero(self):
        self.master.slider_1.set(0.00)
        self.master.slider_2.set(0.00)
        self.master.slider_3.set(0.00)
        self.master.slider_4.set(-0.71670001745224)
        self.master.slider_5.set(-0.11140000075101852)
        self.master.slider_6.set(0.3)

    def Publish_values(self):
        try:
            while not rospy.is_shutdown():
                Transformation_matrix = [[math.cos(self.master.slider_1.get())*math.cos(self.master.slider_2.get()), math.cos(self.master.slider_1.get())*math.sin(self.master.slider_2.get())*math.sin(self.master.slider_3.get())- math.sin(self.master.slider_1.get())*math.cos(self.master.slider_3.get()), math.cos(self.master.slider_1.get())*math.sin(self.master.slider_2.get())*math.cos(self.master.slider_3.get()) + math.sin(self.master.slider_1.get())*math.sin(self.master.slider_3.get()), self.master.slider_4.get()],[math.sin(self.master.slider_1.get())*math.cos(self.master.slider_2.get()), math.sin(self.master.slider_1.get())*math.sin(self.master.slider_2.get())*math.sin(self.master.slider_3.get()) + math.cos(self.master.slider_1.get())* math.cos(self.master.slider_3.get()), math.sin(self.master.slider_1.get())*math.sin(self.master.slider_2.get())*math.cos(self.master.slider_3.get())-math.cos(self.master.slider_1.get())*math.sin(self.master.slider_3.get()), self.master.slider_5.get()],[-1*math.sin(self.master.slider_2.get()), math.cos(self.master.slider_2.get())*math.sin(self.master.slider_3.get()), math.cos(self.master.slider_2.get())*math.cos(self.master.slider_3.get()), self.master.slider_6.get()]]

                vector = [Transformation_matrix[0][0],Transformation_matrix[0][1],Transformation_matrix[0][2],Transformation_matrix[0][3],Transformation_matrix[1][0],Transformation_matrix[1][1],Transformation_matrix[1][2],Transformation_matrix[1][3],Transformation_matrix[2][0],Transformation_matrix[2][1],Transformation_matrix[2][2],Transformation_matrix[2][3],0,0,0,1]
                self.master.pub.publish(data=vector)
                self.update()

        except KeyboardInterrupt, e:
            print e

if __name__== '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


