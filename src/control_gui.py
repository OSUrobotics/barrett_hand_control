from Tkinter import *
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def run():
   root = Tk()
   app = Application(master=root)
   #app.mainloop()
   root.destroy()


class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master.pub = rospy.Publisher('control_slider_values', Float32MultiArray, queue_size = 1)
        rospy.init_node('Publish_values', anonymous=True)
        self.grid(row=0,column=0,sticky=N+S+E+W)
        Grid.rowconfigure(master,0,weight=1)
        Grid.columnconfigure(master,0,weight=1)
        self.CreateWidgets()
        self.Publish_values()

    def CreateWidgets(self):
        for rows in xrange(16):
            Grid.rowconfigure(self, rows, weight=1)
        for columns in xrange(1):
            Grid.columnconfigure(self, columns, weight=1)
        self.master.title("barrett hand control")
        self.master.slider_1 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_1.grid(row = 0, column = 0,sticky=N+S+E+W)
        print (self.master.slider_1.getdouble())
        self.master.slider_2 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_2.grid(row = 1, column = 0,sticky=N+S+E+W)
        self.master.slider_3 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_3.grid(row = 2, column = 0,sticky=N+S+E+W)
        self.master.slider_4 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_4.grid(row = 3, column = 0,sticky=N+S+E+W)
        self.master.slider_5 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_5.grid(row = 4, column = 0,sticky=N+S+E+W)
        self.master.slider_6 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_6.grid(row = 5, column = 0,sticky=N+S+E+W)
        self.master.slider_7 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_7.grid(row = 6, column = 0,sticky=N+S+E+W)
        self.master.slider_8 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_8.grid(row = 7, column = 0,sticky=N+S+E+W)
        self.master.slider_9 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_9.grid(row = 8, column = 0,sticky=N+S+E+W)
        self.master.slider_10 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_10.grid(row = 9, column = 0,sticky=N+S+E+W)
        self.master.slider_11 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_11.grid(row = 10, column = 0,sticky=N+S+E+W)
        self.master.slider_12 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_12.grid(row = 11, column = 0,sticky=N+S+E+W)
        self.master.slider_13 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_13.grid(row = 12, column = 0,sticky=N+S+E+W)
        self.master.slider_14 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_14.grid(row = 13, column = 0,sticky=N+S+E+W)
        self.master.slider_15 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_15.grid(row = 14, column = 0,sticky=N+S+E+W)
        self.master.slider_16 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_16.grid(row = 15, column = 0,sticky=N+S+E+W)
        self.master.slider_17 = Scale(self, from_=0, to=1.0, resolution=0.01,orient = HORIZONTAL) 
        self.master.slider_17.grid(row = 16, column = 0,sticky=N+S+E+W)

    def Publish_values(self):
        try:
            while not rospy.is_shutdown():
                vector = [self.master.slider_1.get(),self.master.slider_2.get(),self.master.slider_3.get(),self.master.slider_4.get(),self.master.slider_5.get(),self.master.slider_6.get(),self.master.slider_7.get(),self.master.slider_8.get(),self.master.slider_9.get(),self.master.slider_10.get(),self.master.slider_11.get(),self.master.slider_12.get(),self.master.slider_13.get(),self.master.slider_14.get(),self.master.slider_15.get(),self.master.slider_16.get(),self.master.slider_17.get()]
                self.master.pub.publish(data=vector)
                self.update()

        except KeyboardInterrupt, e:
            print e

if __name__== '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


