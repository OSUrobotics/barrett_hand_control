import os

obj_number = raw_input("Enter the object number :")
sub_number = raw_input("Enter the subject number :")

directory = "obj"+str(obj_number)+"_sub"+str(sub_number)+"_pointcloud_csvfiles"

if not os.path.exists(directory):
    os.mkdir(directory)
else:
    print "Path Already exists"


