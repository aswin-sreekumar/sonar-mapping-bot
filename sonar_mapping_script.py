# SONAR MAPPING Bot
# Python script to receive data from bot and plot data on graph

# Edit csv file name before usage

# Control using
# 0 - STOP
# 1 - FORWARD
# 2 - LEFT
# 3 - RIGHT
# 4 - BACK
# 'b' - Scan

# required libraries
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import math
import sys
import csv
import numpy as np

# graph parameters for display
plt.title("SONAR SLAM")
plt.xlabel("x")
plt.ylabel("y")

# csv filename
filename="plot_data.csv"

# required global variables
reference_data=[0,0] # starting origin data - x,y
theta=0     # raw yaw angle received
angle=0     # raw servo angle received
distance=0  # IR feedback data from bot
distance_data=[0,0] # store distance values received
x_new=0 # current x coordinate
y_new=0 # current y coordinate
x_list=[0] # list of x coordinates visited
y_list=[0] # list of y coordinates visited
map_x_list=[0]  # list of x-coordinates of obstacles
map_y_list=[0]  # list of y-coordinat aes of obstacles
theta_list=[0]  # computed bot yaw angle

# for plotting data using matplotlib
style.use('fivethirtyeight')
fig=plt.figure()
ax1=fig.add_subplot(1,1,1)


# realtime updation in graph using animation
def animate(i):
    ax1.clear()
    ax1.plot(map_x_list,map_y_list,'b.')
    ax1.plot(x_list,y_list,'r-')

# computing trajectory followed by bot using navigation data received
def compute_position():
    rec_data=bt_path.readline()             # receive data
    rec_data=rec_data.decode('utf-8')
    rec_data=rec_data[:-2]
    rec_data_list=rec_data.split(':')
    theta=-float(rec_data_list[1])          # yaw of bot
    beta=theta
    if beta<-180:
        beta=beta+180
    distance=float(rec_data_list[0])+5.0
    print("REC RAW POSITION DATA : ",distance," : ",theta)  # raw data transmitted from bot
    theta=math.radians(theta)
    beta=math.radians(beta)
    print("POSITION DATA in RAD : ",theta)
    theta_list.append(beta)
    distance_data.append(distance)
    distance=distance-distance_data[-2]
    x_new=x_list[-1]+(distance*math.cos(theta)) # computation of x-coordinate of bot
    y_new=y_list[-1]+(distance*math.sin(theta)) # computation of y-coordinate of bot
    print("COMPUTED POSITION DATA : ",x_new," : ",y_new)    # bot position computed from raw data
    x_list.append(x_new)            # x-coordinate of bot updated
    y_list.append(y_new)            # y-coordinate of bot updated

# plotting obstacle data around bot
def compute_map():
    while True:
        rec_data=bt_path.readline()
        rec_data=rec_data.decode('utf-8')
        rec_data=rec_data[:-2]
        if(rec_data=='**'):
            break
        rec_data_list=rec_data.split(':')
        angle=float(rec_data_list[1])
        distance=float(rec_data_list[0])
        if(distance>20):            # filter/threshold over distance of obstacles
            continue
        print("RAW MAP DATA : ",distance," : ",angle)
        angle=angle-90
        angle=math.radians(angle)
        x_coordinate=distance*(math.cos(angle))
        y_coordinate=distance*(math.sin(angle))
        theta=theta_list[-1]
        print("THETA FOR MAPPING : ",theta)
        Rz=[[math.cos(theta),-math.sin(theta),x_list[-1]],[math.sin(theta),math.cos(theta),y_list[-1]],[0,0,1]] #rotational transform matrix
        Cz=[[x_coordinate],[y_coordinate],[1]]  # raw coordinates matrix of obstacle point
        Tz=[[1],[1],[1]]
        Tz=np.dot(Rz,Cz)        # homogenous transform computed to get final obstacle coordinates
        x_coordinate=Tz[0][0]   # computed obstacle x-coordinate
        y_coordinate=Tz[1][0]   # computed obstacle y-coordinate
        print("COMPUTED MAP DATA : ",x_coordinate," : ",y_coordinate)
        map_x_list.append(x_list[-1])
        map_y_list.append(y_list[-1])
        map_x_list.append(x_coordinate)
        map_y_list.append(y_coordinate)


# write the data into csv file for future computations
def file_write():
    with open(filename,'w') as csvfile:
        i=0
        while (i<(len(x_list))):
            data_list=[x_list[i],y_list[i]]
            csvwriter=csv.writer(csvfile)
            csvwriter.writerow(data_list)
            i=i+1

# initialise and open bluetooth serial port
bt_path=serial.Serial()
bt_path.baudrate=9600
bt_path.port='COM7'
bt_path.open()
time.sleep(1)
if(bt_path.isOpen()):
    print("Bluetooth connected successfully")




ch=input("INITIALISE PROGRAM ?")
bt_path.write(bytes('a','utf-8'))
if(bt_path.readline()):
    print("Bot ready to rage")
    bt_path.flush()
rec_data=bt_path.readline() # received string from bot
rec_data=rec_data.decode('utf-8')
rec_data=rec_data[:-2]
rec_data_list=rec_data.split(':') # list with splitted data
reference_data[0]=float(rec_data_list[0]) # x-coordinate of origin
reference_data[1]=float(rec_data_list[1]) # y-coordinate of origin
x_list.append(reference_data[0])
y_list.append(reference_data[1])

print("x-coordinate: ",reference_data[0],"\t y-coordinate: ",reference_data[1])

ani=animation.FuncAnimation(fig,animate,interval=1000)
fig.show()

# Infinite loop
# Control using 1,2,3,4,0 - F,L,R,B
# Scan using 'b'
while True:
    ch=input(">>")
    if(ch=='a'):
        break
    if(ch=='\n'):
        continue
    print("SENT :",ch[0])
    bt_path.write(bytes(ch[0],'utf-8'))
    time.sleep(0.1)
    if(ch=='b'):
        compute_map()
    else:
        compute_position()

# terminate the program properly
sys.stdout.flush()
bt_path.close()
file_write()
