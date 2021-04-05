import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os

from qkin import QBotPlanning, QBotGcode
from gcode_serial import GCodeSerial

# Set up robot dimensions
robot_params={}
robot_params['link_lengths'] = [250, 250, 50]
robot_params['q_offset'] = [90, 65, 0]

robot = QBotPlanning(robot_params)

# Serial port setup
serial_port = '/dev/ttyACM0'
serial_baudrate = 115200

save_trajectory = True
run_trajectory = True

interp = 'linear'
num_reps = 8
num_subdivisions = 0


# Make a circle
theta=np.linspace(3*np.pi/4,5*np.pi/4,100)
rad=50

x_cart=rad*np.cos(theta)+200
y_cart=rad*np.sin(theta)+200


x_cart=np.array([200]*100)
y_cart=np.array([200]*100)

#x_cart=np.linspace(150,400,60)
#y_cart=100*np.sin(np.pi*(x_cart-150)/125)+100

#points=np.vstack((x_cart, y_cart, [40]*y_cart.shape[0])).T
points=np.vstack((x_cart, y_cart, -np.rad2deg(theta)+180)).T

print(points)


points_back=np.flipud(points)

print(points_back)
points = np.vstack((points,points_back))

#points=[[200,100,0],
#        [300,100, 90],
#        [300,200, 90],
#        [200,200, -40],
#        [200,100, 0],
#        ]

#points=[[200,200,50],
#        [300,200, 50],
#        [300,300, 50],
#        [200,300, 50],
#        [200,200, 50],
#        ]
#points_back=np.flipud(points)
#points = np.vstack((points,points_back))



feedrate=20000
out_filename = 'circles.gcode'
prefix_filename = 'NONE' #'go_home.gcode'
suffix_filename = 'manual_home.gcode'



points = np.asarray(points, dtype=np.float64)

x= np.empty( shape=(0, 0) )
y= np.empty( shape=(0, 0) )
angle= np.empty( shape=(0, 0) )
if interp:
    for idx,point in enumerate(points):
        if idx ==0:
            continue
        x = np.append(x, np.linspace(points[idx-1,0], points[idx,0], num_subdivisions+1))
        y = np.append(y, np.linspace(points[idx-1,1], points[idx,1], num_subdivisions+1))
        angle = np.append(angle, np.linspace(points[idx-1,2], points[idx,2], num_subdivisions+1))


waypoints = np.vstack((x,y,angle)).T

q_vec = robot.inverse_kinematics(waypoints)
robot.visualize(q_vec, animate=True)




print(os.path.exists( os.path.join('gcode')))

filename = os.path.abspath(os.path.join('gcode',out_filename))
prefix   = os.path.abspath(os.path.join('gcode',prefix_filename))
suffix   = os.path.abspath(os.path.join('gcode',suffix_filename))

print(filename)

gcode_converter = QBotGcode()
cmd_list = []

if os.path.exists(prefix):
    cmd_list.extend(gcode_converter.read(prefix))
    print(prefix)
else:
    print("Skipping Prefix")

for i in range(num_reps):
    cmd_list.append("G0 F%0.3f\n"%(feedrate))
    cmd_list.extend(gcode_converter.convert(q_vec))

if os.path.exists(suffix):
    cmd_list.extend(gcode_converter.read(suffix))
else:
    print("Skipping Suffix")

if save_trajectory:
    gcode_converter.save(filename,cmd_list)


if run_trajectory:
    gcode_sender = GCodeSerial(serial_port,serial_baudrate)
    gcode_sender.send_commands(cmd_list)
