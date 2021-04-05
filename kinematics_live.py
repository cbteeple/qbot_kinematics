import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import copy
import os

from qkin import QBotPlanning, QBotGcode
from gcode_serial import GCodeSerial

# Set up robot dimensions
robot_params={}
robot_params['link_lengths'] = [250, 250, 50]
robot_params['q_offset'] = [90, 60, 0]

robot = QBotPlanning(robot_params)

# Serial port setup
serial_port = '/dev/ttyACM0'
serial_baudrate = 115200

interp = 'linear'
num_reps = 8
num_subdivisions = 20

arm_home = [0,0,0]

feedrate=20000


def interp_traj(points):

    global num_subdivisions

    if num_subdivisions <2:
        return points

    x= np.empty( shape=(0, 0) )
    y= np.empty( shape=(0, 0) )
    angle= np.empty( shape=(0, 0) )
    for idx,point in enumerate(points):
        if idx ==0:
            continue
        x = np.append(x, np.linspace(points[idx-1,0], points[idx,0], num_subdivisions+1))
        y = np.append(y, np.linspace(points[idx-1,1], points[idx,1], num_subdivisions+1))
        angle = np.append(angle, np.linspace(points[idx-1,2], points[idx,2], num_subdivisions+1))


    waypoints = np.vstack((x,y,angle)).T

    return waypoints





arm_start = robot.forward_kinematics(arm_home)
fig = robot.visualize(arm_home,animate=False)
ax=plt.gca()

point_plot, = plt.plot([],[],'ok',linewidth=2.0)
move_plot, = plt.plot([],[],'k',linewidth=2.0)


# Set up trajectory converter and sender
gcode_converter = QBotGcode()

gcode_sender = GCodeSerial(serial_port,serial_baudrate)


coords = []

last_pos = arm_start

ix=arm_start[0,0]
iy=arm_start[0,1]

def onclick(event):
    global ix, iy
    global coords
    global last_pos

    ix, iy = event.xdata, event.ydata
    
    coords.append([ix, iy])

    if len(coords) == 2:
        coords_new = np.array(coords)
        angle = -np.rad2deg(np.arctan2(coords_new[1,1] - coords_new[0,1],coords_new[1,0]-coords_new[0,0]))
        waypoint = coords_new[0,:]

        print("WAYPOINT")

        waypoint = np.hstack((waypoint,angle))
        waypoints = np.vstack((last_pos, waypoint))

        print(waypoints)

        waypoints = interp_traj(waypoints)
        print(waypoints)

        q_vec = robot.inverse_kinematics(waypoints)
        cmd_list =["G0 F%0.3f\n"%(feedrate)]
        cmd_list.extend(gcode_converter.convert(q_vec))

        move_plot.set_data(coords_new[:,0],coords_new[:,1])
        move_plot.set_data(waypoints[:,0],waypoints[:,1])
        num_points = q_vec.shape[0]
        diff = len(cmd_list)-num_points
        for idx,cmd in enumerate(cmd_list):
            gcode_sender.send_command(cmd)
            if idx >= diff:
                robot.visualize(q_vec[idx-diff,:],animate=False, fig=fig)
                fig.canvas.draw()
                fig.canvas.flush_events()
                #time.sleep(0.01)

        coords=[]
        last_pos = copy.deepcopy(waypoint)
    #    fig.canvas.mpl_disconnect(cid)
    else:
        point_plot.set_data([coords[0][0]],[coords[0][1]])
        fig.canvas.draw()

    return coords
cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()





