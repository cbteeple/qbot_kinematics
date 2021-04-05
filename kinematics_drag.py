import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import time
import copy
import os
import sys
import yaml

from qkin import QBotPlanning, QBotGcode, GCodeSerial

import matplotlib.collections as mcollections
class UpdatablePatchCollection(mcollections.PatchCollection):
    def __init__(self, patches, *args, **kwargs):
        self.patches = patches
        mcollections.PatchCollection.__init__(self, patches, *args, **kwargs)

    def get_paths(self):
        self.set_paths(self.patches)
        return self._paths



# Set up robot dimensions
with open('config/robot_params.yaml', 'r') as f:
    robot_params = yaml.safe_load(f)

robot = QBotPlanning(robot_params)

# Serial port setup
serial_port = '/dev/ttyACM0'
serial_baudrate = 115200

interp = 'linear'
num_reps = 8
num_subdivisions = 20

arm_home = [0,0,0]

feedrate=10000
acceleration = 500

home_filename = 'go_home.gcode'


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
last_pos = arm_start[0]
hull_pts = robot.get_workspace(0)

fig, _ = robot.visualize(arm_home,animate=False)
ax=plt.gca()
fig.set_size_inches(11,6)

point_plot, = plt.plot(arm_start[0,0],arm_start[0,1],'ok',markersize=5.0, markeredgewidth=2.0,zorder=0)
move_plot, = plt.plot([],[],'k',linewidth=2.0, zorder=0)
workspace_plot = Polygon(hull_pts, True)
workspace_collection = UpdatablePatchCollection([workspace_plot], alpha=0.25, color='g',zorder=0)
ax.add_artist(workspace_collection)

ground, = plt.plot(ax.get_xlim() ,[-90,-90],color=[0.439216, 0.133333, 0],linewidth=2.0, zorder=0)


# Set up trajectory converter and sender
gcode_converter = QBotGcode()
gcode_sender = GCodeSerial(serial_port,serial_baudrate)
home_gcode   = os.path.abspath(os.path.join('gcode',home_filename))

in_main_axis=False
curr_slider = -1
def update_sliders(val):
    global last_pos
    global feedrate
    global acceleration
    sliders
    global curr_slider

    if in_main_axis and isinstance(val, dict):
        slider_curr = sliders[val['slider']]
        slider_curr.set_val(val['value'])
        return
    
    elif not in_main_axis:
        if curr_slider ==3:
            feedrate = val
            cmd_list = []
            cmd_list =["G0 F%0.1f\n"%(feedrate)]
            gcode_sender.send_commands(cmd_list)
            return
        if curr_slider ==4:
            acceleration = val
            cmd_list = []
            cmd_list.extend(["M204 T%0.1f P%0.1f\n"%(acceleration,acceleration)])
            cmd_list.extend(["M201 X%0.1f Y%0.1f\n Z%0.1f\n"%(acceleration,acceleration,acceleration)])
            gcode_sender.send_commands(cmd_list)
            return

        waypoint = last_pos
        q_vec, feasible = robot.inverse_kinematics(waypoint)
        q_vec[0,curr_slider] = val    
        new_pose = robot.forward_kinematics(q_vec)
        last_pos = copy.deepcopy(new_pose[0])
        update_robot(new_pose[0])


# Make the sliders
sliders = []
slider_axes = []

for i in range(3):

    axamp = plt.axes([0.84, 0.8-(i*0.05), 0.12, 0.02])
    # Slider
    s = Slider(axamp, 'q{:d}'.format(i), -90, 90, valinit=arm_home[i])
    sliders.append(s)
    slider_axes.append(axamp)


axamp = plt.axes([0.84, 0.8-(len(sliders)*0.05), 0.12, 0.02])
s = Slider(axamp, 'Feedrate', 0, 30000, valinit=feedrate)
sliders.append(s)
slider_axes.append(axamp)

axamp = plt.axes([0.84, 0.8-(len(sliders)*0.05), 0.12, 0.02])
s = Slider(axamp, 'Acceleration', 0, 1000, valinit=acceleration)
sliders.append(s)
slider_axes.append(axamp)


for slider in sliders:
    #samp.on_changed(update_slider)
    slider.on_changed(update_sliders)

curr_slider =3
update_sliders(feedrate)
curr_slider =4
update_sliders(acceleration)
curr_slider =-1



coords = []
clicked = False



ix=arm_start[0,0]
iy=arm_start[0,1]

x_bounds=ax.get_xlim()
y_bounds=ax.get_ylim()



def axes_enter_event(event):
    global in_main_axis
    global curr_slider
    if event.inaxes == ax:
        print('in main axis')
        in_main_axis = True
        color = 'k'
        plt.setp(ax.spines.values(), color=color)
        plt.setp([ax.get_xticklines(), ax.get_yticklines()], color=color)
        fig.canvas.draw()
    else:
        print('out of main axis')
        in_main_axis = False
        color = [0.7,0.7,0.7]
        plt.setp(ax.spines.values(), color=color)
        plt.setp([ax.get_xticklines(), ax.get_yticklines()], color=color)

        point_plot.set_markerfacecolor('none')
        point_plot.set_markeredgecolor('none')
        fig.canvas.draw()

    curr_slider = -1
    for idx,ax_slider in enumerate(slider_axes):
        if event.inaxes == ax_slider:
            curr_slider = idx
    



def press(event):
    global last_pos
    sys.stdout.flush()
    if event.key == 'h':
        print('Going Home)')
        ax.set_facecolor([0.7, 0.7, 0.7])
        fig.canvas.draw()
        home_routine = gcode_converter.read(home_gcode)
        gcode_sender.send_commands(home_routine)
        last_pos = arm_start[0]
        update_robot(arm_start[0])
        time.sleep(2.0)
        ax.set_facecolor('w')
        fig.canvas.draw()


def button_press_callback(event):
    global clicked
    pass

def button_release_callback(event):
    global clicked

    if not in_main_axis:
        return
    

    clicked = not clicked

    if clicked:
        point_plot.set_markersize(10)
        point_plot.set_markerfacecolor('none')
        point_plot.set_markeredgecolor('r')
        color = 'r'
        plt.setp(ax.spines.values(), color=color)
        plt.setp([ax.get_xticklines(), ax.get_yticklines()], color=color)
        fig.canvas.draw()
    else:
        point_plot.set_markersize(5)
        point_plot.set_markerfacecolor('k')
        point_plot.set_markeredgecolor('k')
        color = 'k'
        plt.setp(ax.spines.values(), color=color)
        plt.setp([ax.get_xticklines(), ax.get_yticklines()], color=color)
        fig.canvas.draw()      


def scroll_event(event):
    global last_pos

    if not in_main_axis:
        return

    if event.button == 'up':
        last_pos[2]-=5.0
    elif event.button == 'down':
        last_pos[2]+=5.0

    update_sliders({"slider":2,"value":last_pos[2]})

    update_robot(last_pos)

def motion_notify_callback(event):
    global ix, iy
    global coords
    global last_pos
    global clicked

    ix, iy = event.xdata, event.ydata

    if event.xdata == None or event.ydata ==None:
        return

    if not in_main_axis:
        return
    
    coords = [ix, iy]

    if clicked:
        waypoint = np.array(coords)
        waypoint = np.hstack((waypoint,last_pos[2]))
        #waypoints = np.vstack((last_pos, waypoint))
        #print(waypoints)
        #waypoints = interp_traj(waypoints)
        last_pos = copy.deepcopy(waypoint)

        q_vec, feasible = robot.inverse_kinematics([waypoint])
        update_sliders({"slider":0,"value":q_vec[0,0]})
        update_sliders({"slider":1,"value":q_vec[0,1]})
        update_sliders({"slider":2,"value":q_vec[0,2]})

        update_robot(waypoint)
    #    fig.canvas.mpl_disconnect(cid)

    return coords

def update_robot(waypoint):
    global feedrate
    global acceleration
    q_vec, feasible  = robot.inverse_kinematics([waypoint])

    hull_pts = robot.get_workspace(waypoint[2])
    if len(hull_pts)>0:
        workspace_plot.set_xy(hull_pts)
        workspace_collection.set_visible(True)
    else:
        print('Out of workspace')
        workspace_collection.set_visible(False)

    if np.all(feasible):
        cmd_list =gcode_converter.convert(q_vec)
        gcode_sender.send_commands(cmd_list)
        ax.set_facecolor('w')
        robot.visualize(q_vec[0,:],animate=False, fig=fig)
    else:
        ax.set_facecolor([1,0.7,0.7])
        

    point_plot.set_data([waypoint[0]],[waypoint[1]])
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.01)
    return q_vec


fig.canvas.mpl_connect('button_press_event', button_press_callback)
fig.canvas.mpl_connect('button_release_event', button_release_callback)
fig.canvas.mpl_connect('motion_notify_event', motion_notify_callback)
fig.canvas.mpl_connect('scroll_event', scroll_event)
fig.canvas.mpl_connect('key_press_event', press)
fig.canvas.mpl_connect('axes_enter_event', axes_enter_event)


plt.show()





