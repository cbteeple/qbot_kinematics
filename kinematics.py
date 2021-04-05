import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os

L1 = 250
L2 = 250
EE_Length = 50


interp = 'linear'
num_reps = 8
num_subdivisions = 0


# Make a circle
theta=np.linspace(0, 2*np.pi,100)
rad=100
x_cart=rad*np.cos(theta)+200
y_cart=rad*np.sin(theta)+200

#x_cart=np.linspace(150,400,60)
#y_cart=100*np.sin(np.pi*(x_cart-150)/125)+100

points=np.vstack((x_cart, y_cart, [40]*y_cart.shape[0])).T

#points_back=np.vstack((np.flipud(x_cart), y_cart, [40]*y_cart.shape[0])).T
#points = np.vstack((points,points_back))

#points=[[200,100,0],
#        [300,100, 90],
#        [300,200, 90],
#        [200,200, -40],
#        [200,100, 0],
#        ]

#points=[[200,200,0],
#        [300,200, 0],
#        [300,300, 0],
#        [200,300, 0],
#        [200,200, 0],
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


print(x)
print(y)
print(angle)


print(os.path.exists( os.path.join('gcode')))


a = 2*L2*x
b = 2*L2*y
c = np.power(L1,2)-np.power(L2,2) - np.power(x,2) - np.power(y,2)


theta2_1 = np.arccos(-c/np.sqrt(np.power(a,2)+np.power(b,2)))
theta2_2 = np.arctan2(b,a)
theta2_opt1 =  theta2_1+theta2_2
theta2_opt2 =  -theta2_1+theta2_2

theta12=np.min([theta2_opt1,theta2_opt2], axis=0)


theta1 = np.arctan2(y-L2*np.sin(theta12), x-L2*np.cos(theta12))
theta2 = theta12 - theta1
theta3 = -np.deg2rad(angle)

theta1_deg = np.rad2deg(theta1)
theta2_deg = np.rad2deg(theta2)
theta3_deg = -angle

theta1_bot = 90-theta1_deg
theta2_bot = theta1_bot-60-theta2_deg #theta1_deg-15-theta2_deg
theta3_bot = -theta3_deg

print(theta1_bot)
print(theta2_bot)
print(theta3_bot)

fig = plt.figure()
fulltraj = plt.plot(x, y, '--', color=[0.7,0.7,0.7])
link1, = plt.plot([], [], 'r',linewidth=5)
link2, = plt.plot([], [], 'b',linewidth=5)
linkee1, = plt.plot([], [], 'k',linewidth=3)
linkee2, = plt.plot([], [], 'k',linewidth=3)
target, = plt.plot([], [], 'go')

ax = plt.gca()
ax.set_xlim(-L1-L2, L1+L2)
ax.set_ylim(-L1-L2, L1+L2)
ax.set_aspect('equal', 'box')

def init():    
    return link1, link2, linkee1, linkee2, target

def update(frame):
    theta1_curr = theta1[frame]
    theta2_curr = theta2[frame]
    theta3_curr = theta3[frame]

    link1_0 = np.array([0,0])
    link1_1=np.array([L1*np.cos(theta1_curr), L1*np.sin(theta1_curr)])
    link2_0 = link1_1
    link2_1 = link1_1 \
        + np.array([L2*np.cos(theta1_curr+theta2_curr), L2*np.sin(theta1_curr+theta2_curr)])

    linkee1_0 = link2_1
    linkee1_1 = link2_1 \
        + np.array([EE_Length*np.cos(theta3_curr), EE_Length*np.sin(theta3_curr)])

    linkee2_0 = link2_1
    linkee2_1 = link2_1 \
        + np.array([EE_Length*np.cos(theta3_curr+np.pi/2), EE_Length*np.sin(theta3_curr+np.pi/2)])

    link1.set_data([link1_0[0], link1_1[0]],[link1_0[1], link1_1[1]])
    link2.set_data([link2_0[0], link2_1[0]],[link2_0[1], link2_1[1]])
    linkee1.set_data([linkee1_0[0], linkee1_1[0]],[linkee1_0[1], linkee1_1[1]])
    linkee2.set_data([linkee2_0[0], linkee2_1[0]],[linkee2_0[1], linkee2_1[1]])
    target.set_data(x[frame],y[frame])
    return link1, link2, linkee1, linkee2, target


ani = FuncAnimation(fig, update, frames=range(len(theta1)),
                    init_func=init, blit=False, interval=10)
plt.show()


filename = os.path.abspath(os.path.join('gcode',out_filename))
prefix   = os.path.abspath(os.path.join('gcode',prefix_filename))
suffix   = os.path.abspath(os.path.join('gcode',suffix_filename))

print(filename)



with open(filename,'w+') as out:
    if os.path.exists(prefix):
        with open(prefix,'r') as pre:
            for line in pre:
                out.write(line)
        out.write('\n')
        print(prefix)
    else:
        print("Skipping Prefix")

    for i in range(num_reps):
        out.write("G0 F%0.3f\n"%(feedrate))
        for idx, line in enumerate(theta1_bot):
            string = "G0 X%0.3f Y%0.3f Z%0.3f\n"%(theta1_bot[idx], theta2_bot[idx], theta3_bot[idx])
            out.write(string)

    if os.path.exists(suffix):
        with open(suffix,'r') as suf:
            for line in suf:
                out.write(line)
        print(suffix)
    else:
        print("Skipping Suffix")
