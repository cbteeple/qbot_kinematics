import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import product 
import alphashape
import pickle
import os


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx


class QBotPlanning:
    def __init__(self, robot_params):
        self.robot_params = robot_params



    def _split_waypoints(self,waypoints):
        x = []
        y = []
        angle = []
        if isinstance(waypoints,list):  
            for point in waypoints:
                x.append(point[0])
                y.append(point[1])
                angle.append(point[2])

        elif isinstance(waypoints,np.ndarray):
            x = waypoints[:,0]
            y = waypoints[:,1]
            angle = waypoints[:,2]

        return x, y, angle


    def _get_robot_angles(self, q_vec, units='degrees'):
        if q_vec.ndim ==1:
            q_vec = np.expand_dims(q_vec,axis=0)
        
        q1_offset = self.robot_params['q_offset'][0]
        q2_offset = self.robot_params['q_offset'][1]
        q3_offset = self.robot_params['q_offset'][2]

        theta1_bot = q1_offset-q_vec[:,0]
        theta2_bot = theta1_bot-q2_offset-q_vec[:,1]
        theta3_bot = -q_vec[:,2] - q3_offset


        out_vec = np.vstack((theta1_bot,theta2_bot,theta3_bot)).T

        if 'radians' in units:
            out_vec = np.deg2rad(out_vec)

        return out_vec


    def _get_simple_angles(self,q_vec, units='degrees'):

        if q_vec.ndim ==1:
            q_vec = np.expand_dims(q_vec,axis=0)

        q1_offset = self.robot_params['q_offset'][0]
        q2_offset = self.robot_params['q_offset'][1]
        q3_offset = self.robot_params['q_offset'][2]

        theta1_real = q1_offset-q_vec[:,0]
        theta2_real = q_vec[:,0] -q_vec[:,1] -q2_offset


        theta3_real = -q_vec[:,2] - q3_offset

        out_vec = np.vstack((theta1_real,theta2_real,theta3_real)).T

        if 'radians' in units:
            out_vec = np.deg2rad(out_vec)

        return out_vec


    def get_link_positions(self, q_vec):
        """
        Calculate forward kinematics of all links
        """

        # Unpack robot parameters
        L1 = self.robot_params['link_lengths'][0]
        L2 = self.robot_params['link_lengths'][1]
        L3 = self.robot_params['link_lengths'][2]

        q_vec = np.array(q_vec)
        if q_vec.ndim ==1:
            q_vec = np.expand_dims(q_vec,axis=0)

        q_vec = self._get_simple_angles(q_vec, units='radians')

        link1_1=np.vstack((L1*np.cos(q_vec[:,0]), 
                          L1*np.sin(q_vec[:,0])))
        link2_1 = link1_1 \
            + np.vstack((L2*np.cos(q_vec[:,0]+q_vec[:,1]),
                    L2*np.sin(q_vec[:,0]+q_vec[:,1])))

        link3_1 = link2_1 \
            + np.vstack((L3*np.cos(q_vec[:,2]), L3*np.sin(q_vec[:,2])))


        # Return the x,y coordinates of all the links
        return link1_1.T, link2_1.T, link3_1.T


    def forward_kinematics(self, q_vec):
        """
        Calculate forward kinematics of end effector
        """
        q_vec = np.array(q_vec)
        if q_vec.ndim ==1:
            q_vec = np.expand_dims(q_vec,axis=0)

        _,_,link3_1 = self.get_link_positions(q_vec)

        waypoints = np.vstack((link3_1.T, q_vec[:,2]))

        # Return the x,y coordinates of all the links
        return waypoints.T



    def inverse_kinematics(self, waypoints):
        """
        Calculate inverse kinematics from waypoints to joint angles
        """

        # Unpack robot parameters
        L1 = self.robot_params['link_lengths'][0]
        L2 = self.robot_params['link_lengths'][1]
        L3 = self.robot_params['link_lengths'][2]
        q1_offset = self.robot_params['q_offset'][0]
        q2_offset = self.robot_params['q_offset'][1]
        q3_offset = self.robot_params['q_offset'][2]

        waypoints = np.array(waypoints)
        if waypoints.ndim ==1:
            waypoints = np.expand_dims(waypoints,axis=0)

        x_init, y_init, angle = self._split_waypoints(waypoints)

        x = x_init-L3*np.cos(np.deg2rad(-angle))
        y = y_init-L3*np.sin(np.deg2rad(-angle))


        a = 2*L1*x
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
        theta3_deg = np.rad2deg(theta3)

        q_vec = np.vstack((theta1_deg,theta2_deg,theta3_deg)).T

        q_vec_rob = self._get_robot_angles(q_vec)

        feasible = self.check_feasible(q_vec_rob)

        return q_vec_rob, feasible



    def check_feasible(self, q_vec):
        q1_bounds_rel = self.robot_params['joint_limits_rel'][0]
        q2_bounds_rel = self.robot_params['joint_limits_rel'][1]
        q3_bounds_rel = self.robot_params['joint_limits_rel'][2]

        q1_bounds_abs = self.robot_params['joint_limits_abs'][0]
        q2_bounds_abs = self.robot_params['joint_limits_abs'][1]
        q3_bounds_abs = self.robot_params['joint_limits_abs'][2]

        q1=q_vec[:,0]
        q2=q_vec[:,1]
        q3=q_vec[:,2]

        q1_abs_feasible = (q1_bounds_abs[0] <= q1) & (q1 <= q1_bounds_abs[1])
        q2_abs_feasible = (q2_bounds_abs[0] <= q2) & (q2 <= q2_bounds_abs[1])
        q3_abs_feasible = (q3_bounds_abs[0] <= q3) & (q3 <= q3_bounds_abs[1])
        
        q1_rel_feasible = (q1_bounds_rel[0] <= q1)    & (q1 <= q1_bounds_rel[1])
        q2_rel_feasible = (q2_bounds_rel[0] <= q2-q1) & (q2-q1 <= q2_bounds_rel[1])
        q3_rel_feasible = (q3_bounds_rel[0] <= q3-q1) & (q3-q1 <= q3_bounds_rel[1])

        #print('FEASIBLE')
        #print(q_vec)

        all_conditions = np.vstack((q1_abs_feasible, q2_abs_feasible, q3_abs_feasible, q1_rel_feasible, q2_rel_feasible, q3_rel_feasible)).T
        all_feasible = np.all(all_conditions, axis=1)

        return all_feasible


    def calculate_workspace(self, discretize=[50,50,5]):
        print("Calculating Workspace")
        q1_bounds_abs = self.robot_params['joint_limits_abs'][0]
        q2_bounds_abs = self.robot_params['joint_limits_abs'][1]
        q3_bounds_abs = self.robot_params['joint_limits_abs'][2]

        sweep1 = np.linspace(q1_bounds_abs[0],q1_bounds_abs[1],discretize[0]).tolist()
        sweep2 = np.linspace(q2_bounds_abs[0],q2_bounds_abs[1],discretize[1]).tolist()
        sweep3 = np.linspace(q3_bounds_abs[0],q3_bounds_abs[1],discretize[2]).tolist()

        workspace = {}
        workspace['values'] = []
        workspace['q3']     = []

        for q3_val in sweep3:
            print('WORKSPACE for q3=%0.3f'%(q3_val))
            permute = product(sweep1, sweep2, [q3_val])
            q_vec = np.array(list(permute))

            waypoints=self.forward_kinematics(q_vec)           
            xya,feasible = self.inverse_kinematics(waypoints)

            x=waypoints[feasible,0]
            y=waypoints[feasible,1]

            points = waypoints[feasible,0:2]

            alpha = 0.95 * alphashape.optimizealpha(points)
            hull = alphashape.alphashape(points, alpha)
            hull_pts = hull.exterior.coords.xy

            hull_pts = np.array(hull_pts).T

            workspace['values'].append(hull_pts.tolist())
            workspace['q3'].append(q3_val)

        with open('qbot_workspace.pkl', 'wb') as f:
            pickle.dump(workspace,f)

        return workspace


    def get_workspace(self, q3 = None, discretize=[50,50,5]):
        workspace = None

        if os.path.exists('qbot_workspace.pkl'):
            with open('qbot_workspace.pkl', 'rb') as f:
                workspace = pickle.load(f)

        if workspace is not None:
            if q3 is None:
                return workspace
            else:
                if (q3 > max(workspace['q3'])) or (q3 < min(workspace['q3'])):
                    return []
                else:
                    q_idx = find_nearest(workspace['q3'],q3)
                    return workspace['values'][q_idx]
        else:
            return None

        #fig,_ = self.visualize(q_vec, animate=False)
        #plt.plot(x,y,'r.')
        #plt.plot(hull_pts[0],hull_pts[1],'r-')
        #self.visualize(q_vec, fig=fig)
        





    def visualize(self, q_vec, animate=True, fig=None, repeat=True, new=False):
        """
        Visualize the trajectory 
        """

        # Unpack robot parameters
        L1 = self.robot_params['link_lengths'][0]
        L2 = self.robot_params['link_lengths'][1]
        L3 = self.robot_params['link_lengths'][2]

        # Use forward kinematics to get link positions
        link1_pos, link2_pos, link3_pos = self.get_link_positions(q_vec)

        waypoints = self.forward_kinematics(q_vec)

        # Set up the figure
        if fig == None:
            fig_new = plt.figure()
            ax = plt.gca()
            ax.set_xlim(-L1-L2, L1+L2)
            ax.set_ylim(-L1-L2, L1+L2)
            ax.set_aspect('equal', 'box')
        else:
            fig_new = fig
        
        if (fig == None) or (new == True):
            #fulltraj = plt.plot(link2_pos[:,0], link2_pos[:,1], '--', color=[0.9,0.9,0.9])
            fulltraj_ee = plt.plot(link3_pos[:,0], link3_pos[:,1], '--', color=[0.7,0.7,0.7])
            link1, = plt.plot([], [], 'r',linewidth=7, solid_capstyle='round')
            link2, = plt.plot([], [], 'b',linewidth=7, solid_capstyle='round')
            linkee1, = plt.plot([], [], 'k',linewidth=3)
            target, = plt.plot([], [], 'go')

        else:
            link1 = self.link1
            link2 = self.link2
            linkee1 = self.linkee1
            target = self.target


        # Define the init function for animation
        def init():    
            return link1, link2, linkee1, target

        # Define the update function for animation
        def update(frame):
            link1_0 = np.array([0,0])
            link1_1 = link1_pos[frame,:]
            link2_0 = link1_1
            link2_1 = link2_pos[frame,:]

            linkee1_0 = link2_1
            linkee1_1 = link3_pos[frame,:]


            link1.set_data([link1_0[0], link1_1[0]],[link1_0[1], link1_1[1]])
            link2.set_data([link2_0[0], link2_1[0]],[link2_0[1], link2_1[1]])
            linkee1.set_data([linkee1_0[0], linkee1_1[0]],[linkee1_0[1], linkee1_1[1]])
            target.set_data(linkee1_1[0],linkee1_1[1])
            return link1, link2, linkee1, target


        # Show the animation
        if not new:
            self.link1 = link1
            self.link2 = link2
            self.linkee1 = linkee1
            self.target = target


        if animate:
            ani = FuncAnimation(fig_new, update, frames=range(q_vec.shape[0]),
                                init_func=init, blit=False, interval=10, repeat=repeat)

            plt.show()
        else:
            init()
            actors = update(0)
        
        return fig_new, actors

        


class QBotGcode:
    def __init__(self, num_format="%0.3f"):
        self.num_format = "{"+num_format.replace('%', ':')+"}"

    def convert(self,q_vec):
        cmd_list = []
        for idx, line in enumerate(q_vec):
            string = "G0 X"+self.num_format.format(q_vec[idx,0]) + \
                     " Y"+self.num_format.format(q_vec[idx,1])+ \
                     " Z"+self.num_format.format(q_vec[idx,2]) +"\n"
            cmd_list.append(string)
        return cmd_list

    def read(self, filename):
        with open(filename,'r') as f:
            cmd_list = []
            for line in f:
                cmd_list.append(line)

        return cmd_list


    def save(self, filename, cmd_list):
        with open(filename,'w+') as f:
            f.writelines(cmd_list)

