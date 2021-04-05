import yaml

from qkin import QBotPlanning

# Set up robot dimensions
robot_params={}
robot_params['link_lengths'] = [250, 250, 50]
robot_params['q_offset'] = [90, 60, 0]
robot_params['joint_limits_abs'] = [[-90, 90], [0,180],  [-45,135]]
robot_params['joint_limits_rel'] = [[-90, 90], [-30,80], [-45,70] ]

with open('robot_params.yaml', 'w') as f:
    yaml.dump(robot_params, f,default_flow_style=None)

robot = QBotPlanning(robot_params)
robot.calculate_workspace(discretize=[20,20,50])