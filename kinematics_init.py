import yaml
from qkin import QBotPlanning

# Get up robot parameters
with open('config/robot_params.yaml', 'r') as f:
    robot_params = yaml.safe_load(f)

# Calculate the workspace
robot = QBotPlanning(robot_params)
robot.calculate_workspace(discretize=[20,20,50])