import numpy as np
import colorsys
import json
from pycrazyswarm import *
import yaml

# Environment constants
Z = 1.0
TAKEOFF_DURATION = 2.5
TARGET_HEIGHT = 0.02
GOTO_DURATION = 3.0
LAND_DURATION = 3.0

# Read the JSON file
with open('../data/utra_sim.json') as json_file:
    data = json.load(json_file)

num_timesteps = len(data['movements'])
print(num_timesteps)
num_robots = len(data['movements']['0'])  # Assuming all timesteps have the same number of robots
print(num_robots)

time_info = np.zeros(num_timesteps)  # holds time values
robot_position_info = np.zeros((num_timesteps, num_robots, 3))  # holds robot position values

for timestep_index, timestep in data['movements'].items():
    timestep_index = int(timestep_index)
    time_info[timestep_index] = float(timestep_index)
    for robot_index, robot in enumerate(timestep):
        robot_id = robot['robot']
        position = robot['position']
        robot_position_info[timestep_index, robot_index, :] = position['x'], position['y'], position['z']

# Synchronizes yaml file with crazyflies in simulation
with open('../launch/allCrazyflies_demo.yaml', 'r') as f:
    allcfs = yaml.load(f)
    my_cfs = allcfs['crazyflies']
    indices_dict = {cf['id']: i for i, cf in enumerate(my_cfs)}

# Reads in crazyflies yaml such that the indices of simulation setup match with crazyflie ids
with open('../launch/crazyflies.yaml') as f:
    mycfs = yaml.load(f)
    robot_ids = [cf['id'] for cf in mycfs['crazyflies']]

robot_indices = [int(indices_dict[r]) for r in robot_ids]
robot_position_info = np.array(robot_position_info)
num_robots = min(len(robot_indices), robot_position_info.shape[1])
print(robot_position_info)
ALL_WAYPOINTS = robot_position_info[:, robot_indices[:num_robots], :]

def generateRGBColors(num_colors):
    output = []
    num_colors += 1  # to avoid the first color
    for index in range(1, num_colors):
        incremented_value = 1.0 * index / num_colors
        output.append(colorsys.hsv_to_rgb(incremented_value, 0.75, 0.75))
    return np.asarray(output)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    """ COLORS """
    for cf in allcfs.crazyflies:
        cf.setParam("ring/effect", 7)
    rgb_bits = generateRGBColors(len(robot_indices))

    for cf, rgb in zip(allcfs.crazyflies, rgb_bits):
        cf.setLEDColor(*rgb)

    """ MISSION """
    allcfs.takeoff(Z, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1)

    # Get the number of timesteps
    num_timesteps = len(time_info)
    num_drones = np.minimum(len(allcfs.crazyflies), ALL_WAYPOINTS.shape[1])
    print(ALL_WAYPOINTS)
    # Assign positions to each drone at each timestep
    for t in range(1, num_timesteps):
        for i in range(num_drones):
            # Gets the correct crazyflie
            WAYPOINTS = ALL_WAYPOINTS[:, i, :]
            # Get the positions for the current timestep
            positions = WAYPOINTS[t, :]  # Gets a vector of all the row values in a specific column t
            cf = allcfs.crazyflies[i]
            cf.goTo(positions, 0.0, (time_info[t] - time_info[t-1]))
        timeHelper.sleep((time_info[t] - time_info[t-1]) - 0.4)

    # Land the drones
    allcfs.land(TARGET_HEIGHT, LAND_DURATION)
    timeHelper.sleep(LAND_DURATION + 1)

if __name__ == "__main__":
    main()