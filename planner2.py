import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import sys
import os
sys_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(sys_path)
from pycrazyswarm import *
import threading
from threading import Thread


def create_graph(length, width, height, cell_size):
    num_cells_x = int(length / cell_size) + 1
    num_cells_y = int(width / cell_size) + 1
    num_cells_z = int(height / cell_size) + 1

    # Create a graph
    graph = nx.Graph()

    # Function to calculate the single digit ID from 3D node position
    def calculate_node_id(x, y, z):
        return x + y * num_cells_x + z * num_cells_x * num_cells_y + 1

    # Add nodes to the graph
    for i in range(num_cells_x):
        for j in range(num_cells_y):
            for k in range(num_cells_z):
                x, y, z = i * (cell_size) - ((cell_size) * num_cells_x // 2 ), j * (cell_size) - ((cell_size)* num_cells_y // 2), k * cell_size   # - num_cells_z // 2
                node_id = calculate_node_id(i, j, k)
                graph.add_node(node_id, position=(x, y, z))

    # Add edges to the graph
    delta = [
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1)
    ]
    for i in range(num_cells_x):
        for j in range(num_cells_y):
            for k in range(num_cells_z):
                node_id = calculate_node_id(i, j, k)

                # Connect with neighboring nodes
                for d in delta:
                    neighbor_pos = tuple((i + d[0], j + d[1], k + d[2]))
                    neighbor_id = calculate_node_id(*neighbor_pos)

                    # Check if the neighbor is within the grid
                    if all(0 <= neighbor_pos[dim] < num_cells for dim, num_cells in zip(range(3), [num_cells_x, num_cells_y, num_cells_z])):
                        graph.add_edge(node_id, neighbor_id)

    return graph

def get_nearest_node_id(graph, position):
    min_distance = float('inf')  # Initialize minimum distance to infinity
    nearest_node_id = None
    for node_id, node_data in graph.nodes(data=True):
        if 'position' in node_data and node_data['position'] == position:
            return node_id
        elif 'position' in node_data:
            node_position = node_data['position']
            distance = np.linalg.norm(np.array(node_position) - np.array(position))
            if distance < min_distance:
                min_distance = distance
                nearest_node_id = node_id

    return nearest_node_id

def a_star(graph, start_position, goal_position, cell_size):
    open_list = []    # Initialize the open list 
    closed_list = []  # Initialize the closed list

    start_node = get_nearest_node_id(graph, start_position)
    goal_node = get_nearest_node_id(graph, goal_position)

    if start_node is None or goal_node is None:
        return None  # Invalid start or goal position

    open_list.append(start_node)

    # Create dictionaries to store properties of nodes
    g_values = {start_node: 0}  # G: distance between the current node and the start node
    h_values = {start_node: heuristic(start_node, goal_node, graph)}  # H: estimated distance from the current node to the goal node
    f_values = {start_node: h_values[start_node]}  # F: total cost of the node
    parent_nodes = {start_node: None}

    while len(open_list) != 0:
        current_node = min(open_list, key=lambda node: f_values[node])  # Get the node with the least f value
        open_list.remove(current_node)  # Remove the current node from the open list
        closed_list.append(current_node)  # Add the current node to the closed list

        if current_node == goal_node:  # Goal node found
            path = reconstruct_path(current_node, parent_nodes, graph)
            return path

        # Generate children
        children = list(graph.neighbors(current_node))

        for child in children:
            if child in closed_list or graph.nodes[child]['position'] == (0, 0, 0):
                continue

            # Calculate the g, h, and f values
            distance_to_child = graph[current_node][child].get('weight', 1)  # Assumes weight = 1
            g = g_values[current_node] + distance_to_child
            if child not in g_values or g < g_values[child]:  # Update g value if it's better
                g_values[child] = g
                h_values[child] = heuristic(child, goal_node, graph)
                f_values[child] = g_values[child] + h_values[child]
                parent_nodes[child] = current_node

            if child not in open_list:  # Child is not in the open list
                open_list.append(child)

    return None  # No path found


def heuristic(node_id, goal_node, graph):
    # Calculate the heuristic using the Euclidean distance between the node's position and the goal position
    node_pos = graph.nodes[node_id]['position']
    goal_pos = graph.nodes[goal_node]['position']
    dx = node_pos[0] - goal_pos[0]
    dy = node_pos[1] - goal_pos[1]
    dz = node_pos[2] - goal_pos[2]
    return np.sqrt(dx**2 + dy**2 + dz**2)

def reconstruct_path(node, parent_nodes, graph):
    path = []
    current = node
    while current is not None:
        current_pos = graph.nodes[current]['position']
        path.append((current_pos[0], current_pos[1], current_pos[2]))
        current = parent_nodes[current]
    path.reverse()
    return path


""" RUNNING """

# Define the start and goal nodes
swarm = Crazyswarm()
allcfs = swarm.allcfs
cf = allcfs.crazyflies[0]
goal = (3, 2.3, 1.5)

def rob_pos_callback(msg):
    global start, goal
     # Acquire the lock before updating the start position
    position_lock.acquire()
    start = (msg.x, msg.y, msg.z)
    print(start)
    position_lock.release()

# def goal_pos_callback(msg):
    # goal = (msg.x, msg.y, msg.z)

# Initialize a lock for synchronized access to start and goal positions
position_lock = threading.Lock()


def planner(rate):
    # Publish the planned path at the specified rate
    rate = rospy.Rate(rate)
    path_pub = rospy.Publisher('planned_path', String, queue_size=10)

    # Create the three-dimensional graph
    length = 12
    width = 6
    height = 2
    cell_size = 0.5
    graph = create_graph(length, width, height, cell_size)

    while not rospy.is_shutdown():
        # Acquire the lock before accessing start and goal positions
        position_lock.acquire()
        if start is not None and goal is not None:
            # Find the path using space-time A*
            path = a_star(graph, start, goal, cell_size)
            # Convert the path to a string representation
            path_str = ""
            for point in path:
                path_str += f"{point[0]}, {point[1]}, {point[2]};"
            rospy.loginfo(path_str)
            path_pub.publish(path_str)
        position_lock.release()

        rate.sleep()

def main():
    # ROS node initialization
    rospy.init_node('subscriber', anonymous=True)

    # Start the planner thread
    Thread(target=planner, kwargs={"rate": 1}).start()

    # Subscriber for the "robot_pos" topic
    rospy.Subscriber('robot_pos', Point, rob_pos_callback)

    # Keep the node running until it's shut down
    rospy.spin()

if __name__ == '__main__':
    try:
        start = (0,0,0)
        goal = (3,2.3, 1.5)
        main()
    except rospy.ROSInterruptException:
        pass

"""  PLOTTING 

# Create a 3D plot to visualize the graph and path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the position of nodes for visualization
pos = {node_id: graph.nodes[node_id]['position'] for node_id in graph.nodes()}

# Draw nodes as scatter plot
ax.scatter([pos[node_id][0] for node_id in pos], [pos[node_id][1] for node_id in pos],
           [pos[node_id][2] for node_id in pos], c='black', marker='o', alpha= 0.2)

# Add edges to the plot
for edge in graph.edges():
    start_node, end_node = edge
    ax.plot([pos[start_node][0], pos[end_node][0]], [pos[start_node][1], pos[end_node][1]],
            [pos[start_node][2], pos[end_node][2]], c='gray', alpha= 0.2)

if path is not None:
    # Draw the path
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], [path[i][2], path[i + 1][2]], c='r')

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()

"""
