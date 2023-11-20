import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import heapq


# This is the implementation of the A* algorithm 
# The algorithm closely follows the code from this project: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
# My VS Code additionally emplyos Cursor: https://cursor.sh/ and Co-pilot: https://copilot.github.com/ to accelerate development
# These tools were used to add in the heapq and scaling functionality to help reduce runtime
# The code was modified to work with the map class and obstacles that are defined in this file and are gooing to be integrated into the toolkits in the future


class map:
    r"""
    A class used to represent a map which is a 2D grid with obstacles
    All maps start at (0,0) and end at (xmax, ymax)
    xmax: max x coordinate of the map
    ymax: max y coordinate of the map
    xstart: x coordinate of the start position
    ystart: y coordinate of the start position
    xgoal: x coordinate of the goal position
    ygoal: y coordinate of the goal position
    obs: list of obstacles of class obstacle in the map
    """

    def __init__(
            self, xmax, ymax, xstart, ystart, xgoal, ygoal, obs=[], margin=0, granularity=None, **kwargs
            ):
        
        if margin > 0:
            for obstacle in obs:
                obstacle.xmin = obstacle.xmin - margin
                obstacle.xmax = obstacle.xmax + margin
                obstacle.ymin = obstacle.ymin - margin
                obstacle.ymax = obstacle.ymax + margin
        
        # Make sure all the inputs are positive integers
        assert isinstance(xmax, int) and xmax > 0, "xmax must be a positive integer"
        assert isinstance(ymax, int) and ymax > 0, "ymax must be a positive integer"
        assert isinstance(xstart, int) and xstart > 0, "xstart must be a positive integer"
        assert isinstance(ystart, int) and ystart > 0, "ystart must be a positive integer"
        assert isinstance(xgoal, int) and xgoal > 0, "xgoal must be a positive integer"
        assert isinstance(ygoal, int) and ygoal > 0, "ygoal must be a positive integer"
        assert isinstance(obs, list), "obs must be a list of obstacles"
        
        self.xmax = xmax
        self.ymax = ymax
        self.xstart = xstart
        self.ystart = ystart
        self.xgoal = xgoal
        self.ygoal = ygoal
        self.obs = obs
        self.kwargs = kwargs
        self.namemap = kwargs.get('name', 'map')
        self.map = np.zeros((self.xmax, self.ymax))

        if granularity is not None:
            # Scale down the grid size by the granularity and round to the nearest integer
            self.xmax = int(round(self.xmax / granularity))
            self.ymax = int(round(self.ymax / granularity))
            self.xstart = int(round(self.xstart / granularity))
            self.ystart = int(round(self.ystart / granularity))
            self.xgoal = int(round(self.xgoal / granularity))
            self.ygoal = int(round(self.ygoal / granularity))

            for obstacle in self.obs:
                obstacle.xmin = int(round(obstacle.xmin / granularity))
                obstacle.xmax = int(round(obstacle.xmax / granularity))
                obstacle.ymin = int(round(obstacle.ymin / granularity))
                obstacle.ymax = int(round(obstacle.ymax / granularity))

            # Resize the map
            self.map = np.resize(self.map, (self.xmax, self.ymax))

        for obstacle in self.obs:
            if obstacle.xmin < 0:
                obstacle.xmin = 0
            if obstacle.xmax > self.xmax:
                obstacle.xmax = self.xmax
            if obstacle.ymin < 0:
                obstacle.ymin = 0
            if obstacle.ymax > self.ymax:
                obstacle.ymax = self.ymax

            self.map[(obstacle.xmin):(obstacle.xmax), (obstacle.ymin):(obstacle.ymax)] = 1
        
        self.path = None

    def plot(self, path=True, grid=False):
        r"""
        Plots the map and obstacles
        Grey rectangles are obstacles
        Red point is the start
        Green point is the goal
        :param path: if True, plots the path
        """

        # Create a figure and axis
        fig, ax = plt.subplots()

        # Plot each block as a rectangle
        for block in self.obs:
            rect = plt.Rectangle((block.xmin, block.ymin), block.xmax - block.xmin, block.ymax - block.ymin, facecolor='gray')
            ax.add_patch(rect)
        # Add start and goal as red points
        plt.plot(self.xstart, self.ystart, 'ro')
        plt.plot(self.xgoal, self.ygoal, 'go')

        # Plot the path
        if path:
            if self.path is None:
                print("No path generated, call astar_search() first")
                return
            for i in range(len(self.path) - 1):
                plt.plot([self.path[i][0], self.path[i+1][0]], [self.path[i][1], self.path[i+1][1]], 'b-')

        # Set the x and y limits of the plot
        ax.set_xlim(0, self.xmax)
        ax.set_ylim(0, self.ymax)

        if grid:
            # Set the grid on
            ax.set_xticks(np.arange(0, self.xmax, step=1))  # Set major x-ticks every 5 units
            ax.set_yticks(np.arange(0, self.ymax, step=1))  # Set major y-ticks every 5 units
            ax.grid(b=True, which='major', color='gray', linestyle='-', linewidth=1)

        # Show the plot
        plt.show()

    def astar_search(self):
        r"""
            A* search algorithm
        """
        # Initialize the start and goal nodes
        start_node = node(None, (self.xstart, self.ystart))
        start_node.g = start_node.h = start_node.f = 0

        goal_node = node(None, (self.xgoal, self.ygoal))
        goal_node.g = goal_node.h = goal_node.f = 0

        open_list = []
        closed_list = set()

        # Add the start to the open list
        heapq.heappush(open_list, start_node)

        # Create a dictionary to store nodes by position
        nodes = {start_node.position: start_node}

        # Going to loop until the open list is empty
        while len(open_list) > 0:
            current_node = heapq.heappop(open_list)
            closed_list.add(current_node)

            # Check if we have reached the goal
            if current_node == goal_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                
                self.path = path[::-1]
                return 

            # Generate children
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (self.xmax - 1) or node_position[0] < 0 or node_position[1] > (self.ymax - 1) or node_position[1] < 0:
                    continue

                # Make sure item is not an obstacle
                if self.map[node_position[0]][node_position[1]] != 0:
                    continue

                # Check if a node for this position already exists
                if node_position in nodes:
                    child = nodes[node_position]
                else:
                    # Create new node
                    child = node(current_node, node_position)
                    nodes[node_position] = child

                # Child is on the closed list
                if child in closed_list:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                
                # For euclidean distance, use the following 
                #child.h = (child.position[0] - goal_node.position[0])**2 + (child.position[1] - goal_node.position[1])**2
                
                # For manhattan distance, use the following instead
                child.h = abs(child.position[0] - goal_node.position[0]) + abs(child.position[1] - goal_node.position[1])
                
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                heapq.heappush(open_list, child)
           
class block_obstacle:
    r"""
    A class used to represent an obstacle
    xstart: x coordinate of the start position
    xend: x coordinate of the end position
    ystart: y coordinate of the start position
    yend: y coordinate of the end position
    """
    
    def __init__(self, xstart, ystart, xend, yend):

        assert isinstance(xstart, int) and xstart >= 0, "xstart must be a positive integer"
        assert isinstance(xend, int) and xend >= 0, "xstop must be a positive integer"
        assert isinstance(ystart, int) and ystart >= 0, "ystart must be a positive integer"
        assert isinstance(yend, int) and yend >= 0, "yend must be a positive integer"


        self.xmin = min(xstart, xend)
        self.xmax = max(xstart, xend)
        self.ymin = min(ystart, yend)
        self.ymax = max(ystart, yend)

class node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.parent, self.position, self.g, self.h))


