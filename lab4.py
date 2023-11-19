import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from lab4 import search


# Load the file to be searched
file = open("lab4/maps/maze_in_lab.txt", "r")

# Initializing function that formats the input file
def main():
    # First line is startx, starty, goalx, goaly
    startx, starty, goalx, goaly = file.readline().split()

    points = pd.DataFrame(columns=['xstart', 'ystart', 'xend', 'yend'])
    # Add rest of points to pd array
    for line in file:
        xstart, ystart, xend, yend = line.split()
        points = pd.concat([points, pd.DataFrame({'xstart': [int(xstart)], 'xend': [int(xend)], 'ystart': [int(ystart)], 'yend': [int(yend)]})], ignore_index=True)   

    xmin = min(points['xend'].min(), points['xstart'].min())
    ymin = min(points['yend'].min(), points['ystart'].min())

    # Add the min to all the points
    points['xstart'] = points['xstart'] - xmin
    points['xend'] = points['xend'] - xmin
    points['ystart'] = points['ystart'] - ymin
    points['yend'] = points['yend'] - ymin
    startx = int(startx) - xmin
    starty = int(starty) - ymin
    goalx = int(goalx) - xmin
    goaly = int(goaly) - ymin

    # Get the max x and y for the map
    xmax = max(points['xend'].max(), points['xstart'].max())
    ymax = max(points['yend'].max(), points['ystart'].max())

    # Create the obstacles
    obstacles = []
    for row in points.iterrows(): 
        obs_xstart, obs_ystart, obs_xend, obs_yend = row[1]
        obstacles.append(search.block_obstacle(int(obs_xstart),int(obs_ystart), int(obs_xend), int(obs_yend)))

    return xmax, ymax, startx, starty, goalx, goaly, obstacles, xmin, ymin

xmax, ymax, startx, starty, goalx, goaly, obstacles, scalex, scaley = main()

# Set the granularity
granularity = 20

# Create the map
maze = search.map(xmax=xmax, ymax=ymax, xstart=startx, ystart=starty, xgoal=goalx, ygoal=goaly, obs=obstacles, margin=29, granularity=granularity)

maze.plot(path=False)

# Run the search
maze.astar_search()

# Plot the path
maze.plot(path=True)

# Convert the path to the correct coordinates
path = maze.path
path = np.array(path)

# Scale the path back up
path[:,0] = path[:,0] * granularity + scalex
path[:,1] = path[:,1] * granularity + scaley

# Calculate the path length
path_length = 0
for i in range(len(path) - 1):
    path_length += np.linalg.norm(path[i+1] - path[i])

print(f"Path length: {path_length}mm")

plt.plot(path[:,0], path[:,1], 'r-')

# subtract 10 from all y coord
path[:,1] = path[:,1] - 30

# Add Z coordinate
z = np.zeros((len(path), 1))
z.fill(-80)
path = np.hstack((path, z))

# Save the path to a csv
np.savetxt("lab4.csv", path, delimiter=",")
