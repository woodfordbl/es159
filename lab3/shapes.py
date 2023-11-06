import matplotlib.pyplot as plt
import numpy as np

def create_color_gradient(num_points):
    # Define the start and end colors (blue and red)
    start_color = (0, 0, 1)  # Blue
    end_color = (1, 0, 0)    # Red

    # Create a linear gradient of colors
    colors = [(
        start_color[0] + (end_color[0] - start_color[0]) * i / (num_points - 1),
        start_color[1] + (end_color[1] - start_color[1]) * i / (num_points - 1),
        start_color[2] + (end_color[2] - start_color[2]) * i / (num_points - 1)
    ) for i in range(num_points)]

    return colors

# Trace an square in the xz plane that is centered at some value
def plot_square(center_x, center_y, center_z, side_length, num_points, show=False):
    half_side = side_length / 2
    step = side_length / (num_points - 1)

    x_points = np.linspace(center_x - half_side, center_x + half_side, num_points)
    z_points = np.linspace(center_z - half_side, center_z + half_side, num_points)

    edge_points = []

    # Add points along the top edge
    for x in x_points:
        edge_points.append([x, center_y, center_z + half_side])

    # Add points along the right edge
    for z in z_points[1:-1]:
        edge_points.append([center_x + half_side, center_y, z])

    # Add points along the bottom edge (in reverse order)
    for x in x_points[::-1]:
        edge_points.append([x, center_y, center_z - half_side])

    # Add points along the left edge (in reverse order)
    for z in z_points[-2:0:-1]:
        edge_points.append([center_x - half_side, center_y, z])

    # Convert the edge points to a NumPy array
    edge_points = np.array(edge_points)

    # create list of colors in gradient
    colors = create_color_gradient(len(edge_points))

    if show:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(edge_points[:, 0], edge_points[:, 1], edge_points[:, 2], c=colors, marker='o')

        # Change color of the points from red to blue as the points move along the path

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Square Centered at ({center_x}, {center_y}, {center_z}) with {num_points} Points along Edges')

        ax.set_xlim(center_x - half_side, center_x + half_side)
        ax.set_ylim(center_y - half_side, center_y + half_side)
        ax.set_zlim(center_z - half_side, center_z + half_side)

        plt.show()
    return edge_points