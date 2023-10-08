import numpy as np
import matplotlib.pyplot as plt

# This is overkill for this lab but I was interested in piecing together a framework for future labs/projects
# Talked with Kade abt this too, but the framework for much of this class based methodology for defining these
# robots is based on the Robotics Toolbox for Python which is very useful and very deep in terms of functionality
# The goal of this was to pull apart the framework and understand how it works, then build a simpler version of it.
# Lots of googling and ai used in here for learning and figuring out different sections but all of this is written by me
# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# General overview for how this works:
# We define a DHRobot class that has a list of links and a name, we can add more things to this later
# This is a supeerclass that we can use to define other robots (like the one in lab) as subclasses
# We define a DHLink class that has all the DH parameters for a given link and a function to calculate the A matrix
# We define a DHRevoluteArm class that is a subclass of DHLink that has the DH parameters for a revolute joint
# We do this in a class based format because each robot class holds all of the information about the robot which is useful

class DHRobot: # Robot superclass
    r"""
        General class for DH robot that includes, links, a name, and general kwargs
        We can use this alter as a way to build a framework for other robots
        Each time we want to "build" a new robot we can just create a new class that inherits from this one
        This is also where we will include oru forward and inverse kinematics functions
    """
    def __init__(
            self, links = [], name=None, **kwargs
    ):
        self.links = links
        self.name = name
        self.kwargs = kwargs

    def fkine(self, q=None, p=False):
        r"""
        Forward kinematics for robot, we iterate through each links A matrix and multiply them together

        """

        if q is None:
            print("No joint angles specified")
            return
        
        T = np.eye(4)
        M = []
        for i, link in enumerate(self.links):

            if q[i] < link.qlim[0] or q[i] > link.qlim[1]:
                print(f"Joint angle for link {i} out of limits | angle: {q[i]} | limits: {link.qlim}")
                return
            
            arr = link.A(q[i])
            T = T @ arr
            if p:
                M.append(T)
        return T, M

    def workspace(self, theta_values, granularity=10):
        r"""Takes in list of theta values and generates scatterplot of of robot workspace.
        theta_values: list of theta values for each joint
        granularity: number of points to sample for each theta value range"""
        
        if len(theta_values) != len(self.links):
            print("Invalid number of joint angles")
            print(f"Expected {len(self.links)} | Received {len(theta_values)}")
            return

        def generate_combinations(thetas, granularity=granularity):
            def expand_ranges(theta_values, granularity):
                expanded_values = []
                for value in theta_values:
                    if isinstance(value, (int, float)):
                        expanded_values.append([value])
                    elif isinstance(value, (list, tuple)) and len(value) == 2:
                        expanded_values.append(np.linspace(value[0], value[1], granularity))
                    else:
                        raise ValueError("Invalid theta value format")
                return expanded_values

            expanded_thetas = expand_ranges(thetas, granularity)
            combos = []

            def generate_combinations_recursive(current_combo):
                if len(current_combo) == len(thetas):
                    combos.append(tuple(current_combo))
                    return
                for val in expanded_thetas[len(current_combo)]:
                    generate_combinations_recursive(current_combo + [val])

            generate_combinations_recursive([])

            return combos
        
        theta_combos = generate_combinations(theta_values, granularity)
        # Initialize an empty array to store the workspace points
        workspace_points = []

        # Loop through each set of theta values
        for theta_combo in theta_combos:
            # Set the robot's joint angles to the current theta values
            pose, m = self.fkine(theta_combo)
            pos = pose[:3, 3]

            # Add the current position to the workspace points array
            workspace_points.append(pos)

        # Convert the workspace points to a NumPy array
        workspace_points = np.array(workspace_points)

        # Plot the workspace points
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2], c='b', marker='o')

        # Customize the plot as needed
        ax.set_xlabel('X-axis (mm)')
        ax.set_ylabel('Y-axis (mm)')
        ax.set_zlabel('Z-axi (mm)')
        ax.set_title('Robot Workspace')

        plt.show()

class DHLink: # General DH link superclass
    r"""
     This specifies the DH parameters for a general link (as a superclass)
     We will write classes for each type of link (prismatic, revolute, etc) then we can do math with them. 
     This is loosely based on the framework provided by Robotics Toolbox for python, but is much more simplified as we don't really care about mass related parameters at this stage

    units: mm-s-kg-rad
        -> robot uses rads so convert later, easier to keep in deg for now
    theta = joint angle
    d = link offset
    alpha = link twist
    a = link length
    sigma = 0 if revolute, 1 if prismatic (will change how we calculate A matrix)
    offset = joint variable offset (will be useful later)
    qlim = joint variable limits [min, max]
    flip = joint moves in opposite direction
    """

    def __init__(
        self, theta=0.0, d=0.0, alpha=0.0, a=0.0, sigma=0, offset=0.0, qlim=None, flip=False
    ):

        self.theta = theta
        self.d = d
        self.alpha = alpha
        self.a = a
        self.sigma = sigma
        self.offset = offset
        self.qlim = qlim
        self.flip = flip

    def A(self, q):
        r"""
        This is where we define a given DH tranformation matrix (A for array) for a given link
        We can call this function in conjunction with all the links in a robot
        to calculate the forward kinematics of the robot later on

        We can normally use the spatialmath library to do this as it handles SE3 ncicely, but 
        we will do it manually for purposes of **education** 

        inputs: self, q
        q is the joint angle

        outputs: A matrix for the given link as a 4x4 array
        """

        # This section defines our d and theta values based on whether the joint is revolute or prismatic
        # We also add in the offset value and flip if necessary
        if self.sigma == 0:
            theta = self.theta + q + self.offset
            d = self.d
        else:
            theta = self.theta
            d = self.d + q + self.offset

        if self.flip:
            theta = -theta

        a = self.a
        alph = self.alpha

        # This is where we define our 4x4 A matrix   
        matrix = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alph), np.sin(theta) * np.sin(alph), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alph), -np.cos(theta) * np.sin(alph), a * np.sin(theta)],
                          [0, np.sin(alph), np.cos(alph), d],
                          [0, 0, 0, 1]])

        return matrix
    
class DHRevoluteArm(DHLink): # Revolute joint subclass of DHLink
    r"""
    This specifies the DH parameters for a DH revolute link as a subclass of DHLink

    units: mm-s-kg-rad
        -> robot uses rads so convert later, easier to keep in deg for now
    theta = joint angle
    d = link offset
    alpha = link twist
    a = link length
    sigma = 0 if revolute, 1 if prismatic
    offset = joint variable offset
    qlim = joint variable limits [min, max]
    flip = joint moves in opposite direction
    """
    def __init__(
        self, d=0.0, a=0.0, alpha=0.0, offset=0.0, qlim=None, flip=False, **kwargs 
    ):
        
        theta = 0.0
        sigma = 0

        super().__init__(
            theta=theta,
            d=d,
            alpha=alpha,
            a=a,
            sigma=sigma,
            offset=offset,
            qlim=qlim,
            flip=flip,
            **kwargs
        )
    
class LabRobot(DHRobot): # Robot subclass with DH parameters for lab robot
    r"""
    This is the robot class for the 6 axis robot in the lab we haver access to.
    We use the classes defined above to define the DH parameters for the robot
    """

    def __init__(self):

        H1 = 162.5
        H2 = 99.7
        L1 = 425
        L2 = 392
        W1 = 133
        W2 = 259.6

        # Define the DH parameters for your robot
        d1, d2, d3, d4, d5, d6 = H1, 0, 0, W1, H2, W2 # Distance along prev z axes
        a1, a2, a3, a4, a5, a6 = 0, L1, L2, 0, 0, 0 # Distance between new and old x axes
        alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = -np.pi/2, 0, 0, -np.pi/2, np.pi/2, 0 # Angles between z axes
        qlim = [-np.pi, np.pi]

        # Create a robot model with DH parameters
        links = [
                DHRevoluteArm(d=d1, a=a1, alpha=alpha1 ,qlim=qlim),
                DHRevoluteArm(d=d2, a=a2, alpha=alpha2 ,qlim=qlim),
                DHRevoluteArm(d=d3, a=a3, alpha=alpha3 ,qlim=qlim),
                DHRevoluteArm(d=d4, a=a4, alpha=alpha4 ,qlim=qlim),
                DHRevoluteArm(d=d5, a=a5, alpha=alpha5 ,qlim=qlim),
                DHRevoluteArm(d=d6, a=a6, alpha=alpha6 ,qlim=qlim)]
        name="Lab Robot"

        super().__init__(links=links, name=name)
