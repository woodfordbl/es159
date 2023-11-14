import numpy as np
from . import spatial_math as sm
from scipy.linalg import expm, norm
import matplotlib.pyplot as plt
# Set np print
np.set_printoptions(precision=4, suppress=True)

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
#
# Update for lab 2:
# Added same thing as DH but used a screw axis reprresentation for continuity with the iline library
# Additionally began to build out spatial_math.py which includes several function for matrix math like skew, adj, and log


# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# Denavit Hartenberg Robot Class

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
        Forward kinematics for DH robot, we iterate through each links A matrix and multiply them together

        """

        if q is None:
            print("No joint angles specified")
            return
        
        T = np.eye(4)
        M = []
        for i, link in enumerate(self.links):
            theta = q[i]
            if theta < link.qlim[0] or theta > link.qlim[1]:
                print(f"Warning: joint angle for link {i} out of limits | angle: {theta} | limits: {link.qlim}")
            
            arr = link.A(theta)
            T = T @ arr
            if p:
                M.append(T)
        if p:
            return M
        else:
            return T

    def workspace(self, theta_values, granularity=10):
        r"""Takes in list of theta values and generates scatterplot of of robot workspace.
        theta_values: list of theta values for each joint
        granularity: number of points to sample for each theta value range
        _______________________________________________________________
        Example:
        theta_values = [
            0,
            [-np.pi/4, 0],
            [-np.pi/2,-np/6],
            [-np.pi/4, np.pi/4],
            0,
            0
        ]
        """
        
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
            pose = self.fkine(theta_combo)
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
        ax.set_zlabel('Z-axis (mm)')
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
    
class DHLabRobot(DHRobot): # Robot subclass with DH parameters for lab robot
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
        qlim = [-2*np.pi, 2*np.pi]

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


# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# Screw Axis Robot Class
class ScrewRobot: # Robot superclass for screw axis representation
    r"""
    General class for screw axis representation of a robot that includes, links, 
    a name, and general kwargs we can use this alter as a way to build a framework 
    for other robots. Each time we want to "build" a new robot we can just create 
    a new class that inherits from this one. This is also where we will include our
    forward and inverse kinematics functions.  
    m = 4x4 tranformation matrix from base to tool  
    links = list of links
    """

    def __init__(
            self, m, links = [], gravity = [0,0,0,0,0,0], name=None, **kwargs
    ):
        
        
        self.m = m
        self.adj_inv_m = sm.adj(np.linalg.inv(m))
        self.links = links
        self.num_links = len(links)
        self.gravity = gravity
        self.name = name
        self.kwargs = kwargs

        for i, link in enumerate(links):
            M = np.eye(4)
            M[:3,3] = link.center
            link.M = M

        for link in links:
            link.A = link.self_screw()

    def jacobian(self, q):
        r"""
        Space jacobian for screw axis robot
        q: joint angles
        """

        links = self.links

        T_list, S_list = self.fkine(q, p=True)

        J = np.zeros((6, len(links)))

        for i in len(links):
            T = T_list[i]
            S = S_list[i]
            J[:,i] = sm.adj(T) @ S
            
        return J
    
    def body_jacobian(self, q):
        r"""
        Body jacobian for screw axis robot
        q: joint angles
        """

        # Short notation to make easier
        n = self.num_links
        
        # Calculate body axis twist for each link
        T_list, s_list = self.fkine(q, p=True)
        b_list = [self.adj_inv_m @ s for s in s_list]
               
        # Pre populate the Jacobian matrix
        # J = 6xn
        J = np.zeros((6, n))
        
        # Each column in the jacobian is the adjoint of its transformation matrix multiplied by the body axis twist
        # Iterate over each column in jacobian
        for i in range(n):
            T =np.eye(4)
            # T = exp(-[B_n]*q_n). . .exp(-[B_i+1]*q_i+1)
            # A = Adj_T
            # J_bi = A @ B_i

            # We want to build the transformation matrix that goes from joint n to joint i+1
            # We use reversed list for this
            # For i = n, this won't iterate and instead T = I and TB_n = B_n which is what we need
            for j in reversed(range(i+1, n)):
                expT = expm(-sm.block(b_list[j]) * q[j])
                T = T @ expT

            # The i'th column of the body jacobian is the adjoint of the above calculation multiplied by the respecive B
            A = sm.adj(T)
            J[:,i] = A @ b_list[i]

        return J
        z
    def fkine(self, q, p=False):
        r"""
        Forward kinematics for screw axis robot, we iterate through each links exponential twist matrix and multiply them together
        q = joint angles
        p = BOOL, true will return list of transformation matrices and respective screwaxes
        """

        if q is None:
            print("No joint angles specified")
            return
        
        m = self.m # Base to tool transformation matrix
        
        t = np.eye(4)
        t_list = []
        s_list = []

        for i, link in enumerate(self.links):
            theta = q[i]
            #if theta > link.qlim[1] or theta < link.qlim[0]:
            #    print(f"Warning: joint angle for link {i} out of limits | angle: {theta} | limits: {link.qlim}")

            if p:
                expT, s = link.expT(theta, p=True)
                t  = t @ expT # Multiply transformation matrix by exponential twist matrix for each link
                t_list.append(t)
                s_list.append(s)
            else:
                t = t @ link.expT(theta)
        
        t = t @ m

        if p:
            return t_list, s_list
        return t
    
    def ikine(self, T_d, theta=0.0, linear_tol=0.01, angular_tol=0.01, opt=False):
        r"""
        Inverse kinematics using the Newton Raphton Method
        T_d: desired end effector pose
        theta: initial guess for joint angles
        linear_tol: linear tolerance for convergence
        angular_tol: angular tolerance for convergence
        """

        if len(theta) != self.num_links:
            print("Invalid number of joint angles")
            print(f"Expected {self.num_links} | Received {len(theta)}")
            return

        def error(T_d, T):
            r"""
            Error function for inverse kinematics
            Takes in desired tranformation matrix and the current, returns angular and linear error
            T_d: desired transformation matrix
            T: current transforamtion matric
            """

            if np.shape(T_d) != np.shape(T):
                print("Cannot calculate error, invalid shape")
                return
            
            e = T_d - T
            e = abs(e)
            
            # Return linear and angular error
            angular_error = norm(e[:3,:3])
            linear_error = norm(e[:3,3])
            return angular_error, linear_error
        
        # Find the current transform matrix from world to end effector
        T_ab = self.fkine(theta)

        # Initialize starting error
        pos_error, ang_error = error(T_d, T_ab)

        # Until we reach our tolerance, iterate through the inverse kinematics algorithm
        # Here we use the Newton Raphton method to solve for the joint angles
        #
        # We start with the current joint angles and calculate the error
        # Then we calculate the body representation and use this to calculate V_b
        # We then calculate the body jacobian and use this to calculate the change in joint angles
        # We then update the joint angles and repeat until we reach our tolerance
        increment = 1
        while pos_error > linear_tol or ang_error > angular_tol:
            
            # Convert from world frame to bodyframe
            T_bd = np.linalg.inv(T_ab) @ T_d

            # Take the log to return the matrix form of body twist
            V_b = sm.logSE3(T_bd)

            # Decompose this back to a vector format v_b = [w  v]'
            v_b = sm.unblock(V_b)

            J = self.body_jacobian(theta)
            
            J_inv = np.linalg.pinv(J)
     
            theta = theta + J_inv @ v_b
            
            T_ab = self.fkine(theta)
            
            pos_error, ang_error = error(T_d, T_ab)

            # This specifies the maximum number of iterations before we stop and assume no convergence
            increment += 1
            if increment > 100:
                print("Failed to converge")
                return None
            
        print(f"Converged in {increment} iterations")
        
        # Convert to coterminal angles
        theta = self.coterm(theta)

        if opt:
            theta = self.optikine(T_d, theta)

        return theta
    
    def optikine(self, T_d, theta=0.0):
        r"""
        Inverse kinemeatics that takes into account optimal joint angles
        """
        links = self.links

        if len(theta) != self.num_links:
            print("Invalid number of joint angles")
            print(f"Expected {self.num_links} | Received {len(theta)}")
            return
        
        for i, link in enumerate(links):
            if link.mu is None:
                continue
            else:
                theta[i] = link.mu
        
        thetas = self.ikine(T_d, theta)
        thetas = self.coterm(thetas)

        return thetas
    
    def coterm(self, thetas=0.0):
        r"""
        Compares theta list to limit angles and returns list of coterminal angles that are in limits
        thetas: list of joint angles
        """
        if len(thetas) != self.num_links:
            print("Invalid number of joint angles")
            print(f"Expected {self.num_links} | Received {len(thetas)}")
            return
        
        try:
            for i, theta in enumerate(thetas):
                low, high = self.links[i].qlim
                A = theta
                while A <= low:
                    A += 2*np.pi
                while A >= high:
                    A -= 2*np.pi
                thetas[i] = A
        except:
            print("Unable to convert to coterminal angles")
            thetas = thetas
        return thetas

    def idynamics(self, thetas=[], dthetas=[], ddthetas=[], F=[]):
        r"""
        Inverse dynamics for a given robot using Newton Euler method
        thetas: joint angles
        dthetas: joint velocities
        ddthetas: joint accelerations
        F: external forces at the end effector
        """

        # Check to make sure we have the correct number of inputs
        if len(thetas) != self.num_links or len(dthetas) != self.num_links or len(ddthetas) != self.num_links:
            print("Invalid number of joint angles, velocities, or accelerations")
            print(f"Expected {self.num_links} | Received {len(thetas)} | {len(dthetas)} | {len(ddthetas)}")
            return
        
        # Initialize lists for V, AdT, adV, V_dot, taus, and Fs
        V = []
        AdT = []
        adV = []
        V_dot = []
        taus = []
        Fs =[]

        # Add the bases to the matrices
        V.append(np.zeros((6,)))
        V_dot.append(self.gravity)
        Fs.append(F)

        # Works on test cases
        for i, link in enumerate(self.links):
            print("Forward Link ", i+1)
            A = link.A

            T_i = expm(-sm.block(A) * thetas[i]) @ np.linalg.inv(link.M) @ self.links[i-1].M
            
            AdT_i = sm.adj(T_i)
            
            V_i = AdT_i @ V[i] + A * dthetas[i]
            adV_i = sm.lie(V_i)

            V_dot_i = sm.adj(T_i) @ V_dot[-1] + adV_i @ A *dthetas[i] + A * ddthetas[i]
            print(f"V{i+1}: {V_i}")
            print(f"V_dot{i+1}: {V_dot_i}")
            print(f"adV{i+1}: {adV_i}")
            print(f"AdT{i+1}{i}: {AdT_i}")
            
            print()
            
            V.append(V_i)
            AdT.append(AdT_i)
            adV.append(adV_i)
            V_dot.append(V_dot_i)

        T_last = np.linalg.inv(self.m) @ self.links[-1].M
        AdT.append(sm.adj(T_last))
        print()
        print("----"*30)
        print("----"*30)
        print()
        for i, link in enumerate(reversed(self.links), start=1):
            i = self.num_links - i
            print()
            print("Backward Link ", i+1)
            print(f"G {i+1} ", link.G)
            print(f"F_i+1", Fs[-1])
            print(f"AdT {i+2}{i+1}", AdT[i+1])
            print(f"C1", AdT[i+1].T @ Fs[-1])
            print(f"V_dot {i+1}", V_dot[i+1])
            print(f"C2", link.G @ V_dot[i+1])
            print(f"V {i+1}", V[i+1])
            print(f"C3", adV[i].T @ (link.G @ V[i]))

            # Stuck here this section is only part that doesnt work :(
            F_i = AdT[i+1].T @ Fs[-1] + link.G @ V_dot[i+1] - adV[i].T @ (link.G @ V[i+1])
            Fs.append(F_i)
            tau = F_i.T @ link.A
            print("F_i: ", F_i)
            taus.append(tau)
        taus.reverse()
        return taus

class ScrewLink:
    r"""
    This specifies the screw axis parameters for a general link (as a superclass)
    q: 3x1 vector of joint axis coordinates
    w: 3x1 vector of joint axis rotation
    theta: joint angle
    ax: if revolute, 3x1 vector of movement axis
    qlim: joint variable limits [min, max]
    sigma: 0 if revolute, 1 if prismatic
    mu: optimal joint angle, None if not applicable
    """

    def __init__(
            self, w=[0,0,0], q=[0,0,0], v = [0,0,0], theta = 0.0, qlim = [0,0], l = 0.0, rho = 0.0, rad = 0.0, mass = None,  G = [], center = [], sigma = 0, mu = None, **kwargs
    ):
        # Check shape of q, w, v
        if len(q) != 3 or len(w) != 3 or len(v) != 3:
            print("Invalid screw axis parameters")
            return
        if not mass:
            if not rho or not rad:
                print("No mass or density and radius specified for link")
            else:
                mass = rho * rad**2 * np.pi * l

        s = np.concatenate((w, v))
        
        def inertial_matrix(self):
            G = np.eye(6)
            G[3:, 3:] = np.eye(3) * self.mass
            G[1,1] = 0.25 * self.mass * (self.rad**2 * self.rad**2)
            G[2,2] = 0.5 * self.mass * (self.l**2)
            G[3,3] = 0.25 * self.mass * (self.rad**2 * self.rad**2)
            return G


        self.q = q
        self.w = w
        self.v = v
        self.s = s
        self.A = None
        self.theta = theta
        self.qlim = qlim
        self.rho = rho
        self.rad = rad
        self.mass = mass
        self.sigma = sigma
        self.M = None
        self.G = G
        self.center = center
        self.mu = mu
        self.kwargs = kwargs
    
    def expT(self, theta, p=False):
        r""""
        This is where we define an exponential twist matrix for a given link
        theta = joint angle
        p = True if we want to return the 6x1 s matrix as well
        """

        w = self.w

        # Represent the skew symmetric matrix of w
        sks_w = sm.skew(w)

        S = np.zeros((4,4))

        # Populate the s matrix
        S[:3,:3] = sks_w
        S[:3,3] = self.v

        # Calculate the exponential twist matrix
        expT = expm(S * theta)
        if p:
            
            return expT, self.s
        
        return expT

    def self_screw (self):
        M_inv = np.linalg.inv(self.M)
        adj_M_inv = sm.adj(M_inv)
        A = adj_M_inv @ self.s
        return A
    
class ScrewRevolute(ScrewLink):
    r"""
    This specifies the screw axis parameters for a revolute link (as a subclass)
    q = 3x1 vector of joint axis coordinates
    w = 3x1 vector of joint axis rotation
    """
    def __init__(
            self, q, w, qlim, mu, **kwargs
    ):
        
        q = np.array(q)
        w = np.array(w)
        v = np.cross(-w, q)

        sigma = 0

        super().__init__(
            q=q,
            w=w,
            v=v,
            qlim=qlim,
            sigma=sigma,
            mu=mu,
            **kwargs
        )

class ScrewPrismatic(ScrewLink):
    r"""
    This specifies the screw axis parameters for a prismatic link (as a subclass)
    """

    # TODO: Implement screw axis parameters for prismatic link
    # Not needed for lab 2

class ScrewLabRobot(ScrewRobot):
    def __init__(self):

        H1 = 162.5
        H2 = 99.7
        L1 = 425
        L2 = 392.2
        W1 = 133.3
        W2 = 259.6

        qlim = [-2*np.pi, 2*np.pi]

        # Define the orintation matrix for the base to tool transformation
        m = np.array([[-1, 0, 0, L1+L2],
                      [0, 0, 1, W1+W2],
                      [0, 1, 0, H1-H2],
                      [0, 0, 0, 1]])
                      

        # Create a robot model with DH parameters
        links = [
                ScrewRevolute(w=[0,0,1], q=[0,0,0], qlim=qlim, mu=None),
                ScrewRevolute(w=[0,1,0], q=[0,0,H1], qlim=qlim, mu=0),
                ScrewRevolute(w=[0,1,0], q=[L1,0,H1], qlim=qlim, mu=0),
                ScrewRevolute(w=[0,1,0], q=[L1+L2,0,H1], qlim=qlim, mu=None),
                ScrewRevolute(w=[0,0,-1], q=[L1+L2,W1,0], qlim=qlim, mu=0),
                ScrewRevolute(w=[0,1,0], q=[L1+L2,0,H1-H2], qlim=qlim, mu=0)
                ]
        
        name="Lab Robot"

        super().__init__(m=m, links=links, name=name)

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# This section will be for general tools to generate coordinate paths for the robot

def plot_path(robot, coords=[0,0,0], guess=[], pose=[], time = 0.0, delta=False):
    r"""
    This function takes in a robot and a list of coordinates and generates a path for the robot to follow
    robot: robot object
    coords: list of coordinates
    guess: initial guess for joint angles
    pose: desired, held pose for the robot
    time: time to complete path
    """

   
    num_links = robot.num_links
   
    # Calculate the time step for each point
    time_step = time / len(coords)
    
    # Initialize joint angles, velocities, and deltas
    joint_angles = []
    test = []
    velocities = []
    deltas = []

    T = np.array([[0, -1, 0, 0],[0, 0, -1, 0],[1, 0, 0 ,0],[0, 0, 0, 1]])
    T[:3, :3] = pose

    if len(guess) != num_links:
        print("Invalid number of joint angles")
        return
    
    for i, coord in enumerate(coords):

        # Calculate the inverse kinematics for the given coordinates
        # Create the pose
        T[:3, 3] = coord

        joint_angle = robot.ikine(T, guess)

        test = robot.fkine(joint_angle)
        test_pos = test[:3, 3]

        joint_angles.append(joint_angle)
        deltas.append(joint_angle - guess)

        if i != 0:
            # append to velocities
            velo = (joint_angle - guess)/time_step
            velocities.append(velo)
        
        # Update the guess for the next iteration
        guess = joint_angle
        
    # Add the last velocity, which is zero
    velocities[-1] = np.zeros(num_links)
    if delta:
        return joint_angles, velocities, deltas
    return joint_angles, velocities