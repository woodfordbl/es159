## NOTE: This does not work, the transformation matrices are incorrect within the links for M10 and M43
## NOTE: Did not end up fixing this


from toolkit import robo_toolkit as rt
import numpy as np

class Lab3b(rt.ScrewRobot):
    def __init__(self):

        L1 = 2
        L2 = 1
        L3 = 0.5
        # Define the orintation matrix for the base to tool transformation
        m = np.array([[1, 0, 0, L1+L3],
                      [0, 1, 0, 0],
                      [0, 0, 1, -L2],
                      [0, 0, 0, 1]])
        
        
        rho = 1
        radius = 0.1

        mass1 = rho * np.pi * radius**2 * L1    
        mass2 = rho * np.pi * radius**2 * L2    
        mass3 = rho * np.pi * radius**2 * L3    

        def inertial_matrix(mass, rad, l, axis):
            G = np.eye(6)
            G[3:, 3:] = np.eye(3) * mass
            if axis == 1:
                G[0,0] = 0.5*mass*rad**2
                G[1,1] = mass*(3*rad**2+l**2)/12
                G[2,2] = mass*(3*rad**2+l**2)/12
            if axis ==2:
                G[0,0] = mass*(3*rad**2+l**2)/12
                G[1,1] = 0.5*mass*rad**2
                G[2,2] = mass*(3*rad**2+l**2)/12
            if axis ==3:
                G[0,0] = mass*(3*rad**2+l**2)/12
                G[1,1] = mass*(3*rad**2+l**2)/12
                G[2,3] = 0.5*mass*rad**2
            return G

        G1 = inertial_matrix(mass1, radius, L1, 1)
        G2 = inertial_matrix(mass2, radius, L2, 3)
        G3 = inertial_matrix(mass3, radius, L3, 1)

        links = [
                rt.ScrewRevolute(w=[0,0,1], q=[0,0,0], qlim=None, mu=None, l=L1, rho= rho, rad=radius, G=G1, center=[0.5*L1,0,0]),
                rt.ScrewRevolute(w=[0,-1,0], q=[L1,0,0], qlim=None, mu=None, l=L2, rho= rho, rad=radius, G=G2, center=[L1,0,-0.5*L2]),
                rt.ScrewRevolute(w=[0,0,1], q=[0,0,-L2], qlim=None, mu=None, l=L3, rho= rho, rad=radius, G=G3, center=[L1+0.5*L3,0,-L2,])
                ]
        
        name="Lab3b Robot"
        gravity = [0,0,0,0,0,9.81]
        super().__init__(m=m, links=links, name=name, gravity=gravity)


robot = Lab3b()

thetas = [1,1,1]
dthetas = [-1,-1,-1]
ddthetas = [2,-1,0.5]
Ftip = [1,1,1,1,1,1]

taus = robot.idynamics(thetas=thetas, dthetas=dthetas, ddthetas=ddthetas, F = Ftip)

print(taus)

"""

thetas = [0,np.pi/2,1]
dthetas = [0,0,0]
ddthetas = [1,1,1]
Ftip = [0,0,0,0,0,0]

taus1 = robot.idynamics(thetas=thetas, dthetas=dthetas, ddthetas=ddthetas, F = Ftip)


thetas = [np.pi,0.75*np.pi,np.pi/2]
dthetas = [0,1,0]
ddthetas = [-0.1,0,2]
Ftip = [0,0,0,10,0,0]

taus2 = robot.idynamics(thetas=thetas, dthetas=dthetas, ddthetas=ddthetas, F = Ftip)


thetas = [-1,1,0.5]
dthetas = [2,0,10]
ddthetas = [2,5,-5]
Ftip = [5,0,0,0,0,0]

taus3 = robot.idynamics(thetas=thetas, dthetas=dthetas, ddthetas=ddthetas, F = Ftip)

print('---------'*20)
print(taus1)
print(taus2)
print(taus3)
"""