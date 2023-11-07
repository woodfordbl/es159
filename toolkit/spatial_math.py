import numpy as np
import scipy.linalg as la
import cmath as cm

def cot(x):
    # Returns the cotangent of x
    return 1/np.tan(x)

def skew(w):
    if np.shape(w) != (3,):
        print('Input vector must be 3x1')
        print(f"Shape of input vector: {np.shape(w)}")
        return
    x = w[0].astype(float)
    y = w[1].astype(float)
    z = w[2].astype(float)

    # Returns the skew-symmetric matrix of a given vector w
    skew = np.array([[0, -z, y],
                     [z, 0, -x],
                     [-y, x, 0]])
    return skew

def adj(T):
    r"""
    Returns adjoint representation of matrix T
    T: 4x4 transformation matrix
    """
    if T.shape != (4,4):
        print('Input matrix must be 4x4')
        return
    
    # Returns the adjoint matrix of a given transformation matrix T
    adj = np.zeros((6,6))
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    # Definition 3.20 
    adj[0:3, 0:3] = R
    adj[3:6, 3:6] = R
    adj[3:6, 0:3] = skew(p) @ R
    return adj

def decompSO3R(R):
    r"""
    Returns w, theta for given R in SO(3) where exp(w_hat*theta) = R
    """
    if R.shape != (3,3):
        print('Input matrix must be 4x4')
        return
         
    tr = np.trace(R)

    # Following algorithm based on section 3.2.3.3 of Modern Robotics
    if R == np.eye(4):
        # If R is identity matrix, then w = undefined and theta = 0
        theta = 0
        w = None

        return w, theta
    elif tr == -1:
        theta = np.pi

        if R[2,2] > -1:

            c = 1/np.sqrt(2*(1+R[2,2]))
            w_hat = c*np.array([R[0,2], R[1,2], R[2,2]+1])
    
            return w_hat, theta
        
        if R[1,1] > -1:

            c = 1/np.sqrt(2*(1+R[1,1]))
            w_hat = c*np.array([R[0,1], R[1,1]+1, R[2,1]])
    
            return w_hat, theta
        
        if R[0,0] > -1:

            c = 1/np.sqrt(2*(1+R[0,0]))
            w_hat = c*np.array([R[0,0]+1, R[1,0], R[2,1]])
    
            return w_hat, theta
        # If here then R is not a valid rotation matrix
        print('Error: Unable to compute w_hat and theta')
        return
    else:
        if tr == 3:
            theta = .0001 
        else:
            theta = cm.acos(0.5*(tr-1))
        
        w_skew = (1/(2*np.sin(theta)))*(R - R.T)

        
        w_hat = np.array([w_skew[2,1], w_skew[0,2], w_skew[1,0]])
        norm = la.norm(w_hat)
        # We want w_hat to be unit vector so we divide by norm
        return w_hat, theta
    
def logSE3(T):
    r"""
    Returns the matrix logarithm of T
    T: 4x4 transformation matrix
    """
    logT = np.zeros((4,4))

    
    if T.shape != (4,4):
        print('Input matrix must be 4x4')
        print(f"Shape of input matrix: {T.shape}")
        return
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    

    if np.array_equal(T, np.eye(4)):
        norm = la.norm(p)

        if norm == 0:
            print('Error: Unable to compute log(T), p = 0')
            return 
        
        v = p / norm
        theta = norm
        
        logT[3:,3] = v @ theta

        return logT
    else:
        w, theta = decompSO3R(R)
        
        skw_w = skew(w)
        G_inv = (1/theta)*np.eye(3) - (1/2)*skw_w + ((1/theta) - ((1/2)*cot(theta/2)))*np.dot(skw_w, skw_w)
        
        v = G_inv @ p
        # [S]ø =  | [w]ø    [v]ø  | 
        #         |   0      0    | 


        logT[0:3, 0:3] = skw_w*theta
        logT[0:3, 3] = v*theta

        return logT

def block(T):
    r"""
    Converts 6x1 matrix to block matrix form
    """

    if T.shape != (6,):
        print('Input matrix must be 6x1')
        return
    w = T[0:3]
    v = T[3:6]

    skw = skew(w)
    block = np.zeros((4,4))
    block[0:3, 0:3] = skw
    block[0:3, 3] = v
    return block

def unblock(T):
    r""""
    Converts block matrix to 6x1 matrix form
    """
    if T.shape != (4,4):
        print('Input matrix must be 4x4')
        return
    w = np.array([T[2,1], T[0,2], T[1,0]])
    v = T[0:3, 3]

    return np.concatenate((w,v))