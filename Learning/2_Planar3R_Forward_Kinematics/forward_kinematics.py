import numpy as np
from types import SimpleNamespace

def forward_kinematics(q,parms):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]

    l1,l2,l3 = parms;

    c1 = np.cos(theta1); s1 = np.sin(theta1)
    c2 = np.cos(theta2); s2 = np.sin(theta2)
    c3 = np.cos(theta3); s3 = np.sin(theta3)
    

    #Compute H01, H12, H23

    H01 = np.array([
        [c1, -s1, 0, 0],
        [s1, c1, 0, 0],
        [0,0,1,0],
        [0, 0,0, 1]
    ])

    H12 = np.array([
        [c2, -s2, 0, l1],
        [s2, c2, 0, 0],
        [0,0,1,0],
        [0, 0,0, 1]
    ])

    H23 = np.array([
        [c3, -s3, 0, l2],
        [s3, c3, 0, 0],
        [0,0,1,0],
        [0, 0,0, 1]
    ])

    #Compute H03
    H03 = H01@H12@H23 #A.dot(B) #np.matmul 

    #Compute xe,ye, orientation
    E3 = np.array([l3, 0, 0, 1])
    E0 = np.matmul(H03,E3)

    e = np.array([E0[0],E0[1]])

    #more calculations for animation
    o = np.array([0,0])
    P0 = H01@H12@np.array([0,0,0,1])
    p = np.array([P0[0],P0[1]])

    Q0 = H03@np.array([0,0,0,1])
    q = np.array([Q0[0],Q0[1]])

    sol = SimpleNamespace(
         o=o,p=p,q=q,e=e
        )
    
    return sol