import math
import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation as R

def euler_angles_from_vectors(v_before, v_after):

    v_before = v_before / np.linalg.norm(v_before)
    v_after = v_after / np.linalg.norm(v_after)
    
    # Calculate the rotation matrix
    R_mat = np.dot(v_after[:, np.newaxis], v_before[:, np.newaxis].T)
    
    # Convert the rotation matrix to Euler angles
    r = R.from_matrix(R_mat)
    euler_angles = r.as_euler('xyz', degrees=False)
    
    return euler_angles

def spherical_to_cartesian(r, theta, phi):
    x = round(r * math.sin(phi) * math.cos(theta), 8)
    y = round(r * math.sin(phi) * math.sin(theta), 8)
    z = round(r * math.cos(phi), 8)
    return x, y, z

motorsa = [13.261, 67.819, 106.134,144.45,182.765,250.547,296.630,334.946]
motorse = [0 ,33.641, 0, 0, 5.404,33.641,0, 17.703]
rs = [0.497, 0.497, 0.397,0.436,0.421,0.454,0.497,0.446]

motorsaa = [72.567,334.798,196.134,234.450,94.266,179.549,346.174,268.665]
motorsee = [344.356,23.169,39.013,303.835,344.524,37.455,31.292,308.433]
rrs = [1 ,1 , 1,1,1,1,1,1]

phii = []
thetaa = []
psii = []
euler_angles =[]

for i in range(len(motorsa)):
    r = rs[i]
    rr = rrs[i]
    a = motorsa[i] * math.pi / 180
    e = (90 - motorse[i]) * math.pi / 180 
    aa = motorsaa[i] * math.pi / 180
    ee = (90 - motorsee[i]) * math.pi / 180 
    x, y, z = spherical_to_cartesian(r, a, e)
    xx, yy, zz = spherical_to_cartesian(rr, aa, ee)

    phii.append(round(asin(xx),3))
    thetaa.append(round(atan2(yy,zz),3))
    psii.append(0)
    print(f"motor[{i}]: {x} {y} {z}")
    #print(f"motor[{i}]: phii={phii[i]}, thetaa={thetaa[i]}, psii={psii[i]}")

    euler_angles.append(euler_angles_from_vectors([0, 0 ,1],[xx, yy, zz]))
    print(f"motor[{i}]: {euler_angles[i]}\n")
