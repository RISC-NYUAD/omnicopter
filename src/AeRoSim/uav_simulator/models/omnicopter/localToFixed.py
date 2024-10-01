from sympy import *
import numpy as np

phi = Symbol('phi')
theta = Symbol('theta')
psi = Symbol('psi')

Rx = Matrix([[1,0,0],[0,cos(phi),-sin(phi)],[0,sin(phi),cos(phi)]])
Ry = Matrix([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
Rz = Matrix([[cos(psi),-sin(psi),0],[sin(psi),cos(psi),0],[0,0,1]])

POS_X = [0.352, -0.36, 0.368, 0.432, 0.072, -0.435, -0.031, -0.407]       
POS_Y = [0.357, -0.263, -0.29, 0.084, 0.447, -0.061, -0.449, 0.231]
POS_Z = [0.0024, 0.15, 0.023, 0.011, 0.085, -0.002, 0.0018, 0.05]
PHI = [-0.73, -1.22, 1.76, -2.59, 1.71, 3.08, 0.47, 1.05]
THETA = [-0.15, -0.5, -0.65, -0.04, 1.42, 0.01, 0.91, -0.55]

GZ_PHI = []
GZ_THETA = []
GZ_PSI = []

for i in range(len(PHI)):
    phi_n = PHI[i]
    theta_n = THETA[i]
    M = Rx.subs('phi',phi_n) * Ry.subs('theta', theta_n)
    th = asin(-M[2,0])
    ph = atan2(M[2,1]/cos(th), M[2,2]/cos(th))
    ps = atan2(M[1,0]/cos(th),M[0,0]/cos(th))
    GZ_PHI.append(ph.evalf())
    GZ_THETA.append(th.evalf())
    GZ_PSI.append(ps.evalf())

#print(GZ_PHI)
#print(GZ_THETA)
#print(GZ_PSI)
Am = np.array([])

f = Symbol('f')
r = Symbol('r')
f_p = Matrix([[0],[0],[f]])
m_p = Matrix([[0],[0],[r]])
for i in range(len(PHI)):
    phi_n = PHI[i]
    theta_n = THETA[i]
    Rpb = Rx.subs('phi',phi_n) * Ry.subs('theta', theta_n)
    f_b = Rpb * f_p
    l = Matrix([[POS_X[i]],[POS_Y[i]],[POS_Z[i]]])
    tau_b = l.cross(f_b) + Rpb * m_p
    print("-------------")
    print(f_b,tau_b)
    print("-------------")
"""
    v = np.array([[f_b[0],f_b[1],f_b[2],tau_b[0],tau_b[1],tau_b[2]]])
    if(i==0):
        Am = v.transpose()
    else:
        Am = np.hstack([Am,v.transpose()])
"""
