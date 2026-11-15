#!/usr/bin/env python3
"""Derive constant thrust allocation and aggregate rigid inertia from this SDF.
Run from any directory. Requires numpy and PyYAML. Output is deterministic.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE.parent / 'uav_simulator/worlds/models/omnicopter/model.sdf'

def rotation(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def load_model():
    model=ET.parse(MODEL).find('model')
    bodies=[]
    for link in model.findall('link'):
        pose=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')
        rot=rotation(pose[3:]); inertial=link.find('inertial')
        if inertial is None: continue
        ipose=np.fromstring(inertial.findtext('pose','0 0 0 0 0 0'),sep=' ')
        mass=float(inertial.findtext('mass')); ir=rot@rotation(ipose[3:])
        node=inertial.find('inertia')
        get=lambda key:float(node.findtext(key,'0'))
        inertia=np.array([[get('ixx'),get('ixy'),get('ixz')],
                          [get('ixy'),get('iyy'),get('iyz')],[get('ixz'),get('iyz'),get('izz')]])
        bodies.append((mass,pose[:3]+rot@ipose[:3],ir@inertia@ir.T))
    mass=sum(x[0] for x in bodies); com=sum(m*p for m,p,_ in bodies)/mass
    inertia=sum(j+m*(np.dot(p-com,p-com)*np.eye(3)-np.outer(p-com,p-com)) for m,p,j in bodies)
    plugin=model.find("plugin[@name='BetaMotorPlugin']")
    directions=[1 if int(s)>0 else -1 for s in plugin.findtext('turning_directions').split()]
    drag=float(plugin.findtext('drag_const')); columns=[]
    for i in range(8):
        link=model.find(f"link[@name='rotor_{i}']")
        pose=np.fromstring(link.findtext('pose'),sep=' ')
        d=rotation(pose[3:])@np.array([0.,0.,1.])
        columns.append(np.r_[d,np.cross(pose[:3]-com,d)-directions[i]*drag*d])
    A=np.array(columns).T
    assert np.linalg.matrix_rank(A)==6
    base=model.find("link[@name='base_link']")
    base_com=np.fromstring(base.find('inertial').findtext('pose'),sep=' ')[:3]
    return mass,com-base_com,inertia,A,float(plugin.findtext('thrust_const')),float(plugin.findtext('throttle'))

if __name__=='__main__':
    mass,offset,J,A,k,rpm=load_model()
    rpm_ceiling=min(rpm,max(0.,-0.0101*1900**2+61.8029*1900-52696))
    ceiling=k*(rpm_ceiling*0.10472)**2
    values=dict(mass=mass,allocation=A.ravel().tolist(),inertia=J.ravel().tolist(),
        thrust_min=[0.]*8,thrust_max=[float(ceiling)]*8,
        position_kp=[15.,15.,18.],position_kd=[10.,10.,12.],position_ki=[6.,6.,8.],
        attitude_kp=[8.5]*3,attitude_kd=[6.5]*3,frequency=250.,
        beta=1e-5,gamma=.1,actuator_reserve=1.5,allocation_force_weight=100.,allocation_moment_weight=10.,
        max_solve_seconds=.002,max_evaluations=80,constraint_tolerance=1e-6,
        governor_warm_budget=.001,governor_seed_budget=.002,recovery_max_evaluations=120,
        position_error_limit=.5,velocity_error_limit=1.,integral_limit=1.,state_timeout=.1,
        allocation_mode='governed',gravity=9.81)
    (PACKAGE/'config/failure_aware.yaml').write_text(yaml.safe_dump({'failure_controller':{'ros__parameters':values}},sort_keys=False))
    # Retain all original geometry/inertia; only create a separate failure model.
    tree=ET.parse(MODEL);model=tree.find('model');model.set('name','omnicopter_failure')
    plugin=model.find("plugin[@name='BetaMotorPlugin']")
    for tag,value in [('direct_thrust','true'),('thrust_tau_up','.15'),('thrust_tau_down','.21'),('command_timeout','.2')]:
        element=plugin.find(tag)
        if element is None:element=ET.SubElement(plugin,tag)
        element.text=value
    # Aggregate center of mass in the base-link frame, not the offset from its own COM.
    base_com=np.fromstring(model.find("link[@name='base_link']/inertial/pose").text,sep=' ')[:3]
    odom=model.find("plugin[@name='OdometrySensor']")
    elem=odom.find('com_position')
    if elem is None:elem=ET.SubElement(odom,'com_position')
    elem.text=' '.join(str(v) for v in offset+base_com)
    for uri in model.findall('.//uri'):
        if uri.text.startswith('meshes/'):uri.text='model://omnicopter/'+uri.text
    dest=MODEL.parent.parent/'omnicopter_failure';dest.mkdir(exist_ok=True)
    for invalid in tree.getroot().findall('physics'):tree.getroot().remove(invalid)
    tree.write(dest/'model.sdf',encoding='unicode',xml_declaration=True)
    (dest/'model.config').write_text('<model><name>omnicopter_failure</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><author><name>RISC</name></author><description>Failure simulation variant</description></model>')
    def nums(a):return ','.join(f'{v:.17g}' for v in np.array(a).ravel())
    (PACKAGE/'test/model_fixture.hpp').write_text('#pragma once\n#include "feasible_allocator.hpp"\nnamespace fixture {\n'
       'inline omni::A68 allocation(){omni::A68 a;a << '+nums(A)+';return a;}\n'
       'inline Eigen::Matrix3d inertia(){Eigen::Matrix3d j;j << '+nums(J)+';return j;}\n'
       f'constexpr double mass={mass:.17g};\nconstexpr double thrust_max={ceiling:.17g};\n}}\n')
    print('Derived from ROS 2 SDF: mass',mass,'thrust limit',ceiling)
