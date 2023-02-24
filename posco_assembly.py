#!/usr/bin/env python
import rospy
import numpy as np
import time

from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import (PlanningSceneCollisionCheck, 
NameVector, IntVector, isometry_to_vectors)
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

rospy.init_node('posco_env',anonymous=True)
roscpp_init('suhan', [])

pc = PlanningSceneCollisionCheck()
names = NameVector()
dofs = IntVector()

names.append('panda_left')
names.append('panda_right')
names.append('panda_top')

dofs.append(7)
dofs.append(7)
dofs.append(7)
pc.set_group_names_and_dofs(names,dofs)

SUB_TABLE_GROUND = 1.16511
BASE_GROUND = 1.001

coil_dim = np.array([0.4, 0.25]) # height, radius
coil_pos = np.array([0.5, 0.0, SUB_TABLE_GROUND + 0.25]) # do not make collision between the coil and support tables.
coil_quat = np.array([0.7071068, 0, 0, 0.7071068])
pc.add_cylinder(coil_dim, "coil", coil_pos, coil_quat)


inner1_dim = np.array([0.02, 0.1])
inner1_pos = np.array([0.95, 0.35, BASE_GROUND + 0.01]) # do not make collision between the coil and support tables.
inner1_quat = np.array([0.0, 0, 0, 1.])
pc.add_cylinder(inner1_dim, "inner1", inner1_pos, inner1_quat)

inner2_dim = np.array([0.02, 0.1])
inner2_pos = np.array([0.475, -0.525, SUB_TABLE_GROUND + 0.01]) # do not make collision between the coil and support tables.
inner2_quat = np.array([0.0, 0, 0, 1.])
pc.add_cylinder(inner2_dim, "inner2", inner2_pos, inner2_quat)

side1_dim = np.array([0.001, 0.25])
side1_pos = np.array([-0.3, 0.75, BASE_GROUND + 0.0005])
side1_quat = np.array([0.0, 0.0, 0.0, 1.0])
pc.add_cylinder(side1_dim, "side1", side1_pos, side1_quat)

side2_dim = np.array([0.001, 0.25])
side2_pos = np.array([0.2, 0.75, BASE_GROUND + 0.0005])
side2_quat = np.array([0.0, 0.0, 0.0, 1.0])
pc.add_cylinder(side2_dim, "side2", side2_pos, side2_quat)

while rospy.is_shutdown() is False:        
    pc.publish_planning_scene_msg()
    time.sleep(0.1)
    