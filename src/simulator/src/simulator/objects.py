"""objects
Common bodies
Author: Simon Armleder
"""

import pybullet as pb
import numpy as np
from simulator.body import Body

class Cube(Body):
    def __init__(self, simulator, mass, position=[0,0,0], halfExtents=[1,1,1], orientation=[0,0,0,1], color=[1,0,0,1], use_fixed_base=False):
        super().__init__(simulator, position=position, orientation=orientation, use_fixed_base=use_fixed_base)

        collision_shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=halfExtents)
        visual_shape = pb.createVisualShape(pb.GEOM_BOX, halfExtents=halfExtents, rgbaColor=color)
        self._half_extends = halfExtents
        self._id = pb.createMultiBody(mass,
                                      collision_shape,
                                      visual_shape,
                                      position,
                                      orientation,
                                      linkMasses=[],
                                      linkCollisionShapeIndices=[],
                                      linkVisualShapeIndices=[],
                                      linkPositions=[],
                                      linkOrientations=[],
                                      linkInertialFramePositions=[],
                                      linkInertialFrameOrientations=[],
                                      linkParentIndices=[],
                                      linkJointTypes=[],
                                      linkJointAxis=[])
        if use_fixed_base:
            pb.createConstraint(self._id, -1, -1, -1, pb.JOINT_FIXED, [0,0,0], [0,0,0], position)

    def halfExtents(self):
        return self._half_extends

class Ball(Body):
    def __init__(self, simulator, mass, position=[0,0,0], radius=1, color=[1,0,0,1], use_fixed_base=False):
        super().__init__(simulator, position=position, orientation=[0,0,0,1], use_fixed_base=use_fixed_base)

        collision_shape = pb.createCollisionShape(pb.GEOM_SPHERE, radius=radius)
        visual_shape = pb.createVisualShape(pb.GEOM_SPHERE, radius=radius, rgbaColor=color)
        self._radius = radius
        self._id = pb.createMultiBody(mass,
                                      collision_shape,
                                      visual_shape,
                                      position,
                                      [0,0,0,1],
                                      linkMasses=[],
                                      linkCollisionShapeIndices=[],
                                      linkVisualShapeIndices=[],
                                      linkPositions=[],
                                      linkOrientations=[],
                                      linkInertialFramePositions=[],
                                      linkInertialFrameOrientations=[],
                                      linkParentIndices=[],
                                      linkJointTypes=[],
                                      linkJointAxis=[])
        if use_fixed_base:
            pb.createConstraint(self._id, -1, -1, -1, pb.JOINT_FIXED, [0,0,0], [0,0,0], position)
    
    def radius(self):
        return self._radius