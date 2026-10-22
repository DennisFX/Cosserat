# -*- coding: utf-8 -*-
"""
Wrench monitor for the Cosserat needle (examples/python3/NeedleInsertion.py).

Put this file next to the scene (like rosLink.py) and add at the very end of
createScene(), after the CosseratNeedleSlidingConstraint is created:

    from wrenchMonitor import WrenchMonitor
    rootNode.addObject(WrenchMonitor(
        name="wrenchMonitor", rootNode=rootNode, needle=needle, solver=generic,
        slidingConstraint=distanceStatsNode.computeDistanceComponent,
        distanceMO=constraintPoinMo,
        youngModulus=PhysicsParams.youngModulus,
        poissonRatio=PhysicsParams.poissonRatio,
        radius=GeometryParams.radius,
        rayleighStiffness=PhysicsParams.rayleighStiffness,
        printEvery=20))

rayleighStiffness must match the EulerImplicitSolver the needle runs under.
Rayleigh damping turns the stiff base spring into a large damper, so the
spring alone reports huge forces whenever the base moves; the monitor
subtracts that damping term.

Results are refreshed after every time step and exposed as attributes:
    baseForce, baseTorque  wrench the needle holder applies ON the needle,
                           at the needle base, SOFA scene frame (negate it to get
                           what a force/torque sensor in the holder would read)
    sectionMoments         (nbSections, 3) internal moments per section,
                           [torsion, bending y, bending z], local section frame
    lateralForces          (nbConstraintPoints, 2) tissue forces on the needle
                           at each sliding point, needle-local [y, z]

Units follow the scene (cm, kg, s): force = kg.cm/s^2 = 0.01 N,
moment = kg.cm^2/s^2 = 1e-4 N.m.

Only needs numpy (SOFA's pixi environment has no scipy).
"""
import numpy as np
import Sofa


# Quaternions as [x, y, z, w], the order SOFA's Rigid3d uses.
def quatMultiply(a, b):
    """Hamilton product a * b (apply b, then a)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def quatInverse(q):
    q = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quatToRotvec(q):
    """Axis * angle of the shortest rotation, like RestShapeSpringsForceField."""
    q = np.asarray(q, dtype=float) / np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    s = np.linalg.norm(q[:3])
    if s < 1e-12:
        return 2.0 * q[:3]  # small-angle limit
    return q[:3] / s * (2.0 * np.arctan2(s, q[3]))


class WrenchMonitor(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        needle = kwargs["needle"]
        self.rootNode = kwargs["rootNode"]
        self.solver = kwargs.get("solver")
        self.sliding = kwargs.get("slidingConstraint")
        self.distanceMO = kwargs.get("distanceMO")
        self.printEvery = kwargs.get("printEvery", 0)

        # 1. Base: held by a RestShapeSpringsForceField named 'spring' (Cosserat prefab)
        self.baseMO = needle.rigidBaseNode.RigidBaseMO
        spring = needle.rigidBaseNode.spring
        self.kLin = float(np.atleast_1d(spring.stiffness.value)[0])
        self.kAng = float(np.atleast_1d(spring.angularStiffness.value)[0])
        self.rayleighStiffness = float(kwargs["rayleighStiffness"])

        # 2. Section stiffness, same formulas as BeamHookeLawForceField (circular section).
        #    If you built the needle with inertialParams, pass sectionStiffness=[GJ, EI, EI].
        if "sectionStiffness" in kwargs:
            self.K = np.asarray(kwargs["sectionStiffness"], dtype=float)
        else:
            E, nu, r = kwargs["youngModulus"], kwargs["poissonRatio"], kwargs["radius"]
            I = np.pi * r ** 4 / 4.0
            G = E / (2.0 * (1.0 + nu))
            self.K = np.array([G * 2.0 * I, E * I, E * I])
        self.strainMO = needle.cosseratCoordinateNode.cosseratCoordinateMO

        self.baseForce = np.zeros(3)
        self.baseTorque = np.zeros(3)
        self.sectionMoments = np.zeros((0, 3))
        self.lateralForces = np.zeros((0, 2))

    def onAnimateEndEvent(self, event):
        self.baseForce, self.baseTorque = self.computeBaseWrench()
        self.sectionMoments = self.computeSectionMoments()
        self.lateralForces = self.computeLateralForces()


    def computeBaseWrench(self):
        """Wrench the holder applies on the needle: spring reaction minus Rayleigh damping.

        Rayleigh stiffness damping adds -beta * k * v on the base, so the holder's net
        effect is k * (gap - beta * v). What's left is -(tissue wrench) + needle inertia.
        """
        x = np.asarray(self.baseMO.position.value[0], dtype=float)
        x0 = np.asarray(self.baseMO.rest_position.value[0], dtype=float)
        v = np.asarray(self.baseMO.velocity.value[0], dtype=float)  # [v, omega], world frame
        beta = self.rayleighStiffness
        force = self.kLin * ((x0[:3] - x[:3]) - beta * v[:3])
        # Same convention as RestShapeSpringsForceField<Rigid3>: dq = q_rest * q^-1
        dq = quatMultiply(x0[3:], quatInverse(x[3:]))
        torque = self.kAng * (quatToRotvec(dq) - beta * v[3:])
        return force, torque

    def computeSectionMoments(self):
        """Internal moment in each section: M_i = K (kappa_i - kappa0_i)."""
        k = np.asarray(self.strainMO.position.value, dtype=float)
        k0 = np.asarray(self.strainMO.rest_position.value, dtype=float)
        return (k - k0) * self.K

    def computeLateralForces(self):
        """Tissue forces from CosseratNeedleSlidingConstraint (2 lines per point: y, z)."""
        if self.solver is None or self.sliding is None or self.distanceMO is None:
            return np.zeros((0, 2))
        nPts = len(self.distanceMO.position.value)
        lam = np.asarray(self.solver.constraintForces.value, dtype=float)
        start = int(self.sliding.constraintIndex.value)
        if nPts == 0 or lam.size < start + 2 * nPts:
            return np.zeros((0, 2))
        # SOFA stores dt * lambda in constraintForces, so divide by dt to get forces
        dt = self.rootNode.dt.value
        return lam[start:start + 2 * nPts].reshape(nPts, 2) / dt