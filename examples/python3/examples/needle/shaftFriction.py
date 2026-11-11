# -*- coding: utf-8 -*-
"""
Friction along the needle shaft (examples/python3/NeedleInsertion.py).

Once the needle is in the gel, CosseratNeedleSlidingConstraint holds each gel point (the
constraint points the needle controller adds every ConstraintsParams.constraintDistance) on
the needle axis. With its `friction` data set it also holds them along the axis, with
Coulomb stick-slip: a point moves with the shaft until that takes more than

    friction[i] + frictionCoefficient * |sideways force at point i|

and then slides, with that force opposing the slip. The needle feels the opposite force, so
it reaches the base wrench and Gazebo.

This controller sets friction[i] before every step from a friction per unit length of shaft
in the gel (perLength, scene units: 0.01 N per cm). The shaft in the gel, from the entry
point to the tip, is split between the points at the midpoints between them, and each point
gets perLength times its share. In total that is perLength * (inserted length), so the
friction grows smoothly with depth instead of in a step at every new point. Points ahead of
the tip get none: while retracting, the deepest point stays until the tip is
constraintDistance above it.

Depth is measured along the straight line from the needle base to its tip, which is exact
for the rigid needle and close for the flexible one.

Add it after the CosseratNeedleSlidingConstraint:

    from shaftFriction import ShaftFriction
    rootNode.addObject(ShaftFriction(
        name="shaftFriction", constraint=distanceStatsNode.computeDistanceComponent,
        needlePoints=slidingPoint.slidingPointMO,
        gelPoints=constraintPointNode.constraintPointsMo,
        perLength=ConstraintsParams.frictionPerLength))

perLength 0 leaves `friction` alone (empty: no friction, unless frictionCoefficient is set).
The limits used in the last step are in `limits` (scene units, one per gel point).
"""
import numpy as np
import Sofa


class ShaftFriction(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.constraint = kwargs['constraint']
        self.needlePoints = kwargs['needlePoints']  # along the needle, base first, tip last
        self.gelPoints = kwargs['gelPoints']  # gel constraint points, entry point first
        self.perLength = float(kwargs.get('perLength', 0.0))
        self.limits = np.zeros(0)

    def onAnimateBeginEvent(self, event):
        if self.perLength <= 0.0:
            return
        self.limits = self.perLength * self.shaftShares()
        # [0.0] rather than [] outside the gel: one value for all of no points
        self.constraint.friction.value = self.limits.tolist() if self.limits.size else [0.0]

    def shaftShares(self):
        """Length of shaft in the gel that each gel point stands for, in cm."""
        gel = np.asarray(self.gelPoints.position.value, dtype=float).reshape(-1, 3)
        shares = np.zeros(len(gel))
        needle = np.asarray(self.needlePoints.position.value, dtype=float).reshape(-1, 3)
        if len(gel) == 0 or len(needle) < 2:
            return shares
        tip = needle[-1]
        axis = tip - needle[0]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return shares
        depth = (gel - tip) @ (axis / norm)  # along the needle, tip at 0, shaft below 0
        onShaft = np.flatnonzero(depth <= 0.0)
        if onShaft.size == 0:
            return shares
        order = onShaft[np.argsort(depth[onShaft])]
        d = depth[order]
        edges = np.concatenate(([d[0]], 0.5 * (d[1:] + d[:-1]), [0.0]))
        shares[order] = np.diff(edges)
        return shares
