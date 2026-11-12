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
in the gel (scene units: 0.01 N per cm). The shaft in the gel, from the entry point to the
tip, is split between the points at the midpoints between them, and each point gets the
friction of its share. In total that is the friction per length times the inserted length,
so the friction grows smoothly with depth instead of in a step at every new point. Points
ahead of the tip get none: while retracting, the deepest point stays until the tip is
constraintDistance above it.

Two layers: shaft at or past scene x = hardLayerTop (the top of the gel's hard layer,
createFemRegularGrid.hardLayerTop) gets hardPerLength per cm, the rest perLength. A share
that straddles the layer top is split there, so the friction's slope with depth changes when
the tip crosses into the hard layer, not when the next point is added. The layer top is its
rest position: the gel near the needle moves a mm or two along it, which this ignores.

Depth is measured along the straight line from the needle base to its tip, which is exact
for the rigid needle and close for the flexible one.

Add it after the CosseratNeedleSlidingConstraint:

    from shaftFriction import ShaftFriction
    rootNode.addObject(ShaftFriction(
        name="shaftFriction", constraint=distanceStatsNode.computeDistanceComponent,
        needlePoints=slidingPoint.slidingPointMO,
        gelPoints=constraintPointNode.constraintPointsMo,
        perLength=ConstraintsParams.frictionPerLength,
        hardPerLength=ConstraintsParams.hardFrictionPerLength,
        hardLayerTop=hardLayerTop(FemParams)))

hardPerLength defaults to perLength, and hardLayerTop to None (one gel, perLength
throughout). perLength and hardPerLength both 0 leave `friction` alone (empty: no friction,
unless frictionCoefficient is set). The limits used in the last step are in `limits` (scene
units, one per gel point).
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
        self.hardPerLength = float(kwargs.get('hardPerLength', self.perLength))
        top = kwargs.get('hardLayerTop')
        self.hardLayerTop = None if top is None else float(top)
        self.limits = np.zeros(0)

    def onAnimateBeginEvent(self, event):
        if self.perLength <= 0.0 and self.hardPerLength <= 0.0:
            return
        soft, hard = self.shaftShares()
        self.limits = self.perLength * soft + self.hardPerLength * hard
        # [0.0] rather than [] outside the gel: one value for all of no points
        self.constraint.friction.value = self.limits.tolist() if self.limits.size else [0.0]

    def shaftShares(self):
        """Length of shaft in the gel each gel point stands for, in cm: (soft, hard layer)."""
        gel = np.asarray(self.gelPoints.position.value, dtype=float).reshape(-1, 3)
        soft, hard = np.zeros(len(gel)), np.zeros(len(gel))
        needle = np.asarray(self.needlePoints.position.value, dtype=float).reshape(-1, 3)
        if len(gel) == 0 or len(needle) < 2:
            return soft, hard
        tip = needle[-1]
        axis = tip - needle[0]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return soft, hard
        axis = axis / norm
        depth = (gel - tip) @ axis  # along the needle, tip at 0, shaft below 0
        onShaft = np.flatnonzero(depth <= 0.0)
        if onShaft.size == 0:
            return soft, hard
        order = onShaft[np.argsort(depth[onShaft])]
        d = depth[order]
        edges = np.concatenate(([d[0]], 0.5 * (d[1:] + d[:-1]), [0.0]))
        length = np.diff(edges)
        # Scene x at both ends of each share; the layers are planes of constant x
        inHard = length * self.hardFraction(tip[0] + edges[:-1] * axis[0],
                                            tip[0] + edges[1:] * axis[0])
        hard[order] = inHard
        soft[order] = length - inHard
        return soft, hard

    def hardFraction(self, xa, xb):
        """Fraction of each straight piece of shaft, from scene x xa to xb, in the hard layer."""
        if self.hardLayerTop is None:
            return np.zeros_like(xa)
        deeper = np.maximum(xa, xb)
        span = np.abs(xb - xa)
        flat = span < 1e-12  # across the needle: all in one layer
        fraction = np.where(flat, (deeper >= self.hardLayerTop).astype(float),
                            (deeper - self.hardLayerTop) / np.where(flat, 1.0, span))
        return np.clip(fraction, 0.0, 1.0)
