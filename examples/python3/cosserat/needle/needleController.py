__authors__ = "younesssss"
__contact__ = "adagolodjo@protonmail.com, yinoussa.adagolodjo@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2021,Inria"
__date__ = "March 8 2021"

import Sofa

import numpy as np
from Sofa import Cosserat
import Sofa
from cosserat.needle.params import ConstraintsParams


class Animation(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rigidBaseMO = args[0].rigidBaseNode.RigidBaseMO
        self.rateAngularDeformMO = args[
            0
        ].cosseratCoordinateNode.cosseratCoordinateMO  # unused

        self.needleSlidingState = args[0].cosseratFrame.slidingPoint.slidingPointMO
        self.needleCollisionModel = args[0].cosseratFrame.needleCollision

        self.contactListener = args[1]
        self.generic = args[2]
        self.entryPoint = []
        self.threshold = 3.0  # \lamba_1

        self.constraintPointsNode = args[3]
        self.pointManager = self.constraintPointsNode.pointsManager

        self.constraintPts = self.constraintPointsNode.constraintPointsMo
        self.constraintPtsContainer = self.constraintPointsNode.constraintPtsContainer
        self.constraintPtsModifier = self.constraintPointsNode.constraintPtsModifier
        self.inside = False
        self.addNewPoint = False
        self.rootNode = args[4]

        self.rate = 0.2
        self.angularRate = 0.02
        self.tipForce = [0.0, 0.0, 0]
        # Scene x of the needle tip when each constraint point was added, deepest last. Points
        # are added and removed by comparing the tip with these, not with the points
        # themselves: the points belong to the gel and move when it deforms (e.g. when the
        # needle is moved sideways), which used to add points in deformed gel mid-sweep.
        self.pointDepths = []
        return

    def tipX(self):
        """Scene x of the needle tip: how far it has gone along the insertion axis."""
        return float(self.needleSlidingState.position.array()[-1][0])

    def leaveGel(self):
        self.inside = False
        self.needleCollisionModel.findData("activated").value = True
        self.generic.computeConstraintForces.value = True
        self.tipForce = [0.0, 0.0, 0]
        self.pointDepths = []

    def onAnimateEndEvent(self, event):
        if self.contactListener.getContactPoints() and not self.inside:
            # @Info: check if the contact force is large enough to go through the tissue
            if (
                self.generic.constraintForces
                and self.generic.constraintForces[0] > self.threshold
            ):
                # 1. @Info: Save the entryPoint and contact force
                vec = self.contactListener.getContactPoints()[0][1]
                self.entryPoint = [vec[0], vec[1], vec[2]]
                self.tipForce = [
                    self.generic.constraintForces[0],
                    self.generic.constraintForces[1],
                    self.generic.constraintForces[2],
                ]

                # 2. @Info: deactivate the contact constraint
                self.needleCollisionModel.findData("activated").value = 0

                # @Info 3. Add entryPoint point as the first constraint point in FEM
                self.pointManager.addNewPointToState()
                self.inside = True
                self.pointDepths = [self.tipX()]
                print(f"[needle] punctured at x = {self.pointDepths[0]:.2f}", flush=True)

            elif self.tipForce[0] > self.threshold:
                pass
                print(
                    "Please activate computeConstraintForces data field inside the GenericConstraint component"
                )

        else:
            if self.inside:
                # @info 1. check if the tip went far enough along the needle axis to add or
                # remove a constraint point
                points = self.constraintPts.position.array()
                if len(points) != len(self.pointDepths):
                    # Points added or removed elsewhere (M/D keys here, S/L in PointsManager)
                    self.pointDepths = [float(p[0]) for p in points]
                if not self.pointDepths:
                    self.leaveGel()
                    return
                tip, last = self.tipX(), self.pointDepths[-1]
                step = ConstraintsParams.constraintDistance

                # Add constraint point when going forwards
                if tip - last > step:
                    self.pointManager.addNewPointToState()
                    self.pointDepths.append(tip)
                    print(f"[needle] gel point {len(self.pointDepths)} added at x = {tip:.2f}",
                          flush=True)

                # Only the entry point left: out of the gel once the tip is back above it
                elif len(self.pointDepths) == 1:
                    if tip < points[0][0]:
                        self.pointManager.removeLastPointfromState()
                        self.leaveGel()
                        print("[needle] out of the gel", flush=True)

                # Remove previous constraint point when going backwards
                elif last - tip > step:
                    self.pointManager.removeLastPointfromState()
                    self.pointDepths.pop()
                    print(f"[needle] gel point {len(self.pointDepths) + 1} removed at x = {tip:.2f}",
                          flush=True)

    def onKeypressedEvent(self, event):
        key = event["key"]
        if key == "M":
            self.pointManager.addNewPointToState()
        if key == "D":
            self.pointManager.removeLastPointfromState()
        if key == "B":
            self.rootNode.findData("animate").value = 1

        if ord(key) == 18:  # left
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] -= self.rate
        elif ord(key) == 20:  # right
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][0] += self.rate

        elif ord(key) == 21:  # down
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][1] -= self.rate

        elif ord(key) == 19:  # up
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][1] += self.rate

        if key == "I":  #
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][2] -= self.rate
        elif key == "K":  #
            with self.rigidBaseMO.rest_position.writeable() as posA:
                posA[0][2] += self.rate

    # def onAnimateBeginEvent(self, event):
    #     if self.inside & self.addNewPoint:
    #         print("Inside the volume")
    #         self.addConstraintPointToFem(self.entryPoint)
    #
    #         # 4. todo: add new constraint points inside the volume if needed.
    #         # todo: This depends on the choice of the algorithm
    #         #  expl1: one can compare tip position related to the last constraint position inside the volume and
    #         #  when this > than the constraintDistance we add new constraint point
    #         # self.addPointToFem()
    #         # expl2: one can compare the tip position related to the entry point and when
    #         # this > than the constraintDistance
    #
    #