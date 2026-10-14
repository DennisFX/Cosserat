# -*- coding: utf-8 -*-
"""
Couples the SOFA needle to the Gazebo needle, through rosLink.py and the ros2sofa_bridge node.

    Gazebo --base_pose--> SOFA:   the needle base follows the Gazebo needle base
    SOFA   --wrench-----> Gazebo: the wrench the SOFA needle puts on its holder

Ctrl+G turns following on and off (runSofa only passes keys on while Ctrl is held).

Following only starts when the Gazebo needle is already where the SOFA needle is
(within engageDistance and engageAngle), i.e. with the arm at the pre-insertion pose.
Otherwise the arm's travel to that pose would drag the SOFA needle through the gel.

While following:
  - each step, the needle base rest pose moves toward the newest Gazebo pose, by at
    most maxStep (scene length units) and maxAngleStep (rad), so a sudden jump can't
    hit the stiff base spring all at once. This overrides the arrow keys.
  - after each step, the base wrench goes to Gazebo: minus the monitor's baseForce and
    baseTorque (what the needle does to its holder), in SI units, times wrenchGain,
    expressed in the sofa_scene frame (the bridge rotates it into the world frame).
Turning it off sends a zero wrench and leaves the needle where it is.

Usage, at the end of createScene(), after the WrenchMonitor:

    from gazeboCoupling import GazeboCoupling
    rootNode.addObject(GazeboCoupling(name="gazeboCoupling", rosLink=rosLink,
                                      monitor=wrenchMonitor, needle=needle))

Scene units default to cm, kg, s (metersPerUnit=0.01, kgPerUnit=1.0).
"""
import time

import numpy as np
import Sofa

from wrenchMonitor import quatInverse, quatMultiply, quatToRotvec


def rotvecToQuat(r):
    r = np.asarray(r, dtype=float)
    angle = np.linalg.norm(r)
    if angle < 1e-12:
        q = np.r_[0.5 * r, 1.0]
        return q / np.linalg.norm(q)
    return np.r_[r / angle * np.sin(0.5 * angle), np.cos(0.5 * angle)]


class GazeboCoupling(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rosLink = kwargs['rosLink']
        self.monitor = kwargs['monitor']
        self.baseMO = kwargs['needle'].rigidBaseNode.RigidBaseMO

        metersPerUnit = kwargs.get('metersPerUnit', 0.01)
        kgPerUnit = kwargs.get('kgPerUnit', 1.0)
        self.unitsPerMeter = 1.0 / metersPerUnit
        self.forceToSI = kgPerUnit * metersPerUnit                    # scene force -> N
        self.torqueToSI = kgPerUnit * metersPerUnit * metersPerUnit   # scene torque -> N.m

        self.maxStep = kwargs.get('maxStep', 0.5)                # scene length units per step
        self.maxAngleStep = kwargs.get('maxAngleStep', 0.05)     # rad per step
        self.engageDistance = kwargs.get('engageDistance', 1.0)  # scene length units
        self.engageAngle = kwargs.get('engageAngle', 0.1)        # rad
        self.wrenchGain = kwargs.get('wrenchGain', 1.0)          # >1 exaggerates it in Gazebo
        self.sendWrenches = kwargs.get('sendWrenches', True)
        self.frame = kwargs.get('frame', 'sofa_scene')           # TF frame of the SOFA scene
        self.staleAfter = kwargs.get('staleAfter', 1.0)          # s without a pose before warning

        self.following = False
        self.target = None          # newest Gazebo pose, scene units, [x, y, z, qx, qy, qz, qw]
        self._lastPose = None       # time.monotonic() of the newest pose
        self._stale = False

        self.key = kwargs.get('key', 'G').upper()
        self.rosLink.subscribe('base_pose', self._onBasePose)
        self.rosLink.bindKey(self.key, self.toggle)

    # ---- incoming pose ------------------------------------------------------
    def _onBasePose(self, msg):
        try:
            p = np.asarray(msg['p'], dtype=float)
            q = np.asarray(msg['q'], dtype=float)
            if p.shape != (3,) or q.shape != (4,):
                raise ValueError('needs p with 3 values and q with 4')
            if not np.all(np.isfinite(np.r_[p, q])) or np.linalg.norm(q) < 1e-9:
                raise ValueError('not a valid pose')
        except (KeyError, TypeError, ValueError) as exc:
            print(f'[coupling] ignored a bad base_pose ({exc})', flush=True)
            return
        self.target = np.r_[p * self.unitsPerMeter, q / np.linalg.norm(q)]
        self._lastPose = time.monotonic()
        if self._stale:
            self._stale = False
            print('[coupling] receiving the Gazebo needle pose again', flush=True)

    # ---- on / off -----------------------------------------------------------
    def toggle(self):
        if self.following:
            self.stop()
            return
        if self.target is None:
            print('[coupling] no needle pose from ROS yet. Is sofa_bridge running, with the '
                  'Gazebo launch file? (Poses only arrive while the scene is animating.)',
                  flush=True)
            return
        rest = np.asarray(self.baseMO.rest_position.value[0], dtype=float)
        distance, angle = self._offset(rest, self.target)
        if distance > self.engageDistance or angle > self.engageAngle:
            print(f'[coupling] not following: the Gazebo needle is '
                  f'{distance / self.unitsPerMeter * 1000:.1f} mm and {np.degrees(angle):.1f} deg '
                  f'away from the SOFA needle. Move the arm to the pre-insertion pose first.',
                  flush=True)
            return
        self.following = True
        what = 'following the Gazebo needle'
        if self.sendWrenches:
            what += ' and sending it the needle wrench'
        print(f'[coupling] {what} (Ctrl+{self.key} to stop)', flush=True)

    def stop(self):
        self.following = False
        if self.sendWrenches:
            self.rosLink.send('clear')
        print('[coupling] stopped following; the needle stays where it is', flush=True)

    # ---- each step ----------------------------------------------------------
    def onAnimateBeginEvent(self, event):
        if not self.following or self.target is None:
            return
        if (not self._stale and self._lastPose is not None
                and time.monotonic() - self._lastPose > self.staleAfter):
            self._stale = True
            print(f'[coupling] no needle pose from ROS for {self.staleAfter:g} s, '
                  f'holding the last one', flush=True)
        with self.baseMO.rest_position.writeable() as rest:
            new = self._stepToward(np.asarray(rest[0], dtype=float), self.target)
            for i in range(7):
                rest[0][i] = new[i]

    def onAnimateEndEvent(self, event):
        # Added after the WrenchMonitor in the scene, so this is this step's wrench
        if not (self.following and self.sendWrenches):
            return
        gain = self.wrenchGain
        force = -np.asarray(self.monitor.baseForce, dtype=float) * self.forceToSI * gain
        torque = -np.asarray(self.monitor.baseTorque, dtype=float) * self.torqueToSI * gain
        self.rosLink.send('wrench', frame=self.frame,
                          force=force.tolist(), torque=torque.tolist())

    # ---- helpers ------------------------------------------------------------
    @staticmethod
    def _offset(a, b):
        """Distance and rotation angle between two poses [x, y, z, qx, qy, qz, qw]."""
        distance = np.linalg.norm(b[:3] - a[:3])
        angle = np.linalg.norm(quatToRotvec(quatMultiply(b[3:], quatInverse(a[3:]))))
        return distance, angle

    def _stepToward(self, current, target):
        p, q = current[:3], current[3:] / np.linalg.norm(current[3:])
        tp, tq = target[:3], target[3:]
        if np.dot(tq, q) < 0.0:
            tq = -tq                     # same rotation, same sign as the current one

        dp = tp - p
        distance = np.linalg.norm(dp)
        if distance > self.maxStep:
            dp *= self.maxStep / distance

        dr = quatToRotvec(quatMultiply(tq, quatInverse(q)))   # rotation from q to tq
        angle = np.linalg.norm(dr)
        if angle > self.maxAngleStep:
            dr *= self.maxAngleStep / angle
        newQ = quatMultiply(rotvecToQuat(dr), q)
        return np.r_[p + dp, newQ / np.linalg.norm(newQ)]
