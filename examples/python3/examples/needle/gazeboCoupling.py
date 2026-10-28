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
  - each step, the needle base rest pose moves toward the Gazebo pose, by at most
    maxStep (scene length units) and maxAngleStep (rad), so a sudden jump can't hit
    the stiff base spring all at once. This overrides the arrow keys.
  - after each step, the base wrench goes to Gazebo: minus the monitor's baseForce and
    baseTorque (what the needle does to its holder), in SI units, times wrenchGain,
    expressed in the sofa_scene frame (the bridge rotates it into the world frame).
  - if that wrench exceeds maxForce or maxTorque (or is not finite), it is not sent and
    following stops: the scene has almost certainly gone unstable, and passing the
    wrench on would throw the arm around, which the needle would then follow.
Turning it off sends a zero wrench and leaves the needle where it is.

Lockstep (lockstep=True, the default): SOFA and Gazebo run on the same clock, whatever
either machine's speed. Each pose from ROS carries Gazebo's sim time ("stamp"). On
engaging, SOFA's step 0 starts at the newest pose's time, and step k covers Gazebo time
tau_k = tau_0 + k dt to tau_k + dt:
  - before step k, SOFA waits (in wall time) until it has a Gazebo pose from about tau_k
    (stamp >= tau_k - dt/2), and drives the needle base toward that pose. A fast SOFA
    thus waits for Gazebo. If poses come less often than every dt of Gazebo time, the
    window widens to the newest one SOFA can expect, so SOFA never waits for a pose
    Gazebo isn't allowed to reach; SOFA then says so once, since it can only follow the
    needle that coarsely.
  - the step's wrench is stamped tau_k + dt. Gazebo's SetLinkWrench system (lockstep in
    needle.sdf) never runs more than a set lag past the newest stamp, so a slow SOFA
    slows Gazebo down.
  - if no such pose comes within maxWait s (Gazebo paused or gone), SOFA steps without
    it, holding the needle base and sending nothing, until Gazebo moves again.
  - if Gazebo's time is more than resyncAfter s away from tau_k (SOFA was paused long
    enough for Gazebo to stop waiting, or Gazebo restarted), SOFA jumps to Gazebo's time.
Poses without a stamp (an older ROS bridge) fall back to following the newest pose.

Usage, at the end of createScene(), after the WrenchMonitor:

    from gazeboCoupling import GazeboCoupling
    rootNode.addObject(GazeboCoupling(name="gazeboCoupling", rosLink=rosLink,
                                      monitor=wrenchMonitor, needle=needle))

Scene units default to cm, kg, s (metersPerUnit=0.01, kgPerUnit=1.0).
"""
import collections
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
        # Safety cutoff on the wrench sent to Gazebo (after wrenchGain). The UR5's wrist joints
        # give way somewhere above ~100 N at the needle base.
        self.maxForce = kwargs.get('maxForce', 50.0)             # N
        self.maxTorque = kwargs.get('maxTorque', 10.0)           # N.m
        self.frame = kwargs.get('frame', 'sofa_scene')           # TF frame of the SOFA scene
        self.staleAfter = kwargs.get('staleAfter', 1.0)          # s without a pose before warning
        # Lockstep with Gazebo's clock (see the top of this file)
        self.lockstep = kwargs.get('lockstep', True)
        self.maxWait = kwargs.get('maxWait', 0.2)                # s of wall time per step
        self.resyncAfter = kwargs.get('resyncAfter', 0.1)        # s of Gazebo time

        self.following = False
        self.target = None          # newest Gazebo pose, scene units, [x, y, z, qx, qy, qz, qw]
        self._lastPose = None       # time.monotonic() of the newest pose
        self._stale = False
        # Recent stamped poses, oldest first: (Gazebo time [s], pose as in target)
        self._poses = collections.deque(maxlen=200)
        self._anchor = None         # Gazebo time of lockstep step 0 [s]
        self._steps = 0             # lockstep steps since then
        self._stamp = None          # Gazebo time this step's wrench is for, None if none
        self._stalled = False
        self._warnedCoarse = False

        self.key = kwargs.get('key', 'G').upper()
        # Every pose, not just the newest per step: lockstep picks the one for the step's time
        self.rosLink.subscribe('base_pose', self._onBasePose, everyMessage=True)
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
        stamp = msg.get('stamp')
        if isinstance(stamp, (int, float)) and np.isfinite(stamp) and stamp > 0.0:
            self._poses.append((float(stamp), self.target))
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
        self._anchor, self._steps, self._stamp, self._stalled = None, 0, None, False
        what = 'following the Gazebo needle'
        if self.sendWrenches:
            what += ' and sending it the needle wrench'
        if self._inLockstep():
            what += ', in lockstep with Gazebo\'s clock'
        print(f'[coupling] {what} (Ctrl+{self.key} to stop)', flush=True)

    def stop(self):
        self.following = False
        self._stamp = None
        if self.sendWrenches:
            self.rosLink.send('clear')  # no stamp: also releases Gazebo from the lockstep
        print('[coupling] stopped following; the needle stays where it is', flush=True)

    # ---- each step ----------------------------------------------------------
    def onAnimateBeginEvent(self, event):
        self._stamp = None
        if not self.following or self.target is None:
            return
        if self._inLockstep():
            target, self._stamp = self._lockstepTarget(float(event['dt']))
            if target is None:
                return  # Gazebo isn't moving: hold the needle base
        else:
            target = self.target
            if (not self._stale and self._lastPose is not None
                    and time.monotonic() - self._lastPose > self.staleAfter):
                self._stale = True
                print(f'[coupling] no needle pose from ROS for {self.staleAfter:g} s, '
                      f'holding the last one', flush=True)
        with self.baseMO.rest_position.writeable() as rest:
            new = self._stepToward(np.asarray(rest[0], dtype=float), target)
            for i in range(7):
                rest[0][i] = new[i]

    def onAnimateEndEvent(self, event):
        # Added after the WrenchMonitor in the scene, so this is this step's wrench
        if not self.following:
            return
        stamp, self._stamp = self._stamp, None
        if stamp is not None:
            self._steps += 1
        elif self._inLockstep():
            return  # a step without Gazebo (paused or gone): nothing to send
        if not self.sendWrenches:
            return
        gain = self.wrenchGain
        force = -np.asarray(self.monitor.baseForce, dtype=float) * self.forceToSI * gain
        torque = -np.asarray(self.monitor.baseTorque, dtype=float) * self.torqueToSI * gain
        f, t = np.linalg.norm(force), np.linalg.norm(torque)
        if not (np.isfinite(f) and np.isfinite(t)) or f > self.maxForce or t > self.maxTorque:
            print(f'[coupling] base wrench |F| = {f:.3g} N, |T| = {t:.3g} N.m is over the limit '
                  f'({self.maxForce:g} N, {self.maxTorque:g} N.m), not sending it. The scene has '
                  f'probably gone unstable: reload it before coupling again.', flush=True)
            self.stop()
            return
        fields = dict(frame=self.frame, force=force.tolist(), torque=torque.tolist())
        if stamp is not None:
            fields['stamp'] = stamp
        self.rosLink.send('wrench', **fields)

    # ---- lockstep -----------------------------------------------------------
    def _inLockstep(self):
        return self.lockstep and bool(self._poses)

    def _lockstepTarget(self, dt):
        """
        Wait for the Gazebo pose at the start of this step and return it, with the Gazebo
        time this step's wrench is for. (None, None) if Gazebo isn't advancing.
        """
        deadline = time.monotonic() + self.maxWait
        while True:
            newest = self._poses[-1][0]
            if self._anchor is None:
                self._anchor, self._steps = newest, 0
            tau = self._anchor + self._steps * dt
            if abs(newest - tau) > self.resyncAfter:
                print(f'[coupling] Gazebo\'s clock is {newest - tau:+.2f} s from SOFA\'s; '
                      f'continuing from Gazebo\'s', flush=True)
                self._anchor, self._steps, tau = newest, 0, newest
                # Drop poses from before a jump back in time (Gazebo restarted)
                self._poses = collections.deque(
                    [(t, p) for t, p in self._poses if t <= newest], maxlen=self._poses.maxlen)
            window = self._window(dt)
            if newest >= tau - window:
                if self._stalled:
                    self._stalled = False
                    print('[coupling] Gazebo is moving again', flush=True)
                pose = next(p for t, p in self._poses if t >= tau - window)
                return pose, tau + dt
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if not self._stalled:
                    self._stalled = True
                    print(f'[coupling] Gazebo\'s clock has not reached SOFA\'s for '
                          f'{self.maxWait:g} s (paused?); holding the needle until it does',
                          flush=True)
                return None, None
            if self.rosLink.waitForMessages(remaining):
                self.rosLink.poll()

    def _window(self, dt):
        """
        How far before the step's start a pose may be [s]: dt/2, or, if Gazebo sends poses
        less often than every dt of its time, one pose interval less dt/2. Waiting for a pose
        any closer would wait for one Gazebo can't reach while it waits for this step.
        """
        stamps = [t for t, _ in list(self._poses)[-21:]]
        gaps = sorted(b - a for a, b in zip(stamps[:-1], stamps[1:]) if b > a)
        if len(gaps) < 3:
            return 0.5 * dt
        interval = gaps[len(gaps) // 2]
        if interval > 1.5 * dt and not self._warnedCoarse:
            self._warnedCoarse = True
            print(f'[coupling] the Gazebo needle pose comes every {1000 * interval:.0f} ms of '
                  f'Gazebo time, so SOFA follows it that coarsely. robot_state_publisher\'s '
                  f'publish_frequency (ur5_gazebo.launch.py) sets it', flush=True)
        return max(0.5 * dt, interval - 0.5 * dt)

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