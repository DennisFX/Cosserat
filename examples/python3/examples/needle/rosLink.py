# -*- coding: utf-8 -*-
"""
UDP link between a SOFA scene and the ros2sofa_bridge ROS 2 node.

The scene never imports rclpy (the two runtimes crash when loaded into one
process). Everything goes over two loopback UDP ports, one JSON object per
datagram, each with a "type" field. Everything on the wire is SI (m, N, N.m,
rad): convert scene units on this side before sending.

    ROS -> SOFA  (port 9871)
        {"type": "joint_error", "stamp": 12.3, "names": [...], "positions": [...]}
    SOFA -> ROS  (port 9872)
        {"type": "wrench", "force": [fx, fy, fz], "torque": [tx, ty, tz]}
        {"type": "clear"}

Usage, in createScene():

    from rosLink import RosLink
    rosLink = rootNode.addObject(RosLink(name="rosLink"))

Extending it without touching this file:

    rosLink.subscribe("some_state", callback)                    # newest one per step
    rosLink.subscribe("some_event", callback, everyMessage=True)  # every single one
    rosLink.bindKey("X", callback)                                # runSofa: Ctrl + X
    rosLink.send("some_type", field=value)
"""
import json
import socket
import time

import Sofa

HOST = '127.0.0.1'
ROS_TO_SOFA_PORT = 9871
SOFA_TO_ROS_PORT = 9872

# Sockets bound by this module, by port. If runSofa reloads the scene in the same
# process, the previous controller's socket is closed so the port can be re-bound.
_boundSockets = {}


class UdpLink:
    """Non-blocking JSON-over-UDP endpoint. No SOFA or ROS dependency."""

    def __init__(self, listenPort=ROS_TO_SOFA_PORT, sendPort=SOFA_TO_ROS_PORT, host=HOST):
        previous = _boundSockets.pop(listenPort, None)
        if previous is not None:
            previous.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, listenPort))
        self.sock.setblocking(False)
        _boundSockets[listenPort] = self.sock
        self.peer = (host, sendPort)

    def send(self, msgType, **fields):
        fields['type'] = msgType
        self.sock.sendto(json.dumps(fields).encode(), self.peer)

    def receiveAll(self):
        """Drain the socket and return every valid message, oldest first."""
        messages = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return messages
            try:
                msg = json.loads(data)
            except ValueError as exc:
                print(f'[rosLink] dropped malformed datagram: {exc}', flush=True)
                continue
            if isinstance(msg, dict) and 'type' in msg:
                messages.append(msg)
            else:
                print('[rosLink] dropped datagram without a "type" field', flush=True)


class RosLink(Sofa.Core.Controller):
    """
    Keys (runSofa only forwards keys to the scene while Ctrl is held):
        W   apply the test wrench to the Gazebo needle (stays on until cleared)
        R   clear it

    Incoming messages are read at the start of each step, so nothing arrives
    while the scene is paused. Key presses are sent immediately, paused or not.
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.link = UdpLink(kwargs.get('listenPort', ROS_TO_SOFA_PORT),
                            kwargs.get('sendPort', SOFA_TO_ROS_PORT))

        # W-key test wrench: SI units, Gazebo world frame, applied by ApplyLinkWrench
        self.testForce = kwargs.get('testForce', [20, 0.0, 0.0])
        self.testTorque = kwargs.get('testTorque', [0.0, 0.0, 0.0])
        self.logPeriod = kwargs.get('logPeriod', 1.0)  # s between joint-error prints, 0 = silent

        self.latest = {}  # newest message received of each type, for other components to read
        self._stateHandlers = {}
        self._eventHandlers = {}
        self._keys = {}
        self._unhandledTypes = set()
        self._lastLog = 0.0

        self.subscribe('joint_error', self._logJointError)
        self.bindKey('W', self.applyTestWrench)
        self.bindKey('R', self.clearWrench)

    # ---- extension points -------------------------------------------------
    def subscribe(self, msgType, callback, everyMessage=False):
        """State (default): callback gets only the newest message per step.
        Events (everyMessage=True): callback gets every message, in order."""
        (self._eventHandlers if everyMessage else self._stateHandlers)[msgType] = callback

    def bindKey(self, key, callback):
        self._keys[key.upper()] = callback

    def send(self, msgType, **fields):
        self.link.send(msgType, **fields)

    # ---- outgoing -----------------------------------------------------------
    def sendWrench(self, force, torque=(0.0, 0.0, 0.0)):
        """Persistent wrench on the Gazebo needle link: SI units, Gazebo world frame."""
        self.send('wrench', force=[float(f) for f in force], torque=[float(t) for t in torque])

    def applyTestWrench(self):
        self.sendWrench(self.testForce, self.testTorque)
        print(f'[rosLink] -> wrench F={self.testForce} T={self.testTorque}', flush=True)

    def clearWrench(self):
        self.send('clear')
        print('[rosLink] -> clear', flush=True)

    # ---- incoming -----------------------------------------------------------
    def onAnimateBeginEvent(self, event):
        newest = {}
        for msg in self.link.receiveAll():
            msgType = msg['type']
            self.latest[msgType] = msg
            if msgType in self._eventHandlers:
                self._eventHandlers[msgType](msg)
            elif msgType in self._stateHandlers:
                newest[msgType] = msg
            elif msgType not in self._unhandledTypes:
                self._unhandledTypes.add(msgType)
                print(f'[rosLink] no handler for "{msgType}" messages, ignoring them', flush=True)
        for msgType, msg in newest.items():
            self._stateHandlers[msgType](msg)

    def onKeypressedEvent(self, event):
        callback = self._keys.get(event['key'].upper())
        if callback is not None:
            callback()

    def _logJointError(self, msg):
        now = time.monotonic()
        if not self.logPeriod or now - self._lastLog < self.logPeriod:
            return
        self._lastLog = now
        errors = ', '.join(f'{n}={e:+.2e}' for n, e in zip(msg['names'], msg['positions']))
        print(f'[rosLink] joint error (rad): {errors}', flush=True)
