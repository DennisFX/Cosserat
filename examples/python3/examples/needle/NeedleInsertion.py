# -*- coding: utf-8 -*-
"""Basic scene using Cosserat in SofaPython3.

Based on the work done with SofaPython. See POEMapping.py
"""

from cosserat.needle.needleController import Animation
from cosserat.needle.params import NeedleParameters, GeometryParams, PhysicsParams, FemParams, ContactParams, \
    ConstraintsParams
from cosserat.usefulFunctions import pluginList
from cosserat.createFemRegularGrid import createFemCubeWithParams, layerBoundary
from cosserat.cosseratObject import Cosserat
from useful.utils import addConstraintPoint
from rosLink import RosLink
from wrenchMonitor import WrenchMonitor
from gazeboCoupling import GazeboCoupling
from shaftFriction import ShaftFriction
import os
import sys

# params = NeedleParameters()

sys.path.append('../')

nbFrames = GeometryParams.nbFrames
needleGeometryConfig = {'init_pos': [0., 0., 0.], 'tot_length': GeometryParams.totalLength,
                        'nbSectionS': GeometryParams.nbSections, 'nbFramesF': nbFrames,
                        'buildCollisionModel': 1, 'beamMass': PhysicsParams.mass}

LAYER_ORDERS = {'soft-on-top': False, 'hard-on-top': True}


def layersFromEnvironment():
    """Set FemParams.hardOnTop from NEEDLE_LAYERS (soft-on-top or hard-on-top), if it is set.

    export NEEDLE_LAYERS=hard-on-top before starting runSofa builds the gel with its hard layer
    on top, without editing params.py. Exported, not put in front of the command: an alias
    that starts with cd would pass it to cd only.
    """
    value = os.environ.get('NEEDLE_LAYERS', '').strip().lower()
    if value:
        if value not in LAYER_ORDERS:
            raise ValueError(f'NEEDLE_LAYERS={value}: use soft-on-top or hard-on-top')
        FemParams.hardOnTop = LAYER_ORDERS[value]


def layerFriction():
    """Friction per cm of shaft (scene units) in the upper and in the lower gel layer.

    Each layer gets its material's: frictionPerLength in the soft one, hardFrictionPerLength in
    the hard one, wherever they are. A uniform gel gets frictionPerLength throughout.
    """
    soft, hard = ConstraintsParams.frictionPerLength, ConstraintsParams.hardFrictionPerLength
    if layerBoundary(FemParams) is None:
        return soft, soft
    return (hard, soft) if FemParams.hardOnTop else (soft, hard)


def describeScene(needle):
    """What this scene built, for the console and for ROS's recordings: gel and needle."""
    upper, lower = layerFriction()
    boundary = layerBoundary(FemParams)
    soft, hard = FemParams.youngModulus / 10.0, FemParams.hardYoungModulus / 10.0  # kPa
    if boundary is None:
        gel = f'uniform, {soft:g} kPa; shaft friction {upper / 100.0:g} N/cm'
    else:
        depth = boundary - float(FemParams.minVol.split()[0])
        names, moduli = ('soft', 'hard'), (soft, hard)
        if FemParams.hardOnTop:
            names, moduli = names[::-1], moduli[::-1]
        gel = (f'{names[0]} on top: {moduli[0]:g} kPa down to {depth:g} cm, {moduli[1]:g} kPa '
               f'below; shaft friction {upper / 100.0:g} then {lower / 100.0:g} N/cm')
    try:  # rigid if the FixedProjectiveConstraint line below is in
        rigid = any(obj.getClassName() == 'FixedProjectiveConstraint'
                    for obj in needle.cosseratCoordinateNode.objects)
        kind = 'rigid' if rigid else 'flexible'
    except AttributeError:
        kind = 'unknown'
    return {'gel': gel, 'needle': kind}


def createScene(rootNode):
    layersFromEnvironment()

    rootNode.addObject('RequiredPlugin', name='plugins', pluginName=[pluginList])
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')  # FixedProjectiveConstraint

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels '
                                                   'hideBoundingCollisionModels hideForceFields '
                                                   'hideInteractionForceFields hideWireframe showMechanicalMappings')
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject("DefaultVisualManagerLoop")
    rootNode.addObject('RuleBasedContactManager',
                       responseParams='mu=0.1', response='FrictionContactConstraint')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    # rootNode.addObject('LocalMinDistance', alarmDistance=1.0, contactDistance=0.01)
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance=0.5,
                       contactDistance=ContactParams.contactDistance,
                       coneFactor=ContactParams.coneFactor, angleCone=0.1)

    rootNode.addObject('FreeMotionAnimationLoop')
    generic = rootNode.addObject('BlockGaussSeidelConstraintSolver', tolerance="1e-20",
                                 maxIterations="500", computeConstraintForces=1, printLog="0")

    gravity = [0, 0, 0]
    rootNode.gravity.value = gravity
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    # rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    # ###############
    # New adds to use the sliding Actuator
    ###############
    solverNode = rootNode.addChild('solverNode')
    solverNode.addObject('EulerImplicitSolver',
                         rayleighStiffness=PhysicsParams.rayleighStiffness)
    solverNode.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")
    solverNode.addObject('GenericConstraintCorrection')

    needle = Cosserat(parent=solverNode, cosseratGeometry=needleGeometryConfig, radius=GeometryParams.radius,
        name="needle", youngModulus=PhysicsParams.youngModulus, poissonRatio=PhysicsParams.poissonRatio,
        rayleighStiffness=PhysicsParams.rayleighStiffness)
    needle.cosseratCoordinateNode.addObject('FixedProjectiveConstraint', fixAll=True) # TODO comment out if we do not want a rigid needle

    # Wrench monitor sanity check: 1 N sideways at the tip, 15 cm from the base. Once settled,
    # expect base F ~ (0, -100, 0) and T ~ (0, 0, -1500) in scene units. Uncomment to run it.
    # tip = len(needle.cosseratFrame.FramesMO.position.value) - 1
    # needle.cosseratFrame.addObject('ConstantForceField', indices=[tip], forces=[[0, 100, 0, 0, 0, 0]])

    needleCollisionModel = needle.addPointCollisionModel("needleCollision")

    # These state is mapped on the needle and used to compute the distance between the needle and the
    # FEM constraint points
    slidingPoint = needle.addSlidingPoints()

    # -----------------
    # Start the volume definition
    # -----------------
    cubeNode = createFemCubeWithParams(rootNode, FemParams)
    gelNode = cubeNode.getChild('gelNode')
    # FEM constraint points
    constraintPointNode = addConstraintPoint(
        gelNode, slidingPoint.slidingPointMO.getLinkPath()[1:])

    # @info : This is the constraint point that will be used to compute the distance between the needle and the volume
    conttactL = rootNode.addObject('ContactListener', name="contactListener",
                                   collisionModel1=cubeNode.gelNode.surfaceNode.surface.getLinkPath(),
                                   collisionModel2=needleCollisionModel.collisionStats.getLinkPath())

    # These stats will represents the distance between the contraint point in the volume and
    # their projection on the needle
    # It 's also important to say that the x direction is not taken into account
    distanceStatsNode = slidingPoint.addChild('distanceStatsNode')
    constraintPointNode.addChild(distanceStatsNode)
    constraintPoinMo = distanceStatsNode.addObject('MechanicalObject', name="distanceStats", template="Vec3d",
                                                   position=[], listening="1", showObject="1", showObjectScale="0.1")
    inputVolumeMo = constraintPointNode.constraintPointsMo.getLinkPath()
    inputNeedleMo = slidingPoint.slidingPointMO.getLinkPath()
    outputDistanceMo = distanceStatsNode.distanceStats.getLinkPath()

    # ---------------------------------------------------
    # @info: Start controller node
    rosLink = rootNode.addObject(RosLink(name="rosLink"))  # Ctrl+W / Ctrl+R, see rosLink.py

    rootNode.addObject(Animation(needle, conttactL, generic,
                       constraintPointNode, rootNode, constraintPoinMo))


    distanceStatsNode.addObject(
        'CosseratNeedleSlidingConstraint', name="computeDistanceComponent",
        frictionCoefficient=ConstraintsParams.frictionCoefficient)
    distanceStatsNode.addObject('DifferenceMultiMapping', name="pointsMulti", input1=inputVolumeMo, lastPointIsFixed=0,
                                input2=inputNeedleMo, output=outputDistanceMo, direction="@../../FramesMO.position")

    # Shaft friction: sets the constraint's friction at each gel point before every step, from
    # a friction per cm of shaft in each gel layer, each layer its material's (layerFriction).
    # ConstraintsParams.frictionPerLength and hardFrictionPerLength = 0 turn it off. See
    # shaftFriction.py.
    upperFriction, lowerFriction = layerFriction()
    rootNode.addObject(ShaftFriction(name="shaftFriction",
                                     constraint=distanceStatsNode.computeDistanceComponent,
                                     needlePoints=slidingPoint.slidingPointMO,
                                     gelPoints=constraintPointNode.constraintPointsMo,
                                     perLength=upperFriction, lowerPerLength=lowerFriction,
                                     boundary=layerBoundary(FemParams)))

    # Must stay last: it looks up computeDistanceComponent by name, created just above
    wrenchMonitor = rootNode.addObject(WrenchMonitor(
        name="wrenchMonitor", rootNode=rootNode, needle=needle, solver=generic,
        slidingConstraint=distanceStatsNode.computeDistanceComponent,
        distanceMO=constraintPoinMo,
        youngModulus=PhysicsParams.youngModulus,
        poissonRatio=PhysicsParams.poissonRatio,
        radius=GeometryParams.radius,
        rayleighStiffness=PhysicsParams.rayleighStiffness,  # same value as the EulerImplicitSolver
        printEvery=20))

    # Ctrl+G: follow the Gazebo needle and send it the base wrench. After the monitor,
    # so the wrench it sends is the one computed this step. See gazeboCoupling.py. It also
    # tells ROS what this scene is (gel and needle), which ROS writes into its recordings.
    scene = describeScene(needle)
    gel, kind = scene['gel'], scene['needle']
    print(f'[gel] {gel}; {kind} needle', flush=True)
    rootNode.addObject(GazeboCoupling(name="gazeboCoupling", rosLink=rosLink,
                                      monitor=wrenchMonitor, needle=needle, sceneInfo=scene,
                                      maxForce=float('50'), maxTorque=float('50')))  # no cutoff
