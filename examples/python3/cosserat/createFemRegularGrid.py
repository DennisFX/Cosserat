# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
"""Basic scene using Cosserat in SofaPython3.

Based on the work done with SofaPython. See POEMapping.py
"""

__authors__ = "younesssss"
__contact__ = "adagolodjo@protonmail.com, yinoussa.adagolodjo@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2021,Inria"
__date__ = "March 16 2021"

import math


def createFemCube(parentNode):
    FemNode = parentNode.addChild("FemNode")
    FemNode.addObject('VisualStyle', displayFlags='showBehaviorModels hideCollisionModels hideBoundingCollisionModels '
                                                  'showForceFields hideInteractionForceFields showWireframe')
    gelVolume = FemNode.addChild("gelVolume")
    gelVolume.addObject("RegularGridTopology", name="HexaTop", n="6 6 6", min="40 -16 -10", max="100 20 10")
    gelVolume.addObject("TetrahedronSetTopologyContainer", name="Container", position="@HexaTop.position")
    gelVolume.addObject("TetrahedronSetTopologyModifier", name="Modifier")
    gelVolume.addObject("Hexa2TetraTopologicalMapping", input="@HexaTop", output="@Container", swapping="false")

    GelSurface = FemNode.addChild("GelSurface")
    GelSurface.addObject("TriangleSetTopologyContainer", name="Container", position="@../GelVolume/HexaTop.position")

    gelNode = FemNode.addChild("gelNode")
    gelNode.addObject("EulerImplicitSolver", rayleighMass="0.1", rayleighStiffness="0.1")
    gelNode.addObject('SparseLDLSolver', name='preconditioner', template="CompressedRowSparseMatrixMat3x3d")
    gelNode.addObject('TetrahedronSetTopologyContainer', src="@../gelVolume/Container", name='container')
    gelNode.addObject('MechanicalObject', name='tetras', template='Vec3d')
    gelNode.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large',
                      poissonRatio='0.45', youngModulus='100')
    gelNode.addObject('BoxROI', name='ROI1', box='40 -17 -10 100 -14 10', drawBoxes='true')
    gelNode.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

    surfaceNode = gelNode.addChild("surfaceNode")
    surfaceNode.addObject('TriangleSetTopologyContainer', name="surfContainer", src="@../../GelSurface/Container")
    surfaceNode.addObject('MechanicalObject', name='msSurface')
    surfaceNode.addObject('TriangleCollisionModel', name='surface')
    surfaceNode.addObject('LineCollisionModel', name='line')
    surfaceNode.addObject('BarycentricMapping')

    gelNode.addObject('LinearSolverConstraintCorrection')

    return FemNode


def hardLayerBox(geometry):
    """BoxROI box of the hard layer (scene coordinates, as a string), or None for one gel.

    The layer runs from geometry.hardDepth below the top surface (x = minVol[0]) to the
    bottom. Elements count as in it when all their nodes are (BoxROI strict), so the layer
    starts on the first node plane at or below hardDepth.
    """
    hardModulus = getattr(geometry, 'hardYoungModulus', geometry.youngModulus)
    depth = getattr(geometry, 'hardDepth', 0.0)
    lo = [float(v) for v in geometry.minVol.split()]
    hi = [float(v) for v in geometry.maxVol.split()]
    if hardModulus == geometry.youngModulus or not 0.0 < depth < hi[0] - lo[0]:
        return None
    nx = int(geometry.mesh.split()[0])
    dx = (hi[0] - lo[0]) / (nx - 1)
    planes = depth / dx
    first = round(planes) if abs(planes - round(planes)) < 1e-6 else math.ceil(planes)
    if first >= nx - 1:
        return None
    if first != round(planes):
        print(f'[gel] no node plane {depth} cm deep (cells {dx:.3g} cm deep): the hard layer '
              f'starts at {first * dx:.3g} cm')
    margin = 1e-3 * dx
    box = [lo[0] + depth - margin, lo[1] - 1.0, lo[2] - 1.0, hi[0] + 1.0, hi[1] + 1.0, hi[2] + 1.0]
    return ' '.join(f'{v:.9g}' for v in box)


def createFemCubeWithParams(parentNode, geometry):
    FemNode = parentNode.addChild("FemNode")

    gelVolume = FemNode.addChild("gelVolume")
    gelVolume.addObject("RegularGridTopology", name="HexaTop", n=geometry.mesh, min=geometry.minVol,
                        max=geometry.maxVol)
    cont = gelVolume.addObject("TetrahedronSetTopologyContainer", name="TetraContainer", position="@HexaTop.position")
    gelVolume.addObject("TetrahedronSetTopologyModifier", name="Modifier")
    gelVolume.addObject("Hexa2TetraTopologicalMapping", input="@HexaTop", output="@TetraContainer", swapping="false")

    GelSurface = FemNode.addChild("GelSurface")
    GelSurface.addObject("TriangleSetTopologyContainer", name="triangleContainer",
                         position="@../gelVolume/HexaTop.position")
    GelSurface.addObject("TriangleSetTopologyModifier", name="Modifier")
    GelSurface.addObject("Tetra2TriangleTopologicalMapping", input="@../gelVolume/TetraContainer",
                         output="@triangleContainer", flipNormals="false")

    gelNode = FemNode.addChild("gelNode")
    gelNode.addObject("EulerImplicitSolver", rayleighMass=geometry.rayleigh, rayleighStiffness=geometry.rayleigh)
    gelNode.addObject('SparseLDLSolver', name='precond', template="CompressedRowSparseMatrixMat3x3d")
    gelNode.addObject('TetrahedronSetTopologyContainer', src="@../gelVolume/TetraContainer", name='container')
    gelNode.addObject('MechanicalObject', name='tetras', template='Vec3d')
    hardBox = hardLayerBox(geometry)
    youngModulus = geometry.youngModulus
    if hardBox is not None:
        # One modulus per tetrahedron: hardYoungModulus inside the hard layer (at rest),
        # youngModulus elsewhere. Before the FEM, so the moduli are there for its init.
        gelNode.addObject('BoxROI', name='hardLayer', box=hardBox, strict=True, doUpdate=False,
                          computeEdges=False, computeTriangles=False, computeQuads=False,
                          computeHexahedra=False)
        gelNode.addObject('IndexValueMapper', name='youngModuli', indices='@hardLayer.tetrahedronIndices',
                          value=geometry.hardYoungModulus, defaultValue=geometry.youngModulus,
                          inputValues=[float(geometry.youngModulus)])
        youngModulus = '@youngModuli.outputValues'
    gelNode.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large',
                      poissonRatio=geometry.poissonRatio, youngModulus=youngModulus)
    gelNode.addObject('BoxROI', name='ROI1', box=geometry.box, drawBoxes='true')
    gelNode.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

    surfaceNode = gelNode.addChild("surfaceNode")
    surfaceNode.addObject('TriangleSetTopologyContainer', name="surfContainer",
                          src="@../../GelSurface/triangleContainer")
    surfaceNode.addObject('MechanicalObject', name='msSurface')
    surfaceNode.addObject('TriangleCollisionModel', name='surface')
    surfaceNode.addObject('LineCollisionModel', name='line')
    surfaceNode.addObject('BarycentricMapping')
    softColor = getattr(geometry, 'softColor', "0.0 0.1 0.9 0.40")
    if hardBox is None:
        visu = surfaceNode.addChild("visu")
        visu.addObject("OglModel", name="Visual", src="@../surfContainer", color=softColor)
        visu.addObject("BarycentricMapping", input="@..", output="@Visual")
    else:
        # The outer surface in two colours: the hard layer's triangles in hardColor
        surfaceNode.addObject('BoxROI', name='hardSurface', box=hardBox, strict=True, doUpdate=False,
                              computeEdges=False, computeQuads=False, computeTetrahedra=False,
                              computeHexahedra=False)
        for name, triangles, color in (("visu", "trianglesOutROI", softColor),
                                       ("visuHard", "trianglesInROI",
                                        getattr(geometry, 'hardColor', "0.85 0.25 0.2 0.55"))):
            visu = surfaceNode.addChild(name)
            visu.addObject("OglModel", name="Visual", position="@../surfContainer.position",
                           triangles="@../hardSurface." + triangles, color=color)
            visu.addObject("BarycentricMapping", input="@..", output="@Visual")

    gelNode.addObject('LinearSolverConstraintCorrection')

    return FemNode
