from dataclasses import dataclass
import string

# Units are cm, kg, s


@dataclass
class GeometryParams:
    radius: float = 0.1
    nbSections: int = 16
    nbFrames: int = 15
    totalLength: float = 15.0


@dataclass
class PhysicsParams:
    youngModulus: float = 1.20e9
    poissonRatio: float = 0.4
    mass: float = 0.005  # kg; light so SOFA's needle inertia doesn't add to the wrench Gazebo gets
    rayleighStiffness: float = 0.1


@dataclass
class FemParams:
    # Two layers: a soft one on top (subcutaneous tissue) and a hard one from hardDepth below
    # the top surface (x = minVol[0]) down to the bottom. Moduli in scene units, 100 Pa: 100 is
    # 10 kPa. hardYoungModulus = youngModulus gives one uniform gel.
    youngModulus: float = 100  # soft top layer
    hardYoungModulus: float = 1000  # hard layer, 10 times stiffer
    hardDepth: float = 2.4  # cm: halfway along the 5 cm insertion; must be on a node plane
    softColor: string = "0.0 0.1 0.9 0.40"  # blue, see-through
    hardColor: string = "0.85 0.25 0.2 0.55"  # red
    poissonRatio: float = 0.48
    mass: float = 0.3
    rayleigh: float = 0.1
    minVol: string = "16. -8. -5."
    maxVol: string = "40. 8. 5."
    mesh: string = "21 6 6"  # nodes along x (depth), y, z: 1.2 cm cells in depth, a plane at 2.4 cm
    box: string = "39 -10 -10 41 10 10"

@dataclass
class ContactParams:
    contactDistance: float = 0.01
    alarmDistance: float = 0.1
    dataMu: string = "mu=0.1"
    angleCone: float = "0.01"
    coneFactor: float = "0"


def display():
    print(f"""
          # Geometry
          {string.ascii_uppercase[:4]}
          {string.ascii_uppercase[4:6]}
          {string.ascii_uppercase[6:8]}
          {string.ascii_uppercase[8:10]}
          """)


@dataclass
class NeedleParameters:
    # Geometry: GeometryParams = GeometryParams()
    # Physics: PhysicsParams = PhysicsParams()
    # Fem: FemParams = FemParams()
    # contact: ContactParams = ContactParams()
    pass


@dataclass
class ConstraintsParams:
    constraintDistance: float = 1.3  # distance between two constraint points
    entryForce: float = 0.3  # The required force to penetrate the volume
    # Friction along the shaft (see examples/needle/shaftFriction.py). frictionPerLength is
    # in scene units, 0.01 N per cm of shaft in the gel: 20 is 0.2 N/cm, 1 N at 5 cm depth.
    # frictionCoefficient adds mu * |sideways force| at each gel point. Both 0: no friction.
    frictionPerLength: float = 20.0
    frictionCoefficient: float = 0.0
