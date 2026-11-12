# Cosserat: needle insertion on SOFA v26.06 (`needle-v26.06`)

This branch pins a known-working setup of the Cosserat plugin's **needle insertion example** on **SOFA v26.06.00**. It adds a UDP link to ROS 2, a monitor for the wrench at the needle base, and a coupling that lets a UR5 simulated in Gazebo drive the needle while the needle's wrench acts back on the arm.

For the plugin's own documentation, see the original README on the [`master`](https://github.com/DennisFX/Cosserat/tree/master) branch, which mirrors upstream [SofaDefrost/Cosserat](https://github.com/SofaDefrost/Cosserat).

This branch is based on upstream `main` at commit `ece9067` (2026-09-05) and is intentionally not rebased.

---

## Tested configuration

| Component | Version |
|---|---|
| SOFA | tag `v26.06.00` (commit `7c18e95`, 2026-07-13) |
| Cosserat | this branch (upstream `main` @ `ece9067` + the changes below) |
| Environment | pixi, `supported-plugins` environment (conda-forge Python 3.12) |
| Host | Ubuntu with ROS 2 Jazzy |

**Do not use SOFA `master`.** As of September 2026, the legacy needle example does not run on it: the constraint solver crashes and the gel becomes unstable on contact.

---

## What this branch changes

On top of upstream `ece9067`:

1. **Bindings fix.** `python/Binding/Cosserat/src/Binding_HookeSeratMapping.cpp` declared a pybind11 base class (`CosseratGeometryMapping`) that is never registered, so importing the bindings failed with `ImportError: generic_type: type "HookeSeratDiscretMapping3" referenced unknown base type`. The base class is dropped from the declaration.
2. **Legacy needle example workarounds.**
   - `needleController.py`: `import Cosserat` → `from Sofa import Cosserat`. The bindings now install as a submodule of `Sofa`, and the controller needs them (`pointManager.addNewPointToState()` drives insertion).
   - `utils.py` copied from `python/cosserat/` to `examples/python3/useful/` and `examples/python3/cosserat/`, where the example still imports it from.
   - `NeedleInsertion.py`: `ProjectedGaussSeidelConstraintSolver` → `BlockGaussSeidelConstraintSolver` (renamed in SOFA v26.06).
   - `NeedleInsertion.py`: `beamPath` is passed as the sliding point's mechanical object path without the leading `@`, as `PointsManager` expects.
3. **ROS link.** `rosLink.py`, next to the scene, exchanges JSON messages over UDP with the ROS 2 bridge node: the Gazebo needle pose in, wrenches out (see [ROS link](#ros-link)).
4. **Wrench monitor.** `wrenchMonitor.py`, next to the scene, computes the wrench at the needle base every step (see [Wrench monitor](#wrench-monitor)).
5. **Rigid or flexible needle.** A `FixedProjectiveConstraint` line in `NeedleInsertion.py` freezes the Cosserat strains, so the needle moves as one rigid body driven by its base; commented out, the needle is flexible. The scene declares the `Sofa.Component.Constraint.Projective` plugin either way. The choice matters: in the bottom-held gel the flexible needle bends and takes up about 60% of a sideways move, so the forces at its base are less than half the rigid needle's (see [Following the Gazebo needle](#following-the-gazebo-needle)). Check which one is active before comparing results. The needle also weighs 5 g (`PhysicsParams.mass`), like the Gazebo needle.
6. **Gazebo coupling.** `gazeboCoupling.py`, next to the scene, makes the needle base follow the Gazebo needle and sends the base wrench back to it, with SOFA and Gazebo in lockstep on Gazebo's clock (see [Following the Gazebo needle](#following-the-gazebo-needle)). If the wrench goes over its safety limit, it stops and tells ROS why, and the ROS side takes the arm back to the pre-insertion pose.
7. **Gel held at its bottom.** `FemParams.box` in `cosserat/needle/params.py` holds the gel's x = 40 face (opposite the entry face), instead of the y = −8 face upstream holds. In the Gazebo world the needle goes in from above, so that face is the bottom of a block standing on the ground. With one uniform soft gel, a sideways push near the top tilted the whole 24 cm block, so the gel was soft sideways; the two-layer gel (item 10) is mostly hard layer and tilts far less. The branch's history also has a container setting that holds the four side faces as well, about 30–75 times stiffer sideways (`git log -p -- examples/python3/cosserat/needle/params.py`).
8. **Gel points follow the needle's own depth.** `cosserat/needle/needleController.py` attaches a gel point at the tip every 1.3 cm on the way in and removes them on the way out. Upstream decided when by measuring from the tip to the deepest gel point, which moves with the gel: when the inserted needle was moved sideways, the gel could shift that point far enough to attach a new one in deformed gel, leaving a permanent sideways force. Now each point's depth (the tip's scene x when it was attached) is remembered and the tip is compared with that, so sideways moves never add or remove points. Straight insertion and retraction behave as before. Each event is printed: `[needle] punctured at x = 16.40, 3.02 N on 2 contact(s)`, `[needle] gel point 2 added at x = ...`, `... removed ...`, `[needle] out of the gel`.
9. **Friction along the shaft.** `CosseratNeedleSlidingConstraint` (C++) can hold each gel point along the needle as well as sideways, with Coulomb stick-slip, and `shaftFriction.py` sets each point's friction from a friction per cm of shaft in each gel layer. Upstream held the points sideways only, so once through the surface nothing resisted the needle along its axis. See [Friction along the shaft](#friction-along-the-shaft).
10. **Two-layer gel.** `createFemRegularGrid.py` gives the gel a soft top layer (the subcutaneous tissue) and a 10 times stiffer layer from 2.4 cm down, drawn red, on a mesh refined in depth. The shaft friction is 3 times higher in the hard layer, so the insertion force shows where the needle crosses into it. See [Two layers](#two-layers).
11. **Puncture on the total contact force.** `needleController.py` lets the needle through when the normal forces of all its contacts with the gel add up to the threshold. Upstream compared only the first contact's, so when the load was shared between contacts the needle pushed harder before puncturing: about 6 N instead of 3 N in the two-layer gel. The puncture force depended on how the tip met the mesh. See [Puncture](#puncture).

The upstream example's keyboard handling is unchanged.

---

## Prerequisites

- `git`
- [pixi](https://pixi.sh/latest/installation/)

**Keep ROS opt-in when working with SOFA.** Sourcing `/opt/ros/jazzy/setup.bash` adds ROS's libraries and Python packages to `LD_LIBRARY_PATH`, `PYTHONPATH` and `CMAKE_PREFIX_PATH`, which can leak into SOFA's build and runtime. Instead of sourcing it in `~/.bashrc`, use an alias and only run it in ROS terminals:

```bash
alias ros='source /opt/ros/jazzy/setup.bash'
```

SOFA and ROS talk over UDP, so SOFA never needs the ROS environment.

---

## Installation

### 1. Clone this branch

```bash
git clone git@github.com:DennisFX/Cosserat.git ~/dev/Cosserat
cd ~/dev/Cosserat
git switch needle-v26.06
```

The SOFA configuration below expects this clone at `~/dev/Cosserat`.

### 2. Clone SOFA, pinned to v26.06.00

```bash
git clone https://github.com/sofa-framework/sofa ~/sofa
cd ~/sofa
git switch -c dennis/v26.06.00 v26.06.00
```

### 3. Edit the `supported-plugins` preset

Three changes are needed in SOFA's `CMakePresets.json`:

- **Turn off SoftRobots.Inverse.** At v26.06 it fetches its `main` branch against SoftRobots `v26.06`, and the two don't compile together. It is only needed for inverse control.
- **Turn on Cosserat.** It isn't part of the v26.06 preset.
- **Build Cosserat from this clone** instead of fetching upstream.

The pixi `build` and `runSofa` tasks re-apply the preset on every run, so these settings must live in the preset rather than be passed as `-D` flags.

In `CMakePresets.json`, inside the `cacheVariables` of the `supported-plugins` preset, replace this block:

```json
        "PLUGIN_SOFTROBOTS_INVERSE": {
          "type": "BOOL",
          "value": "ON"
        },
```

with:

```json
        "PLUGIN_SOFTROBOTS_INVERSE": {
          "type": "BOOL",
          "value": "OFF"
        },
        "SOFA_FETCH_COSSERAT": {
          "type": "BOOL",
          "value": "ON"
        },
        "PLUGIN_COSSERAT": {
          "type": "BOOL",
          "value": "ON"
        },
        "COSSERAT_LOCAL_DIRECTORY": {
          "type": "STRING",
          "value": "$env{HOME}/dev/Cosserat"
        },
```

Leave `SOFA_FETCH_SOFTROBOTS_INVERSE` as it is. Adjust `COSSERAT_LOCAL_DIRECTORY` if this repo is cloned somewhere else.

Check the file still parses, then commit the change on the local branch:

```bash
pixi run -e supported-plugins cmake --list-presets
git commit -am "Presets: Cosserat from ~/dev/Cosserat, SoftRobots.Inverse off"
```

The first `pixi run` installs the environment and takes a while.

### 4. Build

```bash
pixi run -e supported-plugins build
```

On machines with limited RAM (e.g. WSL2 with 8 GB), limit parallel jobs, since some SOFA files need 1–2 GB each to compile:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e supported-plugins build
```

### 5. Verify the configuration

```bash
grep -E "^(PLUGIN_COSSERAT|COSSERAT_LOCAL_DIRECTORY|PLUGIN_SOFTROBOTS_INVERSE)" \
  ~/sofa/.pixi/envs/supported-plugins/sofa-build/CMakeCache.txt
```

Expected:

```
COSSERAT_LOCAL_DIRECTORY:STRING=/home/<user>/dev/Cosserat
PLUGIN_COSSERAT:BOOL=ON
PLUGIN_SOFTROBOTS_INVERSE:BOOL=OFF
```

---

## Running the needle scene

Use the **build-tree** `runSofa`, not the pixi `runSofa` task. The task runs the installed copy of SOFA, which doesn't add `examples/python3` to Python's path, so the example fails with `ModuleNotFoundError: No module named 'cosserat'`.

```bash
cd ~/sofa
pixi run -e supported-plugins .pixi/envs/supported-plugins/sofa-build/bin/runSofa -l SofaPython3 \
  ~/dev/Cosserat/examples/python3/examples/needle/NeedleInsertion.py
```

`-l SofaPython3` is required. SofaPython3 isn't in the default plugin list, and without it runSofa can't load `.py` scenes.

At startup, confirm this line appears:

```
[SofaPython3] Added '/home/<user>/dev/Cosserat/examples/python3/' to sys.path
```

### Shortcuts

Add these to `~/.bashrc`:

```bash
runsofa() {
  (cd ~/sofa && pixi run -e supported-plugins .pixi/envs/supported-plugins/sofa-build/bin/runSofa -l SofaPython3 "$@")
}
alias runsofa_needle='runsofa ~/dev/Cosserat/examples/python3/examples/needle/NeedleInsertion.py'
```

`runsofa <scene.py>` runs any scene, and `runsofa_needle` opens the needle scene. `runsofa` is a function rather than an alias because pixi has to run from `~/sofa`. The parentheses keep your terminal where it was, but they also mean scene paths must be absolute (`~/...` is fine).

### After editing code

- **Python** changes in this repo take effect the next time the scene is launched.
- **C++** changes need a rebuild: `pixi run -e supported-plugins build` from `~/sofa`.

### Keyboard controls

Defined in `examples/python3/cosserat/needle/needleController.py` (`onKeypressedEvent`):

| Key | Effect |
|---|---|
| ← / → | Move the needle base along X |
| ↑ / ↓ | Move the needle base along Y |
| `I` / `K` | Move the needle base along Z |
| `M` / `D` | Add / remove a constraint point (the controller re-syncs its remembered depths) |
| Ctrl + `G` | Start / stop following the Gazebo needle (`gazeboCoupling.py`) |
| Ctrl + `W` | Send the test wrench to the Gazebo needle (`rosLink.py`) |
| Ctrl + `R` | Release it |

If keys don't seem to reach the scene, make sure the 3D view has focus and try holding Ctrl with the key. `W` and `R` always need Ctrl.

---

## Tissue model

What acts on the needle, in order of insertion. The parameters are in `examples/python3/cosserat/needle/params.py`. Forces are in scene units (0.01 N), lengths in cm.

### Puncture

Until it punctures, the needle touches the gel's surface through collision contacts (`FrictionContactConstraint`, μ = 0.1). After every step, `needleController.py` adds up the contacts' normal forces and lets the needle through once they reach `threshold`. The threshold is 3.0 in the solver's units (force × dt), which is 3 N at the scene's dt = 0.01. The contacts are then switched off and the first gel point is attached at the tip.

- `linesPerContact` (3) tells the controller where each contact's normal force is: with friction the solver has a normal and two tangential lines per contact. Set it to 1 if the contact response's μ is set to 0.
- With the two-layer gel the needle punctures after about 2 mm of indentation, and the force at the base peaks at about 3 N. A uniform gel on the same mesh (`hardYoungModulus = 100`) takes the same 3 N but about 8 mm: without the hard layer 2.4 cm below, the surface gives way more.
- The tip has no cutting force inside the gel. Once through, only the gel points act on the needle, so crossing into the hard layer gives no second puncture.

### Gel points

Inside the gel, a gel point is attached at the tip every `ConstraintsParams.constraintDistance` (1.3 cm) and removed again on the way out (see item 8 above). `CosseratNeedleSlidingConstraint` holds each point on the needle axis with two lines, needle-local y and z. With friction it adds a third line along the needle, so each point has `[y, z]` or `[y, z, axial]` lines.

### Friction along the shaft

Along the needle, a gel point sticks to the shaft until holding it there takes more than

    friction[i] + frictionCoefficient × |sideways force at point i|

and then slides, with that force opposing the slip. The needle feels the opposite force, which reaches the base wrench and Gazebo. `NeedleFrictionResolution` (in `CosseratNeedleSlidingConstraint.h`) solves each point's three lines together, in the same Gauss-Seidel loop as the rest of the scene.

`ShaftFriction` (in `shaftFriction.py`) sets `friction[i]` before every step from a friction per cm of shaft in each gel layer: `frictionPerLength` in the soft top layer, `hardFrictionPerLength` from the top of the hard layer down. The shaft from the entry point to the tip is split between the points at the midpoints between them, and each point gets the friction of its share; a share that crosses the layer boundary is split there. So the friction grows smoothly with depth instead of jumping at each new point, and its slope changes where the tip crosses into the hard layer, not when the next point is added. Points ahead of the tip, which happens while retracting, get none.

The boundary is where it is in the gel at rest, from `hardLayerTop()` in `createFemRegularGrid.py`, the same place the gel builder starts the hard layer; the gel near the needle moves a mm or two along it, which the friction ignores. A uniform gel (`hardYoungModulus = youngModulus`) has no hard layer, so `frictionPerLength` applies throughout.

| Parameter (`ConstraintsParams`) | Default | Meaning |
|---|---|---|
| `frictionPerLength` | `20.0` | Friction per cm of shaft in the soft top layer, scene units: 0.2 N/cm. A placeholder, chosen to be visible next to the 3 N puncture; calibrate it before quoting numbers. |
| `hardFrictionPerLength` | `60.0` | The same in the hard layer: 0.6 N/cm, so about 2 N at the 5 cm insertion (2.4 cm soft, 2.6 cm hard). Also a placeholder, 3 times the soft layer's so that the boundary shows. |
| `frictionCoefficient` | `0.0` | Coulomb coefficient on each point's sideways force |

All three 0: the constraint builds only the original two lines per point, as upstream.

The constraint's data, for use in other scenes:

| Data | Meaning |
|---|---|
| `friction` | Friction limit per gel point, scene units: one value for all points, or one per point (points past the end of the list get none). Empty (the default): no friction line, unless `frictionCoefficient` is set. |
| `frictionCoefficient` | Adds `frictionCoefficient` × the sideways force at each point to its limit |
| `useDirections` | Sideways lines to build, needle-local y and z. The first entry is ignored: friction adds the line along the needle. |

What it does in the insert-sweep-retract cycle (two-layer gel, rigid needle, 2026-11-12, read off the plots; c is the Gazebo arm's compensation gain):

- **Insertion:** the force along the needle grows by about 0.2 N/cm in the soft layer and 0.59 N/cm once the tip is in the hard layer, from about 2.9 s, 0.1–0.2 s after the tip reaches the boundary at rest. It reaches about 2 N at full depth. With 0.2 N/cm throughout (2026-11-11) it grew by about 0.195 N/cm, slightly under 0.2, because the friction drags the entry point down a couple of mm.
- **Sweep:** static friction holds that force, but each sideways move changes it. At c = 1, 15 mm moves swing it from +2 N to between about −1.1 and −1.4 N, and the moves back to the centre bring it back to between 0 and +1.7 N. At c = 0 the arm gives way along the needle, and it stays between about +0.9 and +1.7 N. With 0.2 N/cm throughout, the swings stopped at about −0.9 N, near that friction's 1 N limit, so the points probably slid along the needle then.
- **Retraction:** the force flips to about −2 N within about 0.1 s, shrinks by about 0.6 N/cm until the tip leaves the hard layer (about 23.8 s) and by 0.2 N/cm after that, and reaches 0 as the tip leaves the gel.
- **New gel points:** each time one is added, the force dips for one step, by about 0.1 N at 0.2 N/cm (measured in the uniform gel) and up to about 0.2 N in the hard layer. Probably the new point starts with no load while the previous point's share of the shaft, and so its limit, is halved.

### Two layers

`createFemCubeWithParams` (in `cosserat/createFemRegularGrid.py`) builds a soft top layer and, from `hardDepth` below the top surface (x = 16) down to the bottom, a hard one. A `BoxROI` picks the tetrahedra that lie entirely in the hard layer at rest. An `IndexValueMapper` gives them `hardYoungModulus` and every other tetrahedron `youngModulus`, and `TetrahedronFEMForceField` takes one modulus per tetrahedron. A second `BoxROI` splits the outer surface into two visual models, one per colour. `IndexValueMapper` comes from `Sofa.Component.Engine.Transform`, which is now in the scene's plugin list.

| Parameter (`FemParams`) | Default | Meaning |
|---|---|---|
| `youngModulus` | `100` | Soft top layer, the subcutaneous tissue. Scene units are 100 Pa, so this is 10 kPa. |
| `hardYoungModulus` | `1000` | Hard layer, 100 kPa. Set it equal to `youngModulus` for one uniform gel. |
| `hardDepth` | `2.4` | cm below the top surface where the hard layer starts: halfway along the 5 cm insertion |
| `softColor`, `hardColor` | blue, red | RGBA, see-through |
| `mesh` | `"21 6 6"` | Nodes along x (depth), y, z: cells 1.2 × 3.2 × 2 cm |

- **`hardDepth` must fall on a row of nodes**, a multiple of the cell depth (1.2 cm). Otherwise the layer starts at the next row down, and the scene prints `[gel] no node plane 2.5 cm deep (cells 1.2 cm deep): the hard layer starts at 3.6 cm`.
- **The mesh is finer than before.** The old cells were 4.8 cm deep, so the whole 5 cm insertion sat within the top row of cells and there was nowhere to put a boundary. The gel now has 3000 tetrahedra instead of 750, so SOFA steps more slowly; in lockstep, Gazebo then waits for it.
- **To get the old gel back**, set `hardYoungModulus = 100` and `mesh = "6 6 6"`. `hardYoungModulus = 100` alone gives a uniform gel on the new mesh, which is the fair baseline for the layered one, since the finer mesh alone changes the stiffness too.
- **Don't make the hard layer much stiffer than the arm.** The gel's sideways stiffness grows almost in proportion to `hardYoungModulus` (next item), so along scene y it reaches the arm's 2 N/mm at about 1700, and the coupled loop with Gazebo goes unstable once the gel is much stiffer than the arm (2 N/mm sideways by default). `hardYoungModulus = 100000` made the arm oscillate by tens of cm even at c = 1, from when the first gel point landed in the hard layer (2026-11-12, with the wrench cutoff off). Raise it gradually from 1000, watching the c = 0 sideways deflection.
- **With the rigid needle** the two-layer gel is about 0.80 N/mm sideways along scene z (world X) and 1.18 N/mm along scene y (world Y), 6–7 times the old uniform gel's 0.13 / 0.17 N/mm. The uniform gel on the new mesh (`hardYoungModulus = 100`, 2026-11-12) is 0.090 / 0.123 N/mm. So the finer mesh alone makes the gel about 30% softer, and the hard layer makes it about 9 times stiffer, nearly in proportion to its 10 times higher modulus: almost all of the sideways stiffness is the hard layer's.

---

## Wrench monitor

`WrenchMonitor` (in `wrenchMonitor.py`) runs after every step and updates these attributes. It prints nothing; other controllers, such as the Gazebo coupling, read them.

| Attribute | Meaning |
|---|---|
| `baseForce`, `baseTorque` | Wrench the needle holder applies **on** the needle, at the needle base, in the SOFA scene frame. Negate it to get what the tissue does to the needle. |
| `sectionMoments` | Internal bending/twisting moment per section, `[torsion, bending y, bending z]` in the section's frame. Always 0 with the rigid needle. |
| `lateralForces` | Force the gel applies on the needle at each gel point, needle-local `[y, z]`. Before the friction change it had the opposite sign (the force on the gel). |
| `axialForces` | The same along the needle (shaft friction), + toward the tip. Zeros without friction. |
| `frictionForce` | `axialForces` summed: the total shaft friction on the needle |

The scene is in cm, kg and s, so 100 force units are 1 N and 10⁴ torque units are 1 N·m.

How the base wrench is computed: the needle base is held by a stiff spring (`RestShapeSpringsForceField`), and the monitor takes that spring's reaction **minus** the Rayleigh damping the solver adds to it. Without the damping correction, every keypress shows up as a spike of millions of units. For this to work, `rayleighStiffness` passed to the monitor must equal the solver's (the scene passes `PhysicsParams.rayleighStiffness` to both).

The monitor only needs numpy; SOFA's pixi environment has no scipy.

Things to know when reading the numbers:

- **Short blips on each keypress** are the needle's inertia while the base catches up with the new rest position. They scale with `PhysicsParams.mass` (5 g).
- **Along the needle, only friction once inside the gel.** After the puncture the gel resists the needle along its axis only through shaft friction (see [Friction along the shaft](#friction-along-the-shaft)). Moving the inserted needle sideways is what loads it most: a sideways force at the base and a bending moment, whose ratio is the distance from the base to where the gel pushes back.
- **To check the monitor**, uncomment the two `ConstantForceField` lines in `NeedleInsertion.py`. They apply 1 N sideways at the tip, 15 cm from the base. Once settled, `baseForce` should be ≈ (0, −100, 0) and `baseTorque` ≈ (0, 0, −1500); print them from a controller to see them.

---

## ROS link

`RosLink` (in `rosLink.py`) connects the scene to the [`ros2sofa_bridge`](https://github.com/DennisFX/ros2SOFA_medical) node over UDP on localhost. The scene never imports `rclpy`: SOFA's pixi environment and the system ROS install load conflicting libraries, and the two crash when loaded into one process.

| Port | Direction |
|---|---|
| `9871` | ROS → SOFA (SOFA listens) |
| `9872` | SOFA → ROS (the bridge listens) |

Each datagram is one JSON object with a `type` field. All values are SI.

| `type` | Direction | Fields | Effect |
|---|---|---|---|
| `joint_error` | in | `names`, `positions`, `stamp` | UR5 joint tracking errors. Not used at the moment: `rosLink.py` notes once that it has no handler for them. |
| `base_pose` | in | `p` (m), `q` (x, y, z, w), `stamp` | Where the Gazebo needle base is, in the `sofa_scene` frame, and Gazebo's sim time for it (s); used by the coupling. The bridge sends one per TF update, 100 per second of sim time. |
| `wrench` | out | `force`, `torque`, optional `frame` and `stamp` | Wrench applied to the Gazebo needle. Without `frame` it is in Gazebo's world frame; the coupling sends it in `sofa_scene` and the bridge rotates it. `stamp` is the Gazebo sim time it was computed for; Gazebo waits for stamped wrenches (lockstep). |
| `clear` | out | optional `reason` (`key` or `limit`) and `detail` | Stops pushing on the Gazebo needle and ends the lockstep. The coupling says why it stopped: `key` for Ctrl+G, `limit` for its wrench cutoff, with the wrench in `detail`. The bridge passes that on, and on `limit` the ROS side drops the motion it is running. Ctrl+R sends no reason. |

Ctrl+W sends a fixed test wrench (`testForce` in `rosLink.py`) and Ctrl+R sends `clear`. The ROS side sets the Gazebo wrench rather than adding to it, so pressing W again doesn't increase the force.

- Incoming messages are read at the start of each step, so nothing is received while the scene is paused. Key presses are sent immediately, paused or not. Other components can also read them mid-step with `rosLink.waitForMessages(timeout)` and `rosLink.poll()`; the coupling does, while it waits for Gazebo.
- Only one SOFA instance can run the scene at a time, since port 9871 can only be bound once. Reloading the scene in the same runSofa is fine.
- New message types, keys and outgoing messages can be added from the scene without editing `rosLink.py`, through `rosLink.subscribe(...)`, `rosLink.bindKey(...)` and `rosLink.send(...)`. See the docstring at the top of the file.

To test it without ROS, start the scene and press Animate. Send it a needle pose at the scene origin from another terminal:

```bash
python3 -c "import json, socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps({'type': 'base_pose', 'p': [0, 0, 0], 'q': [0, 0, 0, 1]}).encode(), ('127.0.0.1', 9871))"
```

Then press Ctrl+G: SOFA should print `[coupling] following the Gazebo needle ...`, and a second later that no pose has arrived for 1 s. (The pose has no `stamp`, so the coupling doesn't try to run in lockstep.) Press Ctrl+G again to stop. To see what SOFA sends, stop the ROS bridge if it's running (it holds port 9872), run `nc -ul 127.0.0.1 9872`, and press Ctrl+W in SOFA.

---

## Following the Gazebo needle

`GazeboCoupling` (in `gazeboCoupling.py`) couples this scene to the UR5 in Gazebo. The ROS side, including where the scene sits in the robot's world and the insertion workflow, is described in the ros2SOFA_medical README, under [Insert the needle into the SOFA gel](https://github.com/DennisFX/ros2SOFA_medical#insert-the-needle-into-the-sofa-gel).

The scene's origin corresponds to the Gazebo needle base at a fixed **pre-insertion pose**, so at that pose both needles coincide. Press **Ctrl+G** with the arm there:

- **Engaging is refused** if the Gazebo needle is more than 1 cm or about 6° from the SOFA needle, with a message giving the offset. Otherwise the arm's travel to the pre-insertion pose would drag the needle through the gel.
- **While following**, the needle base rest pose moves toward the Gazebo pose for the step (see [Lockstep with Gazebo's clock](#lockstep-with-gazebos-clock)) every step, by at most 0.5 cm and 0.05 rad per step, so a sudden jump can't hit the stiff base spring all at once. The arrow keys are overridden.
- **After every step**, the coupling sends the needle base wrench to Gazebo. That is minus the monitor's `baseForce` / `baseTorque` (what the needle does to its holder), converted to N and N·m. It goes out in the `sofa_scene` frame, and the bridge rotates it into the world frame.
- **If that wrench passes 50 N or 10 N·m**, or isn't a number, it isn't sent and following stops. The scene has almost certainly gone unstable, and passing the wrench on would throw the arm around, which the needle would then follow. The coupling sends a zero wrench and tells ROS why (`clear` with reason `limit`). `sofa_ros_interaction` then drops the motion it is running and takes the arm back to the pre-insertion pose. The SOFA needle stays where it stopped, in the gel, and SOFA keeps stepping. Ctrl+G won't engage again, since the two needles are now far apart: reload the scene.
- **Ctrl+G again** stops following, sends a zero wrench (`clear` with reason `key`), and leaves the needle where it is. A motion running on the ROS side carries on.

Options, passed to `GazeboCoupling(...)` in `NeedleInsertion.py`:

| Option | Default | Meaning |
|---|---|---|
| `wrenchGain` | `1.0` | Multiplies the wrench sent to Gazebo; raise it to make small tissue forces visible on the arm while checking directions |
| `sendWrenches` | `True` | `False` follows Gazebo without pushing back |
| `maxForce`, `maxTorque` | `50.0`, `10.0` | Safety cutoff on the wrench sent to Gazebo, after `wrenchGain` (N, N·m). The UR5's wrist gives way somewhere above ~100 N at the needle base. `float('inf')` for both turns the cutoff off; a wrench that isn't a number still stops the coupling. Check what `NeedleInsertion.py` passes. |
| `maxStep`, `maxAngleStep` | `0.5`, `0.05` | Largest rest-pose change per step (scene units, rad) |
| `lockstep` | `True` | Run on Gazebo's clock when poses are stamped; `False` follows the newest pose and sends unstamped wrenches, as before |
| `maxWait` | `0.2` | s of wall time to wait for Gazebo's pose before taking a step without it |
| `resyncAfter` | `0.1` | s of Gazebo time the clocks may drift apart before SOFA jumps to Gazebo's |
| `staleAfter` | `1.0` | Without lockstep: s without a pose before warning |
| `engageDistance`, `engageAngle` | `1.0`, `0.1` | How close the needles must be for Ctrl+G to engage (scene units, rad) |
| `metersPerUnit`, `kgPerUnit` | `0.01`, `1.0` | Scene units; the defaults are cm and kg |
| `key` | `'G'` | Toggle key, used with Ctrl. Avoid `F`: Ctrl+F makes runSofa fullscreen. |

Things to know:

- **The SOFA needle's mass shows up in Gazebo.** The wrench includes the needle's inertia while the base accelerates. At 5 g, the same as the Gazebo needle, that push is small.
- **Insert along the scene's x axis.** `needleController.py` adds and removes gel points as the tip moves along scene x, so insertions should keep the needle along its starting axis. Sideways moves at full depth are fine: they don't change the tip's x, so they never add or remove points.
- **The scene is placed vertically in the Gazebo world.** The ROS side maps the scene's +x to world −Z, so the needle goes in from above, and the gel stands on its x = 40 face (see [What this branch changes](#what-this-branch-changes)). runSofa still draws the scene in its own frame, with the needle horizontal.
- **Sideways stiffness.** In the two-layer gel, a 15 mm sideways move at full depth (5 cm into the gel) gives about 12 N along z and 17.7 N along y with the rigid needle (0.80 and 1.18 N/mm), with about 1.9 and 2.55 N·m at the base. Without the hard layer, on the same mesh, it gives about 1.35 N along z and 1.85 N along y (0.090 and 0.123 N/mm), with about 0.25 and 0.29 N·m. In the earlier uniform gel, on the coarse mesh, a 5 mm move gave about 0.66 N along z and 0.83 N along y with the rigid needle (0.13 and 0.17 N/mm), and about 0.25 and 0.35 N with the flexible needle. The moment at the base was then about 0.13 N·m with the rigid needle and 0.03–0.04 N·m with the flexible one.
- **Keep sideways moves small in the container gel.** Its mesh was coarse when it was tried (cells 4.8 × 3.2 × 2 cm along x, y, z) and nearly incompressible (Poisson ratio 0.48), and the container walls are 5 cm (z) and 8 cm (y) from the needle. Sideways moves of 3 and 5 mm work: 5 mm gives about 50 N along z and 25 N along y with the rigid needle, roughly 10 and 5 N/mm, and the coarse mesh couples the two (a move along z also pushes along y). 15 mm squashed the gel against the walls until the scene went unstable. A finer mesh around the needle would allow larger moves and reduce the coupling, at the cost of speed.

### Lockstep with Gazebo's clock

SOFA advances 0.01 s per step however long the step takes, so on its own the gel runs as fast as the computer allows: about 3 times faster than the arm on one test machine, half as fast on a slower one. While following, the coupling runs SOFA on Gazebo's clock instead:

- At Ctrl+G, SOFA's next step starts at the sim time (`stamp`) of the newest `base_pose`, and each step after it covers the next 0.01 s of Gazebo time.
- Before each step, SOFA waits until it has the Gazebo pose from the step's start (stamped within half a step of it), and drives the needle base toward that pose. A fast SOFA thus waits for Gazebo.
- Each wrench is stamped with the Gazebo time at the end of its step. Gazebo's `SetLinkWrench` system doesn't let Gazebo get more than 0.02 s past the newest stamp, so a slow SOFA slows Gazebo down.

Both then run at the pace of the slower one. The ROS side (`SetLinkWrench`, the bridge and the insertion services) is described in the ros2SOFA_medical README, under [Keep SOFA and Gazebo on one clock](https://github.com/DennisFX/ros2SOFA_medical#keep-sofa-and-gazebo-on-one-clock).

What SOFA prints:

| Message | Meaning |
|---|---|
| `[coupling] following the Gazebo needle and sending it the needle wrench, in lockstep with Gazebo's clock` | Coupled, with stamped poses coming in |
| `[coupling] Gazebo's clock has not reached SOFA's for 0.2 s (paused?); holding the needle until it does` | No pose for the step within `maxWait`: Gazebo is paused or gone. SOFA steps on (about every 0.2 s) with the needle held and sends nothing. |
| `[coupling] Gazebo is moving again` | Poses are coming again; lockstep continues |
| `[coupling] Gazebo's clock is +3.20 s from SOFA's; continuing from Gazebo's` | The clocks were more than `resyncAfter` apart, usually because SOFA was paused longer than Gazebo waits for it (5 s). SOFA jumps to Gazebo's time. |
| `[coupling] the Gazebo needle pose comes every 50 ms of Gazebo time, so SOFA follows it that coarsely` | Poses come less often than every step. SOFA then uses the newest pose it can get instead of waiting for one Gazebo isn't allowed to reach. Usually `robot_state_publisher` at its default 20 Hz; `ur5_gazebo.launch.py` sets it to publish every joint state. |

Poses without a `stamp` (an older bridge, or a pose sent by hand) make the coupling follow the newest pose without lockstep, as it did before. In that mode, if poses stop arriving, the needle holds the last one and SOFA prints a warning after 1 s.

---

## Messages that are safe to ignore

| Message | Cause |
|---|---|
| `SofaCUDA.Core ... Driver is too old for this runtime` | NVIDIA driver older than the CUDA runtime in the environment. The needle scene uses no GPU components. Updating the driver clears it. |
| `Plugin not found: "BeamAdapter.CUDA"` / `"SoftRobots.CUDA"` | Follows from the SofaCUDA error above. |
| `environment variable SOFA_ROOT is empty` | Cosmetic; runSofa finds its paths itself. |
| `Trying to add a BaseMeshTopology ... into the Node '/FemNode/gelVolume'` | Comes from the upstream gel setup; the scene runs correctly on v26.06. |
| `Class already registered in the ObjectFactory: InterventionalRadiologyController<...>` | Duplicate registration between BeamAdapter and BeamAdapter.CUDA. |
| `Skipped running the post-link scripts` (pixi) | pixi skips package install scripts by default; the one listed belongs to librsvg and doesn't affect SOFA. |
| `Cannot set window position/size from settings` | First run with an empty `~/.config/SOFA`; disappears afterwards. |
| `[MechanicalObject(MousePosition)] Invalid vOp operation` | Clicking in the 3D view (mouse picking); doesn't affect the simulation. |
| `[SceneCheckMultithreading] ... parallel implementation is available` | A suggestion, not an error. |
| `[rosLink] no handler for "joint_error" messages, ignoring them` | The bridge forwards the arm's joint errors, which the scene doesn't use; printed once. |
| Deprecation warnings during compilation | SOFA API deprecations; they don't affect the build. |