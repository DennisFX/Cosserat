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
5. **Rigid needle.** `NeedleInsertion.py` freezes the Cosserat strains with a `FixedProjectiveConstraint`, so the needle moves as one rigid body driven by its base. The scene declares the `Sofa.Component.Constraint.Projective` plugin that provides it. Remove the constraint line to get the flexible needle back. The needle also weighs 5 g (`PhysicsParams.mass`), like the Gazebo needle.
6. **Gazebo coupling.** `gazeboCoupling.py`, next to the scene, makes the needle base follow the Gazebo needle and sends the base wrench back to it (see [Following the Gazebo needle](#following-the-gazebo-needle)).
7. **Gel in a container.** `FemParams.box` in `cosserat/needle/params.py` holds the gel's x = 40 face (opposite the entry face) and its four side faces, instead of only the y = −8 face. In the Gazebo world the needle goes in from above, so the gel sits in a box on the ground with its top open. Held at the bottom only, a sideways push on the needle would tilt the whole 24 cm block instead of loading the gel around the needle.
8. **Gel points follow the needle's own depth.** `cosserat/needle/needleController.py` attaches a gel point at the tip every 1.3 cm on the way in and removes them on the way out. Upstream decided when by measuring from the tip to the deepest gel point, which moves with the gel: when the inserted needle was moved sideways, the gel could shift that point far enough to attach a new one in deformed gel, leaving a permanent sideways force. Now each point's depth (the tip's scene x when it was attached) is remembered and the tip is compared with that, so sideways moves never add or remove points. Straight insertion and retraction behave as before. Each event is printed: `[needle] punctured at x = ...`, `[needle] gel point 2 added at x = ...`, `... removed ...`, `[needle] out of the gel`.

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

## Wrench monitor

`WrenchMonitor` (in `wrenchMonitor.py`) runs after every step and updates these attributes. It prints nothing; other controllers, such as the Gazebo coupling, read them.

| Attribute | Meaning |
|---|---|
| `baseForce`, `baseTorque` | Wrench the needle holder applies **on** the needle, at the needle base, in the SOFA scene frame. Negate it to get what the tissue does to the needle. |
| `sectionMoments` | Internal bending/twisting moment per section, `[torsion, bending y, bending z]` in the section's frame. Always 0 while the needle is rigid. |
| `lateralForces` | Tissue force on the needle at each sliding-constraint point in the gel, needle-local `[y, z]`. |

The scene is in cm, kg and s, so 100 force units are 1 N and 10⁴ torque units are 1 N·m.

How the base wrench is computed: the needle base is held by a stiff spring (`RestShapeSpringsForceField`), and the monitor takes that spring's reaction **minus** the Rayleigh damping the solver adds to it. Without the damping correction, every keypress shows up as a spike of millions of units. For this to work, `rayleighStiffness` passed to the monitor must equal the solver's (the scene passes `PhysicsParams.rayleighStiffness` to both).

The monitor only needs numpy; SOFA's pixi environment has no scipy.

Things to know when reading the numbers:

- **Short blips on each keypress** are the needle's inertia while the base catches up with the new rest position. They scale with `PhysicsParams.mass` (5 g).
- **No axial force once inside the gel.** The sliding constraint ignores the needle axis, so the model has no friction along the shaft; the only axial resistance is at puncture. Moving the inserted needle sideways is what loads it: a sideways force at the base and a bending moment, whose ratio is the distance from the base to where the gel pushes back.
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
| `base_pose` | in | `p` (m), `q` (x, y, z, w), `stamp` | Where the Gazebo needle base is, in the `sofa_scene` frame; used by the coupling |
| `wrench` | out | `force`, `torque`, optional `frame` | Wrench applied to the Gazebo needle. Without `frame` it is in Gazebo's world frame; the coupling sends it in `sofa_scene` and the bridge rotates it. |
| `clear` | out | none | Stops pushing on the Gazebo needle |

Ctrl+W sends a fixed test wrench (`testForce` in `rosLink.py`) and Ctrl+R sends `clear`. The ROS side sets the Gazebo wrench rather than adding to it, so pressing W again doesn't increase the force.

- Incoming messages are read at the start of each step, so nothing is received while the scene is paused. Key presses are sent immediately, paused or not.
- Only one SOFA instance can run the scene at a time, since port 9871 can only be bound once. Reloading the scene in the same runSofa is fine.
- New message types, keys and outgoing messages can be added from the scene without editing `rosLink.py`, through `rosLink.subscribe(...)`, `rosLink.bindKey(...)` and `rosLink.send(...)`. See the docstring at the top of the file.

To test it without ROS, start the scene and press Animate. Send it a needle pose at the scene origin from another terminal:

```bash
python3 -c "import json, socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps({'type': 'base_pose', 'p': [0, 0, 0], 'q': [0, 0, 0, 1]}).encode(), ('127.0.0.1', 9871))"
```

Then press Ctrl+G: SOFA should print `[coupling] following the Gazebo needle ...`, and a second later that no pose has arrived for 1 s. Press Ctrl+G again to stop. To see what SOFA sends, stop the ROS bridge if it's running (it holds port 9872), run `nc -ul 127.0.0.1 9872`, and press Ctrl+W in SOFA.

---

## Following the Gazebo needle

`GazeboCoupling` (in `gazeboCoupling.py`) couples this scene to the UR5 in Gazebo. The ROS side, including where the scene sits in the robot's world and the insertion workflow, is described in the ros2SOFA_medical README, under [Insert the needle into the SOFA gel](https://github.com/DennisFX/ros2SOFA_medical#insert-the-needle-into-the-sofa-gel).

The scene's origin corresponds to the Gazebo needle base at a fixed **pre-insertion pose**, so at that pose both needles coincide. Press **Ctrl+G** with the arm there:

- **Engaging is refused** if the Gazebo needle is more than 1 cm or about 6° from the SOFA needle, with a message giving the offset. Otherwise the arm's travel to the pre-insertion pose would drag the needle through the gel.
- **While following**, the needle base rest pose moves toward the newest `base_pose` every step, by at most 0.5 cm and 0.05 rad per step, so a sudden jump can't hit the stiff base spring all at once. The arrow keys are overridden.
- **After every step**, the coupling sends the needle base wrench to Gazebo. That is minus the monitor's `baseForce` / `baseTorque` (what the needle does to its holder), converted to N and N·m. It goes out in the `sofa_scene` frame, and the bridge rotates it into the world frame.
- **If that wrench passes 50 N or 10 N·m**, or isn't a number, it isn't sent and following stops, as with Ctrl+G. The scene has almost certainly gone unstable, and passing the wrench on would throw the arm around, which the needle would then follow. Reload the scene before coupling again.
- **Ctrl+G again** stops following, sends a zero wrench, and leaves the needle where it is.

If poses stop arriving while following, the needle holds the last one, and SOFA prints a warning after 1 s.

Options, passed to `GazeboCoupling(...)` in `NeedleInsertion.py`:

| Option | Default | Meaning |
|---|---|---|
| `wrenchGain` | `1.0` | Multiplies the wrench sent to Gazebo; raise it to make small tissue forces visible on the arm while checking directions |
| `sendWrenches` | `True` | `False` follows Gazebo without pushing back |
| `maxForce`, `maxTorque` | `50.0`, `10.0` | Safety cutoff on the wrench sent to Gazebo, after `wrenchGain` (N, N·m). The UR5's wrist gives way somewhere above ~100 N at the needle base. `float('inf')` for both turns the cutoff off; a wrench that isn't a number still stops the coupling. |
| `maxStep`, `maxAngleStep` | `0.5`, `0.05` | Largest rest-pose change per step (scene units, rad) |
| `engageDistance`, `engageAngle` | `1.0`, `0.1` | How close the needles must be for Ctrl+G to engage (scene units, rad) |
| `metersPerUnit`, `kgPerUnit` | `0.01`, `1.0` | Scene units; the defaults are cm and kg |
| `key` | `'G'` | Toggle key, used with Ctrl. Avoid `F`: Ctrl+F makes runSofa fullscreen. |

Things to know:

- **The SOFA needle's mass shows up in Gazebo.** The wrench includes the needle's inertia while the base accelerates. At 5 g, the same as the Gazebo needle, that push is small.
- **Insert along the scene's x axis.** `needleController.py` adds and removes gel points as the tip moves along scene x, so insertions should keep the needle along its starting axis. Sideways moves at full depth are fine: they don't change the tip's x, so they never add or remove points.
- **The scene is placed vertically in the Gazebo world.** The ROS side maps the scene's +x to world −Z, so the needle goes in from above, and the gel sits in a container (see [What this branch changes](#what-this-branch-changes)). runSofa still draws the scene in its own frame, with the needle horizontal.
- **Keep sideways moves small.** The gel mesh is coarse (cells 4.8 × 3.2 × 2 cm along x, y, z) and nearly incompressible (Poisson ratio 0.48), and the container walls are 5 cm (z) and 8 cm (y) from the needle. Sideways moves of 3 and 5 mm work: 5 mm gives about 50 N along z and 25 N along y with the rigid needle, roughly 10 and 5 N/mm, and the coarse mesh couples the two (a move along z also pushes along y). 15 mm squashed the gel against the walls until the scene went unstable. A finer mesh around the needle would allow larger moves and reduce the coupling, at the cost of speed; a flexible needle (remove the `FixedProjectiveConstraint` line) would bend and take up much of the motion.

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
| Deprecation warnings during compilation | SOFA API deprecations; they don't affect the build. |