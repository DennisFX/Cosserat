# Cosserat: needle insertion on SOFA v26.06 (`needle-v26.06`)

This branch pins a known-working setup of the Cosserat plugin's **needle insertion example** on **SOFA v26.06.00**, plus a small UDP bridge for communicating with ROS.

For the plugin's own documentation, see the original README on the [`master`](https://github.com/DennisFX/Cosserat/tree/master) branch, which mirrors upstream [SofaDefrost/Cosserat](https://github.com/SofaDefrost/Cosserat).

This branch is based on upstream `main` at commit `ece9067` (2026-09-05) and is intentionally not rebased.

---

## Tested configuration

| Component | Version |
|---|---|
| SOFA | tag `v26.06.00` (commit `7c18e95`, 2026-07-13) |
| Cosserat | this branch (upstream `main` @ `ece9067` + 3 commits) |
| Environment | pixi, `supported-plugins` environment (conda-forge Python 3.12) |
| Host | Ubuntu with ROS 2 Jazzy |

**Do not use SOFA `master`.** As of September 2026, the legacy needle example does not run on it: the constraint solver crashes and the gel becomes unstable on contact.

---

## What this branch changes

Three commits on top of upstream `ece9067`:

1. **Bindings fix.** `python/Binding/Cosserat/src/Binding_HookeSeratMapping.cpp` declared a pybind11 base class (`CosseratGeometryMapping`) that is never registered, so importing the bindings failed with `ImportError: generic_type: type "HookeSeratDiscretMapping3" referenced unknown base type`. The base class is dropped from the declaration.
2. **Legacy needle example workarounds.**
   - `needleController.py`: `import Cosserat` → `from Sofa import Cosserat`. The bindings now install as a submodule of `Sofa`, and the controller needs them (`pointManager.addNewPointToState()` drives insertion).
   - `utils.py` copied from `python/cosserat/` to `examples/python3/useful/` and `examples/python3/cosserat/`, where the example still imports it from.
   - `NeedleInsertion.py`: `ProjectedGaussSeidelConstraintSolver` → `BlockGaussSeidelConstraintSolver` (renamed in SOFA v26.06).
   - `NeedleInsertion.py`: `beamPath` is passed as the sliding point's mechanical object path without the leading `@`, as `PointsManager` expects.
3. **RosListener.** A controller in `NeedleInsertion.py` that exchanges UDP messages with an external ROS process (see [ROS bridge](#ros-bridge)).

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

### Aliases

Add these to `~/.bashrc`:

```bash
alias runsofa='pixi run --manifest-path ~/sofa -e supported-plugins ~/sofa/.pixi/envs/supported-plugins/sofa-build/bin/runSofa -l SofaPython3'
alias runsofa_needle='runsofa ~/dev/Cosserat/examples/python3/examples/needle/NeedleInsertion.py'
```

`runsofa <scene.py>` runs any scene from any folder, and `runsofa_needle` opens the needle scene.

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
| `M` / `D` | Add / remove a constraint point |

If keys don't seem to reach the scene, make sure the 3D view has focus and try holding Ctrl with the key.

---

## ROS bridge

`RosListener` (in `NeedleInsertion.py`) binds UDP `127.0.0.1:9871`. On every simulation step, it reads any pending datagrams, prints them as `[ROS] <message>`, and replies `bye` to `127.0.0.1:9872`.

- Messages are only processed while the simulation is animating.
- Only one SOFA instance can run the scene at a time, since the port can only be bound once.

To test it without ROS, start the scene and press Animate, then from another terminal:

```bash
python3 -c "import socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b'hello', ('127.0.0.1', 9871))"
```

SOFA should print `[ROS] hello` followed by `[SOFA]: bye`.

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
| Deprecation warnings during compilation | SOFA API deprecations; they don't affect the build. |
