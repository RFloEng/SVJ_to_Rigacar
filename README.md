# SVJ to Rigacar

Convert a car described in the [SVJ standard](https://github.com/RFloEng/SVJ-standard-vehicle-json)
(Standard Vehicle JSON) into a **rigged Blender car** built on the [Rigacar](https://github.com/digicreatures/rigacar)
suspension rig — with the **suspension geometry and kinematics taken from the SVJ**, not from the template.

The SVJ describes every suspension as wheel stations: bodies (uprights, wishbones, links, tie rods, rack, rigid axles),
joints, springs and steering. This tool solves that mechanism with the SVJ loop-closure solver and bakes the result into
driver tables, so in Blender the arms, uprights, rack and axles move exactly as the SVJ says they should:
camber, toe, track change, caster and Ackermann all come from the SVJ.

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation (Blender add-on)](#installation-blender-add-on)
- [Usage](#usage)
- [Rig controls](#rig-controls)
- [How it works](#how-it-works)
- [What comes from the SVJ](#what-comes-from-the-svj)
- [Verification](#verification)
- [Limitations and assumptions](#limitations-and-assumptions)
- [Repository layout](#repository-layout)
- [Development](#development)
- [Licenses](#licenses)

## Features

- One-click import of an `.svj.json` as a complete Rigacar-style rig (ground sensing, steering, wheel spin, body
  suspension control).
- Suspension **kinematics solved from the SVJ**: every body of the mechanism is driven by tables computed with the SVJ
  loop-closure solver (spec section 26), for wheel travel, steering rack, and heave/roll of shared rigid axles.
- Wheel travel limited by the SVJ bump / rebound stops.
- Springs and dampers rebuilt between their SVJ mounts (stretch-to bones).
- Legacy SVJ v0.95 `suspension` sections are migrated automatically (`svj.migrate_suspension`).
- Optional car mesh: a `.glb` next to the SVJ is imported and bound to the body and wheel bones.
- Optional proxy geometry: wishbones, links, uprights, axle and springs drawn as tubes (not rendered) so the
  mechanism can be inspected.
- Works as a **Blender add-on** or as a **command-line script** (batch conversion).
- A verification script compares the rig against the SVJ solver.

## Requirements

- Blender 4.2 or newer (developed and tested on Blender 5.2).
- The [Rigacar](https://github.com/digicreatures/rigacar) add-on (7.1), which generates the base rig. Nothing from
  Rigacar is shipped here: it is found in `~/.svj_car_rig`, `$RIGACAR`, `--rigacar`, or an installed copy, or downloaded
  from its GitHub release (button in the add-on preferences, or `--download-rigacar`). Rigacar 7.1 predates Blender 4.0,
  so a few lines are patched at runtime on a temporary copy (see `rigacar_template.py`).
- The `svj-py` folder of the [SVJ repository](https://github.com/RFloEng/SVJ-standard-vehicle-json) (the solver). It is
  bundled into the add-on zip when you build it.
- An SVJ file with `wheel_stations` for the four wheels `FL FR RL RR` (or a legacy v0.95 `suspension` section).

## Installation (Blender add-on)

1. Build the add-on zip, pointing at the `svj-py` folder of the SVJ repo:
   ```bash
   python make_addon.py /path/to/SVJ-standard-vehicle-json/svj-py
   ```
   (or set the `SVJ_PY` environment variable). This produces `dist/svj_car_rig.zip`, containing the converter and the
   SVJ solver.
2. In Blender: **Edit > Preferences > Add-ons > Install from Disk** and choose `dist/svj_car_rig.zip`. Enable
   "SVJ Car Rig".
3. In the add-on preferences press **Download Rigacar 7.1** (or install Rigacar / set its path). In the same place you can
   point to a different `svj-py` folder; leave it empty to use the bundled one.

## Usage

### In Blender

- **File > Import > SVJ car (rigged)**, then pick the `.svj.json`.
  - If there is a `.glb` / `.gltf` in the same folder it is used as the car mesh (this can be switched off or replaced
    by an explicit mesh path in the import options).
  - Without a mesh you can generate a proxy body and wheels.
  - The suspension proxy (arms, links, springs) can be switched off.
- Or open the **SVJ Car** tab in the 3D View sidebar (**N**).
- The rig is appended to the current scene in a collection named `SVJ <car>`. The conversion takes 10–20 s and runs in a
  background Blender; its log is stored in the text block `SVJ import log`.

### From the command line

```bash
blender -b --python svj_to_blender_rig.py -- car.svj.json --mesh car.glb --out car.rig.blend
```

| Option | Effect |
|---|---|
| `--mesh F` | import and bind a car mesh (glb/gltf/fbx/obj; glb nodes `SVJ::wheel::wheel_fl…` and `SVJ::body::chassis` are recognised) |
| `--no-proxy` | no proxy body/wheels when there is no `--mesh` |
| `--no-susp-proxy` | no tube proxy for arms, links and springs |
| `--svj-py P` | `svj-py` folder (otherwise `SVJ_PY` or the bundled copy) |
| `--rigacar P` | Rigacar folder or release zip used to generate the base rig |
| `--download-rigacar` | download Rigacar 7.1 from GitHub if it is not found |
| `--template F` | use a ready-made rig template `.blend` instead of generating one |
| `--out F` | output file (default `<svj name>.rig.blend` next to the SVJ) |

The kinematics can also be extracted without Blender, for inspection:

```bash
python -I svj_suspension_kinematics.py car.svj.json
```

## Rig controls

Select the armature `Car Rig`, enter Pose Mode (**Ctrl+Tab**, or the **Enter Pose Mode** button in the SVJ Car panel),
and use the control bones:

| Bone | What it does |
|---|---|
| `Steering` | location X steers the front wheels (the rig converts the steering angle to rack travel through the SVJ solver tables) |
| `Suspension` | body heave / roll / pitch relative to the ground; the wheels stay on the ground and the SVJ mechanism follows |
| `Root` / ground sensors | move the car over the generated `Ground` plane; wheels are projected onto it (wheel radius from the SVJ) |
| `Wheel.*` | wheel rotation (spin), as in Rigacar |

The generated bones `KIN.*`, `MCH-Kin.*`, `SPR.*` and `TGT.*` live in the hidden bone collection **SVJ suspension**.
The armature custom property `svj_kinematics_report` contains every assumption and warning of the conversion (also
available from the panel).

## How it works

1. **Extraction** (`svj_suspension_kinematics.py`, no Blender dependency): the SVJ is loaded (and migrated if legacy),
   wheel stations are read, and the loop-closure solver is swept over each input — wheel travel between the bump and
   rebound stops, steering rack travel, and heave/roll for axles shared by two wheels — with continuation. The pose
   (position and rotation) of each body is recorded for every input value.
2. **Rig construction** (`svj_to_blender_rig.py`): Rigacar generates a default car rig, which is used as the template; its armature is re-fitted to the SVJ geometry
   (wheelbase, tracks, wheel radius, wheel centres); ground, shrinkwrap distance and axle limits are re-set; then for each
   solved body a chain of bones `KIN.<body>.<input>` is created under the body bone and driven by F-curve tables on the
   armature properties (`MCH-Kin.<corner>` measures each wheel's travel relative to the body). The wheels are
   parented to the solved upright, so everything that follows the upright (camber, toe, track change, steer) is the SVJ's.
3. **Steering**: the template's wheel yaw is read at the ground sensor and mapped to rack position with the solver table,
   so toe, caster and Ackermann are the SVJ's, not an ideal parallel steering.
4. **Springs and dampers** become stretch-to bones between their SVJ mounts.
5. **Mesh / proxies**: optional car mesh binding or proxy geometry.

The Blender frame of the result: the car faces -Y (Rigacar convention), left is +X, Z is up, the rear axle is at y = 0 and
the front axle at y = -wheelbase. The SVJ uses SAE J670 (X forward, Y right, Z down, origin at the front axle on the
ground); the conversion handles the change of frame.

## What comes from the SVJ

| Item | Source |
|---|---|
| Wheelbase, front/rear track, wheel radius, wheel centres | `wheel_stations` |
| Position and orientation of uprights, wishbones, links, tie rods, rack, rigid axle for every travel / steer | loop-closure solver over the SVJ bodies and joints |
| Camber, toe, track change, recession, caster, Ackermann | consequence of the solved uprights |
| Wheel travel limits | `bump_stop_*` / `rebound_stop_*` |
| Spring and damper mounts and lengths | `spring` / `damper` elements |
| Centre of gravity | stored as the custom property `svj_cog_blender` |

The template still provides the rig plumbing (ground projection, controls, wheel spin, widgets).

## Verification

`verify_rig_vs_svj.py` reads the wheel travel and steering angle the rig sees, solves the same state with the SVJ
solver, and compares joint-point positions per axle, wheel centres, camber/toe and spring lengths over a set of cases
(rest, ground pitch/roll, steering, combined, `Suspension` control down/up/roll/pitch, extreme combination):

```bash
blender -b car.rig.blend --python verify_rig_vs_svj.py -- car.svj.json
```

Maximum error in suspension joint positions:

| Car | One input at a time (full travel, steering) | Everything combined at the extreme |
|---|---|---|
| Multi-link car (legacy v0.95 SVJ, migrated) | ≤ 0.04 mm | 5.7 mm |
| Car with shared-carrier axles and no steering joint | ≤ 0.08 mm | not measured |
| Independent front axle of a small rear-drive car | ≤ 0.08 mm | ≈ 10 mm |
| Rigid rear axle of the same car | ≈ 2 mm | ≈ 40 mm (outside the solved range, extrapolated) |

"Everything combined" pushes travel (+70–95 mm), roll and steering (≈ 17°) at the same time; it is the worst case.

Extraction was also run on a random sample of 24 cars from a library of ~2900 SVJ files, with no failures.

## Limitations and assumptions

- **Several inputs at once**: the tables of one body are composed as a chain, which is exact for a single input and a
  first-order approximation for several. A rigid axle uses `heave` and `roll`, which are almost uncoupled.
- **Incomplete data**: some SVJ files (for example those derived from Assetto Corsa) do not locate a rigid axle
  laterally, and the solver then returns an arbitrary yaw. A **virtual Panhard rod** at wheel-centre height is added
  (recorded as an assumption in the report). Some axles are over-constrained (residual ≤ 2 mm); their solved range is
  truncated, and beyond it the rig extrapolates linearly.
- **No stops in the SVJ**: the wheel travel is assumed to be ±80 mm (noted in the report).
- **No steering joint in the SVJ**: the front wheels use the template's parallel steering (warned).
- **Four-wheel cars only** (`FL FR RL RR`).
- Bushings and bearings are not modelled; the anti-roll bar is kept as data only.
- The add-on was tested in Blender 5.2 in background mode; the import freezes the UI for 10–20 s while converting.

## Repository layout

```
svj_to_blender_rig.py          rig builder (CLI script, runs inside Blender)
svj_suspension_kinematics.py   SVJ -> solved kinematics tables (no Blender dependency)
verify_rig_vs_svj.py           compares a generated rig with the SVJ solver
rigacar_template.py            generates the base rig with the Rigacar add-on (patched for Blender 4+)
addon_src/__init__.py          Blender add-on (import operator + sidebar panel)
make_addon.py                  builds dist/svj_car_rig.zip
NOTICE.md, LICENSE             licensing
```

## Development

- Edit the scripts, then rebuild the add-on with `make_addon.py`. The add-on is a thin layer: on import it runs
  `svj_to_blender_rig.py` in a background Blender and appends the result to the current scene.
- Run `python -I svj_suspension_kinematics.py car.svj.json` to print the extracted kinematics report for any SVJ.
- Run `verify_rig_vs_svj.py` after any change to the rig construction.

## Licenses

This project is **GPL-3.0** (see `LICENSE`), like Blender add-ons in general and the [Rigacar](https://github.com/digicreatures/rigacar)
add-on it drives and patches at runtime (GPL-3.0). Rigacar itself is not included in this repository. The bundled SVJ
solver (`svj-py`) is Apache-2.0, which is compatible with GPL-3.0. See `NOTICE.md`.

**No vehicle data is included**: no car meshes, textures or SVJ files. Cars from third-party libraries have their own
licenses; bring your own SVJ (and `.glb`) files.
