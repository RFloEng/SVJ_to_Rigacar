"""
Builds the rig template from the original Rigacar add-on (https://github.com/digicreatures/rigacar, GPL-3.0),
so that no Rigacar-generated .blend has to be shipped with this project.

The template is exactly what Rigacar's "Add car deformation rig" + "Generate" produce for a default car, moved +2 m in Y
(rear axle at y = 0, front axle at y = -4, wheel radius 0.5, half track 0.9): see svj_to_blender_rig.T_WB / T_HALF / T_R.

Rigacar 7.1 predates Blender 4.0 (bone groups / bone layers were removed) and one driver helper relies on a
default F-curve modifier. Those few spots are patched on a temporary copy of the add-on (never on the original);
every patch asserts that the expected Rigacar 7.1 source is there, so another version fails loudly.

Where Rigacar is found, in order:
  1. explicit path (folder containing the `rigacar` package, the package folder itself, or a release .zip)
  2. environment variable RIGACAR
  3. cache folder ~/.svj_car_rig/  (filled by `download()`)
  4. a `rigacar` folder inside any Blender add-on search path
"""
import os, sys, shutil, tempfile, zipfile, urllib.request

RELEASE_URL = "https://github.com/digicreatures/rigacar/releases/download/v7.1/rigacar.v7.1.zip"
CACHE = os.path.join(os.path.expanduser("~"), ".svj_car_rig")

# (old, new) source patches for Rigacar 7.1 on Blender >= 4.0
PATCHES = [
    ("    group = pose.bone_groups.new(name=group_name)\n    group.color_set = color_set\n"
     "    for bone_name in bone_names:\n        bone = pose.bones.get(bone_name)\n"
     "        if bone is not None:\n            bone.bone_group = group\n",
     "    pass\n"),
    ("    wheel_brake_pose_bone.bone_group = wheel_pose_bone.bone_group\n"
     "    wheel_brake_pose_bone.bone.layers = wheel_pose_bone.bone.layers\n", ""),
    ("def dispatch_bones_to_armature_layers(ob):", "def dispatch_bones_to_armature_layers(ob):\n    return"),
    ("        fmod = fcurve.modifiers[0]\n",
     "        fmod = fcurve.modifiers[0] if len(fcurve.modifiers) else fcurve.modifiers.new('GENERATOR')\n"),
]


def _is_pkg(d): return os.path.isfile(os.path.join(d, "car_rig.py")) and os.path.isfile(os.path.join(d, "__init__.py"))


def _find_pkg(root):
    """Return the folder that contains car_rig.py under `root` (root itself, root/rigacar, or one level deeper)."""
    for c in (root, os.path.join(root, "rigacar")):
        if _is_pkg(c): return c
    if os.path.isdir(root):
        for n in os.listdir(root):
            c = os.path.join(root, n)
            if os.path.isdir(c) and (_is_pkg(c) or _is_pkg(os.path.join(c, "rigacar"))):
                return c if _is_pkg(c) else os.path.join(c, "rigacar")
    return None


def locate(explicit=None):
    """Path of the Rigacar package folder (or a .zip), or None."""
    cands = [explicit, os.environ.get("RIGACAR"), CACHE]
    try:
        import addon_utils
        cands += list(addon_utils.paths())
    except Exception:
        pass
    for c in cands:
        if not c: continue
        c = os.path.abspath(os.path.expanduser(c))
        if os.path.isfile(c) and c.lower().endswith(".zip"): return c
        if os.path.isdir(c):
            p = _find_pkg(c)
            if p: return p
            for n in os.listdir(c):                       # downloaded zip kept in the cache
                if n.lower().startswith("rigacar") and n.lower().endswith(".zip"): return os.path.join(c, n)
    return None


def download(dest=CACHE):
    """Download the official Rigacar release from GitHub into `dest` and return the zip path."""
    os.makedirs(dest, exist_ok=True)
    zp = os.path.join(dest, "rigacar.v7.1.zip")
    urllib.request.urlretrieve(RELEASE_URL, zp)
    with zipfile.ZipFile(zp) as z:
        if "rigacar/car_rig.py" not in z.namelist():
            os.remove(zp); raise RuntimeError("unexpected Rigacar archive")
    return zp


def _stage(src, tmp):
    """Copy Rigacar into tmp/rigacar, applying the compatibility patches. Returns the parent folder for sys.path."""
    pkg = os.path.join(tmp, "rigacar")
    if os.path.isfile(src):
        with zipfile.ZipFile(src) as z:
            z.extractall(tmp)
    else:
        shutil.copytree(src, pkg, ignore=shutil.ignore_patterns("__pycache__", ".git*"))
    p = os.path.join(pkg, "car_rig.py")
    s = open(p, encoding="utf-8").read()
    for old, new in PATCHES:
        if old not in s:
            raise RuntimeError("Rigacar source differs from v7.1 (patch not applicable): " + old.strip().splitlines()[0])
        s = s.replace(old, new)
    open(p, "w", encoding="utf-8").write(s)
    return tmp


def build(rigacar=None, out_path=None):
    """Run inside Blender: generate the template .blend with Rigacar and return its path."""
    import bpy, mathutils
    src = locate(rigacar)
    if src is None:
        raise SystemExit("Rigacar not found. Install it from https://github.com/digicreatures/rigacar, pass --rigacar <path>, "
                         "set RIGACAR, or run with --download-rigacar.")
    tmp = tempfile.mkdtemp(prefix="svj_rigacar_")
    sys.path.insert(0, _stage(src, tmp))
    for m in [m for m in sys.modules if m == "rigacar" or m.startswith("rigacar.")]: del sys.modules[m]
    import rigacar as R
    from rigacar import car_rig
    R.register()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    V = mathutils.Vector
    C = car_rig.OBJECT_OT_armatureCarDeformationRig
    C.bones_position = {"Body": V((0, 0, .8)), "Wheel.Ft.L": V((.9, -2, .5)), "Wheel.Ft.R": V((-.9, -2, .5)),
                        "Wheel.Bk.L": V((.9, 2, .5)), "Wheel.Bk.R": V((-.9, 2, .5)),
                        "WheelBrake.Ft.L": V((.8, -2, .5)), "WheelBrake.Ft.R": V((-.8, -2, .5)),
                        "WheelBrake.Bk.L": V((.8, 2, .5)), "WheelBrake.Bk.R": V((-.8, 2, .5))}
    C.target_objects_name = {}
    shift = (0, 2, 0)                                        # rear axle at y = 0, front axle at y = -4
    bpy.ops.object.armature_car_deformation_rig(body_pos_delta=shift, front_wheel_pos_delta=shift,
                                                back_wheel_pos_delta=shift)
    bpy.ops.object.mode_set(mode="POSE")
    if bpy.ops.pose.car_animation_rig_generate() != {"FINISHED"}:
        raise SystemExit("Rigacar could not generate the animation rig")
    out_path = out_path or os.path.join(tmp, "template.blend")
    bpy.ops.wm.save_as_mainfile(filepath=out_path)
    return out_path
