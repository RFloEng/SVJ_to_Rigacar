"""Build dist/svj_car_rig.zip (Blender add-on) from the converter sources.
   python make_addon.py [path-to-svj-py]"""
import os, sys, shutil, zipfile

here = os.path.dirname(os.path.abspath(__file__))
svj_py = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SVJ_PY")
if not svj_py or not os.path.isdir(os.path.join(svj_py, "svj")):
    sys.exit("usage: python make_addon.py <path to svj-py of the SVJ repo>   (or set SVJ_PY)")
pkg = os.path.join(here, "dist", "svj_car_rig")
shutil.rmtree(os.path.join(here, "dist"), ignore_errors=True)
os.makedirs(pkg)
shutil.copy(os.path.join(here, "addon_src", "__init__.py"), pkg)
for f in ("LICENSE", "NOTICE.md"):
    shutil.copy(os.path.join(here, f), pkg)
for f in ("svj_to_blender_rig.py", "svj_suspension_kinematics.py", "rigacar_template.py"):
    shutil.copy(os.path.join(here, f), pkg)
shutil.copytree(os.path.join(svj_py, "svj"), os.path.join(pkg, "svj-py", "svj"),
                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
for f in ("LICENSE",):                                   # Apache-2.0 of the SVJ repo
    p = os.path.join(svj_py, "..", f)
    if os.path.isfile(p): shutil.copy(p, os.path.join(pkg, "svj-py", "LICENSE-SVJ"))
zp = os.path.join(here, "dist", "svj_car_rig.zip")
with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
    for root, _, files in os.walk(pkg):
        for f in files:
            full = os.path.join(root, f)
            z.write(full, os.path.relpath(full, os.path.join(here, "dist")))
print("built", zp, round(os.path.getsize(zp) / 1e6, 2), "MB")
