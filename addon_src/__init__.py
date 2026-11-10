bl_info = {
    "name": "SVJ Car Rig (Rigacar suspension)",
    "author": "SVJ converter",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "File > Import > SVJ car (rigged)   |   3D View > Sidebar (N) > SVJ Car",
    "description": "Imports an SVJ (Standard Vehicle JSON) car as a Rigacar-style rig whose suspension "
                   "kinematics are solved from the SVJ",
    "category": "Import-Export",
}

import os
import subprocess
import tempfile

import bpy
from bpy.props import StringProperty, BoolProperty
from bpy.types import AddonPreferences, Operator, Panel
from bpy_extras.io_utils import ImportHelper

from . import rigacar_template

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDER = os.path.join(HERE, "svj_to_blender_rig.py")
BUNDLED_SVJ_PY = os.path.join(HERE, "svj-py")


class SVJ_Prefs(AddonPreferences):
    bl_idname = __package__

    svj_py: StringProperty(
        name="svj-py folder (optional)",
        description="Folder 'svj-py' of the SVJ-standard-vehicle-json repo. Leave empty to use the copy bundled with the add-on",
        subtype='DIR_PATH', default="")

    rigacar_path: StringProperty(
        name="Rigacar (optional)",
        description="Folder or release .zip of the Rigacar add-on. Leave empty to use ~/.svj_car_rig, $RIGACAR or an installed Rigacar",
        subtype='FILE_PATH', default="")

    def draw(self, context):
        self.layout.prop(self, "svj_py")
        self.layout.prop(self, "rigacar_path")
        found = rigacar_template.locate(bpy.path.abspath(self.rigacar_path) or None)
        row = self.layout.row()
        row.label(text="Rigacar: " + (found if found else "NOT FOUND (needed to generate the rig)"))
        row.operator("svj_car_rig.download_rigacar", icon='IMPORT')
        self.layout.label(text="Bundled solver: " + ("found" if os.path.isdir(BUNDLED_SVJ_PY) else "MISSING"))


def _svj_py(context):
    p = bpy.path.abspath(context.preferences.addons[__package__].preferences.svj_py).strip()
    return p if p else BUNDLED_SVJ_PY


def _rigacar(context):
    p = bpy.path.abspath(context.preferences.addons[__package__].preferences.rigacar_path).strip()
    return rigacar_template.locate(p or None)


def _build(svj, mesh, with_proxy, with_susp_proxy, svj_py, rigacar, out):
    """Run the converter in a background Blender (it re-opens the template file) and return (ok, log)."""
    cmd = [bpy.app.binary_path, "-b", "--factory-startup", "--python", BUILDER, "--",
           svj, "--out", out, "--rigacar", rigacar, "--svj-py", svj_py]
    if mesh: cmd += ["--mesh", mesh]
    if not with_proxy: cmd += ["--no-proxy"]
    if not with_susp_proxy: cmd += ["--no-susp-proxy"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    return (r.returncode == 0 and os.path.isfile(out)), (r.stdout or "") + (r.stderr or "")


def _append_all(path, name):
    """Append every object / collection of the generated file into the current scene, grouped in a new collection."""
    with bpy.data.libraries.load(path, link=False) as (src, dst):
        dst.objects = list(src.objects)
        dst.collections = list(src.collections)
    root = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(root)
    in_col = set()
    for c in dst.collections:
        for o in c.all_objects: in_col.add(o)
    for c in dst.collections:
        root.children.link(c)
    for o in dst.objects:
        if o is not None and o not in in_col:
            root.objects.link(o)
    rig = next((o for o in dst.objects if o is not None and o.type == 'ARMATURE'), None)
    return root, rig


class IMPORT_OT_svj_car_rig(Operator, ImportHelper):
    """Import an SVJ car as a rigged Blender car with SVJ-solved suspension"""
    bl_idname = "import_scene.svj_car_rig"
    bl_label = "Import SVJ car (rigged)"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.svj.json;*.json", options={'HIDDEN'})
    mesh_path: StringProperty(
        name="Car mesh (glb/fbx/obj)", subtype='FILE_PATH', default="",
        description="Optional. Leave empty to auto-detect the .glb next to the SVJ")
    auto_mesh: BoolProperty(name="Auto-detect .glb next to the SVJ", default=True)
    body_proxy: BoolProperty(name="Proxy body/wheels (if no mesh)", default=True)
    susp_proxy: BoolProperty(name="Suspension proxy (arms, links, springs)", default=True)

    def execute(self, context):
        svj = self.filepath
        if not os.path.isfile(svj):
            self.report({'ERROR'}, "SVJ file not found"); return {'CANCELLED'}
        mesh = bpy.path.abspath(self.mesh_path) if self.mesh_path else ""
        if not mesh and self.auto_mesh:
            d = os.path.dirname(svj)
            glbs = sorted(f for f in os.listdir(d) if f.lower().endswith((".glb", ".gltf")))
            if glbs: mesh = os.path.join(d, glbs[0])
        rigacar = _rigacar(context)
        if not rigacar:
            self.report({'ERROR'}, "Rigacar not found: install it or use the Download button in the add-on preferences")
            return {'CANCELLED'}
        tmp = os.path.join(tempfile.gettempdir(), "svj_car_rig_build.blend")
        if os.path.isfile(tmp): os.remove(tmp)
        context.window.cursor_set('WAIT')
        try:
            ok, log = _build(svj, mesh, self.body_proxy, self.susp_proxy, _svj_py(context), rigacar, tmp)
        finally:
            context.window.cursor_set('DEFAULT')
        txt = bpy.data.texts.new("SVJ import log"); txt.write(log)
        if not ok:
            self.report({'ERROR'}, "Conversion failed - see Text 'SVJ import log'"); return {'CANCELLED'}
        name = os.path.basename(svj).split(".")[0]
        root, rig = _append_all(tmp, "SVJ " + name)
        if rig is not None:
            for o in context.selected_objects: o.select_set(False)
            context.view_layer.objects.active = rig
            rig.select_set(True)
        warns = [l for l in log.splitlines() if "WARN" in l]
        self.report({'WARNING' if warns else 'INFO'},
                    f"Imported {name}" + (f" ({len(warns)} warnings, see Text 'SVJ import log')" if warns else ""))
        return {'FINISHED'}


class SVJ_OT_download_rigacar(Operator):
    """Download the official Rigacar 7.1 release from GitHub (github.com/digicreatures/rigacar, GPL-3.0)"""
    bl_idname = "svj_car_rig.download_rigacar"
    bl_label = "Download Rigacar 7.1"

    def execute(self, context):
        try:
            zp = rigacar_template.download()
        except Exception as e:
            self.report({'ERROR'}, f"Download failed: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, "Rigacar downloaded to " + zp)
        return {'FINISHED'}


class VIEW3D_OT_svj_pose_mode(Operator):
    """Select the car rig and enter Pose Mode"""
    bl_idname = "svj_car_rig.pose_mode"
    bl_label = "Enter Pose Mode"

    def execute(self, context):
        rig = context.object if (context.object and context.object.type == 'ARMATURE' and "svj_source" in context.object) else None
        rig = rig or next((o for o in context.scene.objects if o.type == 'ARMATURE' and "svj_source" in o), None)
        if rig is None:
            self.report({'ERROR'}, "No SVJ car rig in the scene"); return {'CANCELLED'}
        if context.object and context.object.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
        context.view_layer.objects.active = rig
        rig.select_set(True)
        bpy.ops.object.mode_set(mode='POSE')
        return {'FINISHED'}


class VIEW3D_PT_svj_car(Panel):
    bl_label = "SVJ Car"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "SVJ Car"

    def draw(self, context):
        l = self.layout
        l.operator("import_scene.svj_car_rig", icon='IMPORT')
        rig = next((o for o in context.scene.objects if o.type == 'ARMATURE' and "svj_source" in o), None)
        if rig is None:
            l.label(text="No SVJ car in the scene"); return
        l.label(text=f"{rig['svj_source']}  (wheelbase {rig['svj_wheelbase']:.2f} m)")
        l.operator("svj_car_rig.pose_mode", icon='POSE_HLT')
        if context.mode == 'POSE' and context.object == rig:
            b = rig.pose.bones
            box = l.box(); box.label(text="Controls")
            for bn, lab in (("Steering", "Steering rack (X)"), ("Suspension", "Body heave/roll/pitch")):
                if bn in b:
                    col = box.column(align=True); col.label(text=lab)
                    col.prop(b[bn], "location", text="")
        if "svj_kinematics_report" in rig:
            if l.operator("svj_car_rig.show_report", icon='TEXT'): pass


class VIEW3D_OT_svj_report(Operator):
    """Open the kinematics report (assumptions / warnings) in a text block"""
    bl_idname = "svj_car_rig.show_report"
    bl_label = "Kinematics report"

    def execute(self, context):
        rig = next((o for o in context.scene.objects if o.type == 'ARMATURE' and "svj_kinematics_report" in o), None)
        if rig is None: return {'CANCELLED'}
        t = bpy.data.texts.get("SVJ kinematics report") or bpy.data.texts.new("SVJ kinematics report")
        t.clear(); t.write(rig["svj_kinematics_report"])
        self.report({'INFO'}, "Report in the Text editor: 'SVJ kinematics report'")
        return {'FINISHED'}


def menu_import(self, context):
    self.layout.operator(IMPORT_OT_svj_car_rig.bl_idname, text="SVJ car (rigged) (.svj.json)")


classes = (SVJ_Prefs, SVJ_OT_download_rigacar, IMPORT_OT_svj_car_rig, VIEW3D_OT_svj_pose_mode, VIEW3D_OT_svj_report, VIEW3D_PT_svj_car)


def register():
    for c in classes: bpy.utils.register_class(c)
    bpy.types.TOPBAR_MT_file_import.append(menu_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_import)
    for c in reversed(classes): bpy.utils.unregister_class(c)
