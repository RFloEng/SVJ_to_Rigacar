"""
SVJ -> Rigacar-style Blender car rig, with the suspension kinematics taken from the SVJ.

Usage (Blender 4.x/5.x, background):
  blender -b --python svj_to_blender_rig.py -- car.svj.json [options]

Options:
  --rigacar PATH    Rigacar add-on folder / release zip used to generate the rig template
                    (default: $RIGACAR, ~/.svj_car_rig, installed add-ons); --download-rigacar fetches v7.1
  --template PATH   use a ready-made rig template .blend instead
  --out PATH        output .blend (default: <svj name>.rig.blend next to the SVJ)
  --mesh PATH       optional car mesh (.glb/.gltf/.fbx/.obj) to import and bind (see bind_mesh)
  --no-proxy        do not generate the proxy body / wheels (when no --mesh)
  --no-susp-proxy   do not generate the proxy wishbones / links / springs
  --svj-py PATH     svj-py folder of the SVJ-standard-vehicle-json repo (loop-closure solver)

What it does
  1. Reads the wheel stations of the SVJ (legacy `suspension` sections are migrated with svj.migrate_suspension):
       wheel centres -> wheelbase, front/rear track, wheel radius. Re-fits the template armature to them.
  2. Solves the SVJ suspension mechanism with the SVJ loop-closure solver (spec section 26) for the wheel
     travel of every wheel (between the SVJ bump / rebound stops) and for the steering rack, and bakes
     the pose of every body (upright, wishbones, links, tie rods, rack, rigid axle) into driver tables.
  3. In the rig, `MCH-Kin.<corner>` measures each wheel's travel relative to the body, the front wheels'
     steering comes from the rack tables (so toe, caster and Ackermann are the SVJ's), and
     `KIN.<body>.<input>` bones, driven by those tables, move every body of the mechanism. The wheels
     hang from the solved uprights: camber, toe, track change, recession and steer all follow the SVJ.
  4. Springs / dampers of the SVJ become stretch-to bones between their SVJ mounts.
  5. Geometry-dependent constraints (shrinkwrap distance, axle limit distance, Child-Of inverses) are re-set.

Blender frame of the result: car faces -Y (Rigacar convention), left = +X, Z up,
rear axle at y=0, front axle at y=-wheelbase, ground at z=0.
"""
import sys, os, json, math, argparse
import bpy, bmesh
from mathutils import Vector, Matrix, Euler

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svj_suspension_kinematics as K
import rigacar_template

# nominal geometry of the template rig
T_WB, T_HALF, T_R = 4.0, 0.9, 0.5
CORNER_BONE = {"FL": "Ft.L", "FR": "Ft.R", "RL": "Bk.L", "RR": "Bk.R"}

# bones whose tail keeps the template length scaled by the wheel-size ratio
# (so widgets keep their proportions); every other bone has both endpoints remapped.
LOCAL_PREFIXES = ("Wheel.", "WheelBrake.", "WheelDamper.", "GroundSensor.", "SHP-GroundSensor.",
                  "MCH-Wheel.", "DEF-Wheel.")


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    p = argparse.ArgumentParser()
    p.add_argument("svj")
    p.add_argument("--template", help="use this rig template .blend instead of generating one with Rigacar")
    p.add_argument("--rigacar", help="Rigacar add-on folder or release zip (default: $RIGACAR, ~/.svj_car_rig, installed add-ons)")
    p.add_argument("--download-rigacar", action="store_true", help="download Rigacar 7.1 from its GitHub release")
    p.add_argument("--out")
    p.add_argument("--mesh")
    p.add_argument("--no-proxy", action="store_true")
    p.add_argument("--no-susp-proxy", action="store_true")
    p.add_argument("--svj-py")
    return p.parse_args(argv)


# ----------------------------------------------------------------------------- geometry from the SVJ stations
def geometry(kin):
    st, doc = kin["stations"], kin["doc"]
    sets = (doc.get("tires") or {}).get("sets", {}) or {}
    def width(side):
        for k, v in sets.items():
            if side in k.lower():
                return (v.get("dimensions") or {}).get("section_width") or 0.2
        for v in sets.values():
            return (v.get("dimensions") or {}).get("section_width") or 0.2
        return 0.2
    cog = (doc.get("chassis") or {}).get("center_of_gravity")
    return dict(wb=kin["wb"], hf=kin["half_track"]["F"], hr=kin["half_track"]["R"],
                rf=(st["FL"]["radius"] + st["FR"]["radius"]) / 2, rr=(st["RL"]["radius"] + st["RR"]["radius"]) / 2,
                wf=width("front"), wr=width("rear"),
                cog=Vector(kin["frame"].point(cog)) if cog else None,
                name=os.path.basename(doc.get("_src", "car")).split(".")[0])


# ----------------------------------------------------------------------------- rig fitting
def make_mapper(g):
    sy = g["wb"] / T_WB
    def t_of(y): return min(max(-y / T_WB, 0.0), 1.0)
    def lerp(a, b, t): return a + (b - a) * t
    def kz(y): return lerp(g["rr"], g["rf"], t_of(y)) / T_R
    def half(y): return lerp(g["hr"], g["hf"], t_of(y))
    def point(v):
        y = v.y * sy
        k = kz(v.y)
        ax = abs(v.x)
        if ax <= T_HALF + 1e-6:
            x = v.x * half(v.y) / T_HALF
        else:
            x = math.copysign(half(v.y) + (ax - T_HALF) * k, v.x)
        return Vector((x, y, v.z * k))
    return point, kz


def fit_armature(arm, g):
    point, kz = make_mapper(g)
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    eb = arm.data.edit_bones
    new = {}
    for b in eb:
        h, t = b.head.copy(), b.tail.copy()
        nh = point(h)
        nt = nh + (t - h) * kz(h.y) if b.name.startswith(LOCAL_PREFIXES) else point(t)
        new[b.name] = (nh, nt)
    for n in new:
        eb[n].use_connect = False
    for n, (h, t) in new.items():
        eb[n].head, eb[n].tail = h, t
    bpy.ops.object.mode_set(mode="OBJECT")


def make_ground():
    mesh = bpy.data.meshes.new("Ground")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=500.0)
    bm.to_mesh(mesh); bm.free()
    ground = bpy.data.objects.new("Ground", mesh)
    bpy.context.scene.collection.objects.link(ground)
    return ground


def fix_constraints(arm, g, ground):
    for pb in arm.pose.bones:
        r = g["rr"] if ".Bk" in pb.name else g["rf"]
        for c in pb.constraints:
            if c.type == "SHRINKWRAP":
                c.target = ground; c.distance = r
            elif c.type == "LIMIT_DISTANCE" and c.subtarget == "GroundSensor.Axle.Bk":
                c.distance = g["wb"]
    bpy.context.view_layer.update()
    refresh_child_of(arm)
    try:
        arm["conform_object"] = ground
    except Exception as e:
        print("WARN conform_object:", e)


def refresh_child_of(arm):
    """Child Of inverse = inverse of the parent's rest matrix (what the template stores)."""
    for pb in arm.pose.bones:
        for c in pb.constraints:
            if c.type == "CHILD_OF" and c.subtarget:
                c.inverse_matrix = (arm.matrix_world @ arm.data.bones[c.subtarget].matrix_local).inverted()


# ----------------------------------------------------------------------------- SVJ suspension -> rig
def _euler_table(xs, Ms):
    """Per-sample translation + Euler XYZ (continuous, starting from the sample at x=0)."""
    i0 = min(range(len(xs)), key=lambda i: abs(xs[i]))
    locs, eul = [None] * len(xs), [None] * len(xs)
    for rng in (range(i0, len(xs)), range(i0, -1, -1)):
        prev = Euler((0, 0, 0), "XYZ")
        for i in rng:
            m = Matrix(Ms[i])
            e = m.to_euler("XYZ", prev); prev = e
            eul[i] = e; locs[i] = m.translation.copy()
    return locs, eul


def input_spec(inp):
    """Rig-side reading of a solver input -> (driver variables [(bone, channel)], expression, x scale, extrapolation).
    F-curve keys closer than 0.01 along X are merged by Blender, so the X axis is rescaled:
    metres -> mm for wheel travel, radians -> degrees for steering (simple driver expressions, no Python)."""
    kin_bone = lambda sid: ("MCH-Kin." + CORNER_BONE[sid], "LOC_Z")
    if inp.startswith("travel_"):
        return [kin_bone(inp[7:])], "a*1000", 1000.0, "LINEAR"
    if inp.startswith(("heave_", "roll_")):
        kind, ids = inp.split("_")
        a, b = kin_bone(ids[:2]), kin_bone(ids[2:])
        return [a, b], ("(a+b)*500" if kind == "heave" else "(a-b)*500"), 1000.0, "LINEAR"
    if inp == "steer":
        return [("GroundSensor.Ft.L", "ROT_Z")], f"a*{math.degrees(1.0):.6f}", math.degrees(1.0), "CONSTANT"
    raise ValueError(inp)


def _add_table_driver(arm, bone, prop, idx, xs, ys, inp):
    vars_, expr, scale, extrapolation = input_spec(inp)
    fc = arm.driver_add(f'pose.bones["{bone}"].{prop}', idx)
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    d = fc.driver
    d.type = "SCRIPTED"
    d.expression = expr
    for name, (var_bone, ttype) in zip("ab", vars_):
        v = d.variables.new(); v.name = name; v.type = "TRANSFORMS"
        t = v.targets[0]
        t.id = arm; t.bone_target = var_bone; t.transform_type = ttype; t.transform_space = "LOCAL_SPACE"
    while len(fc.keyframe_points):                # driver_add() creates default keys (0,0) and (1,1)
        fc.keyframe_points.remove(fc.keyframe_points[0])
    for x, y in zip(xs, ys):                      # insert() keeps the points sorted (add()+co mixes them up)
        k = fc.keyframe_points.insert(x * scale, y, options={"FAST"})
        k.interpolation = "BEZIER"
    fc.extrapolation = extrapolation
    fc.update()
    assert len(fc.keyframe_points) == len(xs), f"{bone}.{prop}[{idx}]: {len(fc.keyframe_points)} keys for {len(xs)} samples"


def build_suspension(arm, kin, g, make_springs=True):
    st = kin["stations"]
    chassis = "MCH-Body"
    chains = {}                         # body -> [bone names] (chassis -> ... -> last)
    kcoll = arm.data.collections.new("SVJ suspension")

    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    eb = arm.data.edit_bones

    def new_bone(name, head, tail, parent, deform=False):
        b = eb.new(name)
        b.head, b.tail = Vector(head), Vector(tail)
        b.parent = eb[parent]; b.use_connect = False; b.use_deform = deform
        kcoll.assign(b)
        return b

    stub = lambda p, L=0.05: (p[0], p[1] + L, p[2])

    for sid, c in st.items():
        new_bone(f"MCH-Kin.{CORNER_BONE[sid]}", c["center"], stub(c["center"]), chassis)

    order = kin["input_order"]
    for body, data in kin["bodies"].items():
        parent, names = chassis, []
        for inp in order:
            if inp in data["inputs"]:
                nm = f"KIN.{body}.{inp}"
                new_bone(nm, data["anchor"], stub(data["anchor"]), parent)
                parent = nm; names.append(nm)
        eb[names[-1]].use_deform = True
        chains[body] = names

    # front stations without a steering joint: parallel steer about the hub, driven by the rig's steering angle
    fallback = {}
    if kin["steer"] is None:
        for sid in ("FL", "FR"):
            up = st[sid]["upright"]
            parent = chains[up][-1] if up in chains else chassis
            nm = f"KIN.hub-steer.{CORNER_BONE[sid]}"
            new_bone(nm, st[sid]["center"], stub(st[sid]["center"]), parent)
            fallback[sid] = nm

    # wheels hang from the solved carriers
    for sid, c in st.items():
        wheel = eb["MCH-Wheel." + CORNER_BONE[sid]]
        if sid in fallback:
            wheel.parent = eb[fallback[sid]]
        elif c["upright"] in chains:
            wheel.parent = eb[chains[c["upright"]][-1]]
        else:
            print(f"WARN: carrier '{c['upright']}' of {sid} does not move with travel; wheel keeps its template parent")

    # springs / dampers
    spr_bones = []
    if make_springs:
        for s in kin["springs"]:
            a, b = Vector(s["a"]), Vector(s["b"])
            if (a - b).length < 1e-3 or s["body_a"] != "chassis":
                continue
            tgt_parent = chains[s["body_b"]][-1] if s["body_b"] in chains else chassis
            new_bone(f"SPR.{s['id']}", a, b, chassis, deform=True)
            new_bone(f"TGT.{s['id']}", b, stub(b, 0.03), tgt_parent)
            spr_bones.append(s)
    bpy.ops.object.mode_set(mode="POSE")

    # ---- wheel-travel sensors
    for sid in st:
        pb = arm.pose.bones[f"MCH-Kin.{CORNER_BONE[sid]}"]
        c = pb.constraints.new("COPY_LOCATION")
        c.target = arm; c.subtarget = "GroundSensor." + CORNER_BONE[sid]
        c.target_space = "WORLD"; c.owner_space = "WORLD"

    # ---- drivers from the solver tables
    n_drv = 0
    for body, data in kin["bodies"].items():
        for inp, bone in zip([i for i in order if i in data["inputs"]], chains[body]):
            tab = data["inputs"][inp]
            locs, eul = _euler_table(tab["x"], tab["M"])
            pb = arm.pose.bones[bone]
            pb.rotation_mode = "XYZ"
            for i in range(3):
                ys = [l[i] for l in locs]
                if max(abs(y) for y in ys) > 1e-7:
                    _add_table_driver(arm, bone, "location", i, tab["x"], ys, inp); n_drv += 1
                ys = [e[i] for e in eul]
                if max(abs(y) for y in ys) > 1e-7:
                    _add_table_driver(arm, bone, "rotation_euler", i, tab["x"], ys, inp); n_drv += 1
    for sid, nm in fallback.items():          # parallel steering: hub yaw = rig steering angle (1 rad -> 1 rad)
        pb = arm.pose.bones[nm]; pb.rotation_mode = "XYZ"
        _add_table_driver(arm, nm, "rotation_euler", 2, [-1.0, 1.0], [-1.0, 1.0], "steer"); n_drv += 1

    for s in spr_bones:
        pb = arm.pose.bones[f"SPR.{s['id']}"]
        c = pb.constraints.new("STRETCH_TO")
        c.target = arm; c.subtarget = f"TGT.{s['id']}"
        c.volume = "NO_VOLUME"; c.rest_length = pb.bone.length
    kcoll.is_visible = False
    bpy.ops.object.mode_set(mode="OBJECT")
    refresh_child_of(arm)               # MCH-Wheel.rotation.* etc. unaffected, but keep inverses exact
    print(f"suspension: {len(chains)} solved bodies, {n_drv} driver channels, {len(spr_bones)} spring/damper bones")
    return chains, spr_bones


# ----------------------------------------------------------------------------- meshes
def bind(obj, arm, bone):
    vg = obj.vertex_groups.new(name=bone)
    vg.add(range(len(obj.data.vertices)), 1.0, "REPLACE")
    m = obj.modifiers.new("Armature", "ARMATURE"); m.object = arm
    obj.parent = arm


def _tubes(name, segments, radius, arm, bone, coll):
    bm = bmesh.new()
    for p0, p1 in segments:
        p0, p1 = Vector(p0), Vector(p1)
        d = p1 - p0
        if d.length < 1e-4:
            continue
        res = bmesh.ops.create_cone(bm, cap_ends=True, segments=8, radius1=radius, radius2=radius, depth=d.length)
        q = Vector((0, 0, 1)).rotation_difference(d.normalized())
        bmesh.ops.transform(bm, matrix=Matrix.Translation((p0 + p1) / 2) @ q.to_matrix().to_4x4(), verts=res["verts"])
    if not bm.verts:
        bm.free(); return None
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    o = bpy.data.objects.new(name, me); coll.objects.link(o)
    bind(o, arm, bone)
    return o


def susp_proxy(arm, kin, chains, spr_bones):
    coll = bpy.data.collections.new("Suspension proxy")
    bpy.context.scene.collection.children.link(coll)
    coll.hide_render = True            # visible in the viewport to inspect the linkage, not in renders
    st = kin["stations"]
    carrier_of = {}
    for sid, c in st.items():
        carrier_of.setdefault(c["upright"], []).append(sid)
    uprights = set(carrier_of)
    n = 0
    for body, data in kin["bodies"].items():
        pts, seen = [], []
        for p in data["pts"]:
            v = Vector(p["p"])
            if all((v - s).length > 1e-5 for s in seen):
                seen.append(v); pts.append((v, p["other"]))
        segs = []
        if body in carrier_of and len(carrier_of[body]) >= 2:          # rigid axle: tube between the wheel centres
            cs = [Vector(st[s]["center"]) for s in carrier_of[body]]
            segs.append((cs[0], cs[1]))
            segs += [(min(cs, key=lambda c: (c - v).length), v) for v, _ in pts]
        elif body in carrier_of:                                       # upright: star from the wheel centre
            c = Vector(st[carrier_of[body][0]]["center"])
            segs = [(c, v) for v, _ in pts]
        elif len(pts) == 2:
            segs = [(pts[0][0], pts[1][0])]
        else:
            out = [v for v, o in pts if o in uprights]
            apex = out[0] if out else sum((v for v, _ in pts), Vector()) / max(len(pts), 1)
            segs = [(apex, v) for v, _ in pts]
        if _tubes("PROXY-" + body, segs, 0.012, arm, chains[body][-1], coll):
            n += 1
    for s in spr_bones:
        r = 0.022 if s["type"] == "spring" else 0.012
        if _tubes("PROXY-" + s["id"], [(s["a"], s["b"])], r, arm, f"SPR.{s['id']}", coll):
            n += 1
    print(f"suspension proxy: {n} objects")


def proxy(arm, g):
    wb = g["wb"]
    def cyl(name, centre, r, w, bone):
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=True, segments=32, radius1=r, radius2=r, depth=w)
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(math.pi / 2, 3, "Y"))
        bmesh.ops.translate(bm, verts=bm.verts, vec=centre)
        me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
        o = bpy.data.objects.new(name, me); bpy.context.scene.collection.objects.link(o)
        bind(o, arm, bone)
    for corner, side, y, half, r, w in (("Ft.L", 1, -wb, g["hf"], g["rf"], g["wf"]), ("Ft.R", -1, -wb, g["hf"], g["rf"], g["wf"]),
                                         ("Bk.L", 1, 0, g["hr"], g["rr"], g["wr"]), ("Bk.R", -1, 0, g["hr"], g["rr"], g["wr"])):
        cyl("PROXY-Wheel." + corner, Vector((side * half, y, r)), r, w, "DEF-Wheel." + corner)
    length = wb * 1.65
    width = max(g["hf"], g["hr"]) * 2 + 0.1
    z0, z1 = 0.15, max(1.0, 4.0 * g["rf"])
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector((v.co.x * width, v.co.y * length, z0 + (v.co.z + 0.5) * (z1 - z0)))
        v.co.y += -wb / 2
    me = bpy.data.meshes.new("PROXY-Body"); bm.to_mesh(me); bm.free()
    o = bpy.data.objects.new("PROXY-Body", me); bpy.context.scene.collection.objects.link(o)
    bind(o, arm, "DEF-Body")


WHEEL_NODE = {"wheel_fl": "Ft.L", "wheel_fr": "Ft.R", "wheel_rl": "Bk.L", "wheel_rr": "Bk.R"}


def bind_mesh(arm, path, g):
    """Import an SVJ glb (AC-derived: nodes 'SVJ::wheel::wheel_fl|fr|rl|rr' and 'SVJ::body::chassis',
    car facing +Y, front axle at y=0, ground z=0) and bind every mesh to a DEF- bone:
    meshes under a wheel node -> DEF-Wheel.<corner>, everything else -> DEF-Body.
    The import is converted to the rig frame (rotate 180 deg about Z, front axle to y=-wheelbase)."""
    ext = os.path.splitext(path)[1].lower()
    before = set(bpy.data.objects)
    if ext in (".glb", ".gltf"): bpy.ops.import_scene.gltf(filepath=path)
    elif ext == ".fbx": bpy.ops.import_scene.fbx(filepath=path)
    elif ext == ".obj": bpy.ops.wm.obj_import(filepath=path)
    else: raise SystemExit("unsupported mesh format " + ext)
    new = [o for o in bpy.data.objects if o not in before]
    M = Matrix.Translation((0, -g["wb"], 0)) @ Matrix.Rotation(math.pi, 4, "Z")
    for o in new:
        if o.parent is None: o.matrix_world = M @ o.matrix_world
    bpy.context.view_layer.update()
    n_w = n_b = 0
    for o in [o for o in new if o.type == "MESH"]:
        bone = "DEF-Body"
        p = o
        while p is not None:
            key = p.name.split("::")[-1].lower()
            if key in WHEEL_NODE: bone = "DEF-Wheel." + WHEEL_NODE[key]; break
            p = p.parent
        mw = o.matrix_world.copy(); o.parent = None; o.matrix_world = mw
        bind(o, arm, bone)
        if bone == "DEF-Body": n_b += 1
        else: n_w += 1
    for o in [o for o in new if o.type != "MESH" and o.name in bpy.data.objects]:
        bpy.data.objects.remove(o, do_unlink=True)
    print(f"bound {n_b} body meshes, {n_w} wheel meshes")


# ----------------------------------------------------------------------------- main
def main():
    a = parse_args()
    print("== solving SVJ suspension kinematics ==")
    kin = K.extract(a.svj, a.svj_py)
    kin["doc"]["_src"] = a.svj
    for line in kin["report"]:
        print("  ", line)
    g = geometry(kin)
    print("GEOMETRY", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in g.items()})

    template = a.template
    if not template:
        rc = a.rigacar
        if a.download_rigacar and not rigacar_template.locate(rc):
            print("downloading Rigacar from", rigacar_template.RELEASE_URL)
            rc = rigacar_template.download()
        print("== generating the rig template with Rigacar ==")
        template = rigacar_template.build(rc)
    bpy.ops.wm.open_mainfile(filepath=template)
    arm = bpy.data.objects["Car Rig"]
    fit_armature(arm, g)
    ground = make_ground()
    fix_constraints(arm, g, ground)
    for sid, c in kin["stations"].items():           # the rig's wheel centres must be the SVJ's
        h = arm.data.bones["GroundSensor." + CORNER_BONE[sid]].head_local
        err = (h - Vector(c["center"])).length
        if err > 1e-3:
            print(f"WARN: {sid} sensor is {err*1000:.1f} mm from the SVJ wheel centre")
    chains, spr = build_suspension(arm, kin, g)
    arm["svj_source"] = os.path.basename(a.svj)
    arm["svj_wheelbase"] = g["wb"]
    if g["cog"]: arm["svj_cog_blender"] = list(g["cog"])
    arm["svj_kinematics_report"] = "\n".join(kin["report"])
    if a.mesh: bind_mesh(arm, a.mesh, g)
    elif not a.no_proxy: proxy(arm, g)
    if not a.no_susp_proxy: susp_proxy(arm, kin, chains, spr)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.svj)), os.path.basename(a.svj).split(".")[0] + ".rig.blend")
    bpy.ops.wm.save_as_mainfile(filepath=out)
    print("SAVED", out)


main()
