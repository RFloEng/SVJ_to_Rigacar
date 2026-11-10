"""
Check a rig produced by svj_to_blender_rig.py against the SVJ loop-closure solver.

  blender -b rig.blend --python verify_rig_vs_svj.py -- car.svj.json [--svj-py PATH]

For several poses (tilted ground, steering, both) it reads the inputs the rig sees (wheel travel relative to the
body, steering angle), solves the same inputs with the SVJ solver and compares, in the chassis frame:
  * the position of every joint point of every suspension body,
  * the wheel centres, camber and toe,
  * the length of the spring / damper elements.
"""
import sys, os, math
import bpy
from mathutils import Vector, Matrix
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svj_suspension_kinematics as K

argv = sys.argv[sys.argv.index("--") + 1:]
svj = argv[0]
svj_py = argv[argv.index("--svj-py") + 1] if "--svj-py" in argv else None
kin = K.extract(svj, svj_py, log=lambda *_: None)
ls, doc, fr = K.import_svj(svj_py)[0], kin["doc"], kin["frame"]
arm = bpy.data.objects["Car Rig"]
ground = bpy.data.objects["Ground"]
CB = {"FL": "Ft.L", "FR": "Ft.R", "RL": "Bk.L", "RR": "Bk.R"}
stations = ls._station_map(doc)
steer = kin["steer"]


def chassis_inv(ae):
    b = ae.pose.bones["MCH-Body"]
    return (b.matrix @ arm.data.bones["MCH-Body"].matrix_local.inverted()).inverted()


def rig_state():
    bpy.context.view_layer.update()
    ae = arm.evaluated_get(bpy.context.evaluated_depsgraph_get())
    Pi = chassis_inv(ae)
    travel = {}
    for sid, c in kin["stations"].items():
        p = Pi @ ae.pose.bones[f"MCH-Kin.{CB[sid]}"].matrix.translation
        travel[sid] = p.z - c["center"][2]
    gs = ae.pose.bones["GroundSensor.Ft.L"].matrix_basis.to_euler()   # reference only
    ys = Matrix(ae.pose.bones["GroundSensor.Ft.L"].matrix).to_euler()
    return ae, Pi, travel


def out_of_range(travel):
    """inputs whose current value lies outside the range the solver tables cover (the rig extrapolates there)"""
    rng = {}
    for data in kin["bodies"].values():
        for inp, t in data["inputs"].items():
            rng[inp] = (t["x"][0], t["x"][-1])
    bad = []
    for inp, (lo, hi) in rng.items():
        if inp.startswith("travel_"): v = travel[inp[7:]]
        elif inp.startswith("heave_"): v = (travel[inp[6:8]] + travel[inp[8:]]) / 2
        elif inp.startswith("roll_"): v = (travel[inp[5:7]] - travel[inp[7:]]) / 2
        else: continue
        if not (lo - 1e-4 <= v <= hi + 1e-4): bad.append(inp)
    return bad


def body_point_err(ae, Pi, transforms):
    """returns {'front': (err, where), 'rear': (err, where)}"""
    res = {"front": (0.0, ""), "rear": (0.0, "")}
    for body, data in kin["bodies"].items():
        grp = "rear" if any(("RL" in i or "RR" in i) for i in data["inputs"]) else "front"
        worst, where = res[grp]
        bone = f"KIN.{body}." + [i for i in kin["input_order"] if i in data["inputs"]][-1]
        pb = ae.pose.bones[bone]
        M = pb.matrix @ arm.data.bones[bone].matrix_local.inverted()
        for p in data["pts"]:
            ref = Vector(p["p"])
            got = Pi @ (M @ ref)
            # expected: solver pose of the body acting on the SAE reference point
            R, t = transforms[body]
            sae = _to_sae(ref, kin["wb"])
            exp = Vector(fr.point(tuple(sum(R[i][j] * sae[j] for j in range(3)) + t[i] for i in range(3))))
            e = (got - exp).length
            if e > worst: worst, where = e, f"{body}:{p['joint']}"
        res[grp] = (worst, where)
    return res


def _to_sae(pb, wb):
    # inverse of Frame.point: b = LR*p + c0  ->  p = LR^T (b - c0)
    c0 = Vector((0.0, -wb, 0.0))
    v = pb - c0
    return (-v.y, -v.x, -v.z)


def steer_rack(delta):
    ys, rs = steer["yaw"], steer["rack"]
    if delta <= ys[0]: return rs[0]
    if delta >= ys[-1]: return rs[-1]
    for i in range(len(ys) - 1):
        if ys[i] <= delta <= ys[i + 1]:
            f = (delta - ys[i]) / (ys[i + 1] - ys[i]); return rs[i] + f * (rs[i + 1] - rs[i])


def case(name, tilt=(0, 0), steer_x=0.0, susp=(0, 0, 0)):
    ground.rotation_euler = (tilt[0], tilt[1], 0)
    arm.pose.bones["Steering"].location = (steer_x, 0, 0)
    arm.pose.bones["Suspension"].location = susp
    ae, Pi, travel = rig_state()
    # steering angle as the rig sees it (same reading as the driver variable)
    delta = ae.pose.bones["GroundSensor.Ft.L"].matrix_basis.to_euler().z      # placeholder, replaced below
    gsl = ae.pose.bones["GroundSensor.Ft.L"]
    par = gsl.parent
    local = (par.matrix.inverted() @ gsl.matrix) if par else gsl.matrix
    rest = (par.bone.matrix_local.inverted() @ gsl.bone.matrix_local) if par else gsl.bone.matrix_local
    delta = (rest.inverted() @ local).to_euler().z
    tr = {s: max(-lim["droop"], min(lim["bump"], travel[s])) if False else travel[s] for s, lim in kin["limits"].items()}
    jv = {steer["joint"]: steer_rack(delta)} if steer else {}
    r = ls.solve(doc, travel=tr, joint_values=jv)
    werr_g = body_point_err(ae, Pi, r["transforms"])
    oor = out_of_range(travel)
    werr, where = max(werr_g.values())
    # wheel centres / camber / toe
    cerr = 0.0; cam = [];
    for sid in K.CORNERS:
        up = ae.pose.bones["MCH-Wheel." + CB[sid]]
        got = Pi @ up.matrix.translation
        exp = Vector(fr.point(ls._wheel_centre(doc, stations[sid], r["transforms"])))
        cerr = max(cerr, (got - exp).length)
        # spin axis (bone X) in chassis frame -> camber / toe of the rig
        ax = (Pi.to_3x3() @ up.matrix.to_3x3() @ Vector((1, 0, 0))).normalized()
        side = 1 if kin["stations"][sid]["center"][0] > 0 else -1
        if ax.x * side < 0: ax = -ax
        cam.append((sid, math.degrees(math.asin(max(-1, min(1, ax.z)))), math.degrees(math.atan2(-ax.y, abs(ax.x))) * (1 if False else 1)))
        sol = ls.alignment(doc, sid, r["transforms"])
        cam[-1] = cam[-1] + (sol["camber_deg"], sol["toe_deg"])
    print(f"-- {name}: travel mm " + " ".join(f"{s}:{v*1000:+.1f}" for s, v in travel.items()) + f" | delta {math.degrees(delta):+.1f} deg | rack {1000*jv.get(steer['joint'],0) if steer else 0:+.1f} mm")
    print(f"   body points: front {werr_g['front'][0]*1000:.2f} mm, rear {werr_g['rear'][0]*1000:.2f} mm | wheel centres: max err {cerr*1000:.2f} mm | solver residual {r['residual']:.1e}"
          + (f" | OUTSIDE solved range {oor} (rig extrapolates)" if oor else ""))
    for s, c_rig, t_rig, c_sol, t_sol in cam:
        print(f"   {s}: camber rig {c_rig:+.2f} vs solver {-c_sol:+.2f} (solver sign: top-out +) | toe-ish rig {t_rig:+.2f} vs solver {t_sol:+.2f}")
    # springs
    lengths = ls._lengths(doc, r["transforms"])
    worst = 0.0
    for s in kin["springs"]:
        sb, tb = ae.pose.bones.get("SPR." + s["id"]), ae.pose.bones.get("TGT." + s["id"])
        if not sb or not tb: continue
        a = Pi @ sb.head; b = Pi @ tb.head
        rig_len = (a - b).length
        sol_len = lengths.get(s["id"])
        if sol_len is not None: worst = max(worst, abs(rig_len - sol_len))
    print(f"   springs/dampers: max length err {worst*1000:.2f} mm")


case("rest")
case("pitch 2.0deg", tilt=(math.radians(2.0), 0))
case("roll 2.0deg", tilt=(0, math.radians(2.0)))
case("steer x=+0.5", steer_x=0.5)
case("steer x=-0.8", steer_x=-0.8)
case("pitch+roll+steer", tilt=(math.radians(-1.5), math.radians(1.5)), steer_x=0.4)
case("body down (Suspension z=-0.4)", susp=(0, 0, -0.4))
case("body up (Suspension z=+0.4)", susp=(0, 0, 0.4))
case("body roll (Suspension x=+0.4)", susp=(0.4, 0, 0))
case("body pitch (Suspension y=+0.4)", susp=(0, 0.4, 0))
case("roll 6deg ground + body roll", tilt=(0, math.radians(6)), susp=(0.3, 0, 0))
case("everything: bump+roll+steer", tilt=(math.radians(3), math.radians(4)), steer_x=-0.5, susp=(0.35, 0.2, -0.4))
