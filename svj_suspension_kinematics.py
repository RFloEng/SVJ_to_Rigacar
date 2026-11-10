"""
Suspension kinematics extraction from an SVJ file (no Blender needed).

Uses the SVJ loop-closure solver (spec section 26, svj-py/svj/loopsolver.py) to solve every body of the
suspension mechanisms (uprights, wishbones, links, tie rods, rack, rigid axle...) for

  * the wheel travel of each station (bump positive, wheel centre rises relative to the chassis),
  * the steering rack position,

and returns, for every moving body, one table per input: input value -> rigid transform of the body,
expressed in the Blender rig frame (rear axle at y=0, car facing -Y, left = +X, Z up, ground z=0).

Legacy files (v0.95 `suspension` section, e.g. AC converters) are migrated first with svj.migrate_suspension.

The rig composes a body's tables as a chain, X = X_travel(...) . X_steer(...): exact when one input moves,
first-order when several move at once.
"""
import os, sys, json, math

DEFAULT_SVJ_PY = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "svj-py"),      # bundled with the add-on
    os.environ.get("SVJ_PY", ""),                                           # env var override
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "SVJ-standard-vehicle-json", "svj-py"),
]
CORNERS = ("FL", "FR", "RL", "RR")
EPS_MOVE = 1e-5
ACCEPT_RES = 2e-3       # m: least-squares solutions up to this residual are used (flagged in the report)

# SAE J670 (x fwd, y right, z down, origin front axle at ground)  ->  Blender rig frame
LR = ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))      # bx=-y, by=-x, bz=-z (proper rotation)


def _mv(M, v): return tuple(sum(M[i][j] * v[j] for j in range(3)) for i in range(3))
def _mm(A, B): return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))
def _tr(A): return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))
def _add(a, b): return tuple(x + y for x, y in zip(a, b))
def _sub(a, b): return tuple(x - y for x, y in zip(a, b))


def import_svj(svj_py=None):
    cands = ([svj_py] if svj_py else []) + DEFAULT_SVJ_PY
    for c in cands:
        if c and os.path.isdir(os.path.join(c, "svj")):
            if c not in sys.path: sys.path.insert(0, c)
            from svj import loopsolver as ls, constraints as cc, migrate_suspension as ms
            return ls, cc, ms
    raise SystemExit("svj-py not found; pass --svj-py <path to SVJ-standard-vehicle-json/repo/svj-py>")


class Frame:
    """SAE point/transform -> Blender, given the wheelbase."""
    def __init__(self, wb):
        self.c0 = (0.0, -wb, 0.0)
    def point(self, p):
        return _add(_mv(LR, p), self.c0)
    def rigid(self, X):
        """(R,t) acting on SAE points -> (Rb, tb) acting on Blender points."""
        R, t = X
        Rb = _mm(_mm(LR, R), _tr(LR))
        tb = _sub(_add(_mv(LR, t), self.c0), _mv(Rb, self.c0))
        return Rb, tb


def to_local(Xb, anchor):
    """Pose-bone basis for a bone at `anchor` (axis-aligned rest): M = T(-a) X T(a), returned as 4x4 rows."""
    Rb, tb = Xb
    t = _sub(_add(_mv(Rb, anchor), tb), anchor)
    return [list(Rb[0]) + [t[0]], list(Rb[1]) + [t[1]], list(Rb[2]) + [t[2]], [0, 0, 0, 1]]


def load_model(path, svj_py=None):
    ls, cc, ms = import_svj(svj_py)
    doc = json.load(open(path, encoding="utf-8"))
    notes = []
    if not doc.get("wheel_stations"):
        doc, notes = ms.migrate(doc)
    return ls, cc, doc, notes


def _moved(Xs, anchor):
    """max displacement (m) of the anchor point plus rotation angle (rad) over a list of (R,t)."""
    m = 0.0
    for R, t in Xs:
        d = _sub(_add(_mv(R, anchor), t), anchor)
        ang = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
        m = max(m, math.sqrt(sum(x * x for x in d)) + ang)
    return m


def add_lateral_location(ls, doc, stations, centres, log):
    """A carrier shared by two stations (solid axle) that the SVJ locates only with rods can be left free
    sideways (e.g. AC converters write the axle without a Panhard rod): the solver then picks an arbitrary
    minimum-norm yaw. Detect that and add a virtual, flagged Panhard rod at wheel-centre height."""
    groups = {}
    for sid in CORNERS:
        groups.setdefault(stations[sid].get("upright_body"), []).append(sid)
    added = []
    for body, sids in groups.items():
        if len(sids) < 2:
            continue
        r = ls.solve(doc, travel={sids[0]: 0.01})
        R, _ = r["transforms"][body]
        yaw = math.degrees(math.atan2(R[1][0], R[0][0]))
        if abs(yaw) < 0.5:
            continue
        cs = [centres[s] for s in sids]
        xc, zc = sum(c[0] for c in cs) / 2, sum(c[2] for c in cs) / 2
        half = abs(cs[0][1] - cs[1][1]) / 2
        pid = "panhard_" + body
        fa_pt, fb_pt = [xc, 0.6 * half, zc], [xc, -0.6 * half, zc]
        doc["chassis"].setdefault("mass_bodies", []).append(
            {"id": pid, "name": "Panhard rod (virtual)", "category": "structural", "mass": 0.0,
             "position": [xc, 0.0, zc], "x_derived": True, "x_synthetic": "lateral location added by svj_to_blender_rig"})
        for jid, ba, bb, p in ((pid + "_chassis", "chassis", pid, fa_pt), (pid + "_axle", pid, body, fb_pt)):
            fr_ = {"position": p, "orientation": [0.0, 0.0, 0.0, 1.0]}
            doc.setdefault("constraints", []).append(
                {"id": jid, "type": "joint", "joint_type": "spherical", "body_a": ba, "frame_a": dict(fr_),
                 "body_b": bb, "frame_b": dict(fr_), "x_derived": True})
        added.append((body, yaw))
        log(f"NOTE: carrier '{body}' is not located laterally in the SVJ (yaw {yaw:.1f} deg per 10 mm): virtual Panhard rod added")
    return added


def extract(path, svj_py=None, n_travel=7, log=print):
    ls, cc, doc, notes = load_model(path, svj_py)
    stations = ls._station_map(doc)
    missing = [c for c in CORNERS if c not in stations]
    if missing:
        raise SystemExit(f"SVJ has no wheel stations {missing}; only 4-wheel cars (FL FR RL RR) are supported")
    centres = {sid: ls._wheel_centre(doc, stations[sid], {}) for sid in CORNERS}
    if any(c is None for c in centres.values()):
        raise SystemExit("a wheel station has no wheel_center marker")
    panhard = add_lateral_location(ls, doc, stations, centres, log)
    wb = centres["FL"][0] - centres["RL"][0]
    fr = Frame(wb)
    out = dict(doc=doc, notes=notes, wb=wb, frame=fr, stations={}, bodies={}, springs=[], steer=None, report=[])
    for body, yaw in panhard:
        out["report"].append(f"ASSUMPTION: '{body}' had no lateral location in the SVJ; a virtual Panhard rod at wheel-centre height was added")

    # ---- station geometry (Blender frame)
    for sid in CORNERS:
        st = stations[sid]
        c = centres[sid]
        rad = (st.get("wheel") or {}).get("loaded_radius") or -c[2]
        out["stations"][sid] = dict(center=fr.point(c), radius=-c[2], loaded_radius=rad,
                                    upright=st.get("upright_body"), steering_joint=st.get("steering_joint"))
    out["half_track"] = {"F": abs(centres["FL"][1] - centres["FR"][1]) / 2, "R": abs(centres["RL"][1] - centres["RR"][1]) / 2}

    # ---- travel limits from bump / rebound stops
    cons = [c for c in doc.get("constraints", []) if isinstance(c, dict)]
    lim = {}
    for sid in CORNERS:
        lim[sid] = [0.08, 0.08, False]                      # droop, bump, from_svj
        for c in cons:
            if c.get("type") != "spring" or not str(c.get("id", "")).lower().endswith("_" + sid.lower()):
                continue
            fa, fb = c.get("frame_a", {}).get("position"), c.get("frame_b", {}).get("position")
            fl = c.get("free_length")
            if not (fa and fb and fl):
                continue
            dist = math.dist(fa, fb)
            if "bump_stop" in c["id"]:
                lim[sid][1] = min(max(dist - fl, 0.02), 0.25); lim[sid][2] = True
            elif "rebound_stop" in c["id"]:
                lim[sid][0] = min(max(fl - dist, 0.02), 0.25); lim[sid][2] = True
    out["limits"] = {s: dict(droop=v[0], bump=v[1], from_svj=v[2]) for s, v in lim.items()}
    if not all(v[2] for v in lim.values()):
        out["report"].append("NOTE: no bump/rebound stops in the SVJ for %s: wheel travel assumed +-80 mm"
                             % ",".join(s for s, v in lim.items() if not v[2]))

    # ---- mechanisms that carry a wheel
    uprights = {stations[s].get("upright_body") for s in CORNERS}
    mech = sorted({b for comp in ls._components(doc) if uprights & set(comp) for b in comp})

    # joint points per body (Blender frame) for anchors / proxy geometry
    pts = {b: [] for b in mech}
    for c in cc._constraints(doc, "joint"):
        for side, other in (("a", "b"), ("b", "a")):
            b = c.get("body_" + side)
            if b in pts:
                f = cc._frame(doc, c, side)
                if f is not None:
                    pts[b].append(dict(p=fr.point(f[0]), other=c.get("body_" + other), joint=c.get("id"), jt=c.get("joint_type")))

    anchors = {}
    for b in mech:
        st_of = [s for s in CORNERS if stations[s].get("upright_body") == b]
        if st_of:
            anchors[b] = fr.point(centres[st_of[0]])
        elif pts[b]:
            n = len(pts[b]); anchors[b] = tuple(sum(p["p"][i] for p in pts[b]) / n for i in range(3))
        else:
            anchors[b] = (0.0, 0.0, 0.0)

    # ---- sweeps
    tables = {}          # (body, input) -> list of (x, Xb)
    def record(inp, x, transforms):
        for b in mech:
            tables.setdefault((b, inp), []).append((x, fr.rigid(transforms[b])))
    ref = ls.solve(doc)
    if not ref["converged"]:
        log("WARN: reference pose does not close (residual %.2e)" % ref["residual"])

    worst = [0.0]

    def sweep_input(name, travel_of, lo, hi, label):
        """Solve outwards from 0 (each solve starts from its neighbour); stop at the first non-closing position."""
        xs = [-lo * k / n_travel for k in range(n_travel, 0, -1)] + [0.0] + [hi * k / n_travel for k in range(1, n_travel + 1)]
        sols = {0.0: ref["transforms"]}
        for direction in (1, -1):
            prev = ref["transforms"]
            for x in sorted((x for x in xs if x * direction > 0), key=abs):
                r = ls.solve(doc, travel=travel_of(x), start=prev)
                if r["residual"] > ACCEPT_RES:
                    log(f"WARN: {label} {x*1000:+.0f} mm did not converge (res {r['residual']*1000:.2f} mm); range truncated")
                    break
                worst[0] = max(worst[0], r["residual"] if not r["converged"] else 0.0)
                sols[x] = prev = r["transforms"]
        for x in sorted(sols):
            record(name, x, sols[x])
        return sols

    # two stations on one carrier (rigid axle): its pose depends on both wheels together, so the inputs
    # are heave (both wheels rise equally) and roll (one rises, the other falls), which hardly couple.
    groups = {}
    for sid in CORNERS:
        groups.setdefault(stations[sid].get("upright_body"), []).append(sid)
    pairs = [tuple(v) for v in groups.values() if len(v) == 2]
    out["pairs"] = pairs
    paired = {s for p in pairs for s in p}
    for sid in CORNERS:
        if sid in paired:
            continue
        sols = sweep_input("travel_" + sid, lambda x, sid=sid: {sid: x}, lim[sid][0], lim[sid][1], sid + " travel")
        out["report"].append(_station_report(ls, doc, sid, sols, centres))
    for a_, b_ in pairs:
        dr, bp = min(lim[a_][0], lim[b_][0]), min(lim[a_][1], lim[b_][1])
        sols = sweep_input(f"heave_{a_}{b_}", lambda x, a_=a_, b_=b_: {a_: x, b_: x}, dr, bp, f"{a_}{b_} heave")
        out["report"].append("heave " + _station_report(ls, doc, a_, sols, centres).replace(a_ + ":", f"{a_}{b_}:", 1))
        rr = min(dr, bp)
        sols = sweep_input(f"roll_{a_}{b_}", lambda x, a_=a_, b_=b_: {a_: x, b_: -x}, rr, rr, f"{a_}{b_} roll")
        out["report"].append("roll  " + _station_report(ls, doc, a_, sols, centres).replace(a_ + ":", f"{a_}(+) {b_}(-):", 1))

    if worst[0] > 0:
        out["report"].append("WARN: some travel positions only close approximately (over-constrained links): worst residual %.2f mm" % (worst[0] * 1000))

    # steering
    sj = next((stations[s].get("steering_joint") for s in CORNERS if stations[s].get("steering_joint")), None)
    if sj:
        steered = [s for s in CORNERS if stations[s].get("steering_joint") == sj]
        up = {s: stations[s]["upright_body"] for s in steered}
        def yaw_of(tr):
            ys = []
            for s in steered:
                Rb, _ = fr.rigid(tr[up[s]])
                ys.append(math.atan2(Rb[1][0], Rb[0][0]))
            return sum(ys) / len(ys)
        jr = next((c.get("range") for c in cons if c.get("id") == sj), None) or {}
        rmax = min(max(abs(jr.get("max", 0.12)), abs(jr.get("min", -0.12))), 0.2)
        samples = {0.0: (0.0, ref["transforms"])}
        step = rmax / 12
        for sign in (1, -1):
            prev = ref["transforms"]
            for k in range(1, 13):
                r_ = sign * k * step
                r = ls.solve(doc, joint_values={sj: r_}, start=prev)
                if r["residual"] > ACCEPT_RES:
                    log(f"WARN: rack {r_*1000:+.0f} mm did not converge; steering range truncated"); break
                y = yaw_of(r["transforms"])
                if abs(y) > math.radians(48): break
                samples[r_] = (y, r["transforms"]); prev = r["transforms"]
        seq = sorted(samples.items(), key=lambda kv: kv[1][0])
        dedup, last = [], None
        for rk, (y, tr) in seq:
            if last is None or y > last + 1e-6:
                dedup.append((rk, y, tr)); last = y
        for rk, y, tr in dedup:
            record("steer", y, tr)
        out["steer"] = dict(joint=sj, stations=steered, max_yaw=max(abs(d[1]) for d in dedup),
                            rack=[d[0] for d in dedup], yaw=[d[1] for d in dedup])
        out["report"].append("steering: rack %+.0f..%+.0f mm -> wheel yaw %+.1f..%+.1f deg"
                             % (dedup[0][0] * 1000, dedup[-1][0] * 1000, math.degrees(dedup[0][1]), math.degrees(dedup[-1][1])))
    else:
        out["report"].append("WARN: no steering joint on the front stations: the rig steers with the template's parallel steering only")

    # ---- bodies that move with each input
    order = (["travel_" + s for s in CORNERS if s not in paired]
             + [f"heave_{a_}{b_}" for a_, b_ in pairs] + [f"roll_{a_}{b_}" for a_, b_ in pairs] + ["steer"])
    out["input_order"] = order
    for b in mech:
        inputs = {}
        for inp in order:
            tab = tables.get((b, inp))
            if not tab:
                continue
            tab.sort(key=lambda p: p[0])
            if _moved([t for _, t in tab], anchors[b]) > EPS_MOVE:
                inputs[inp] = dict(x=[x for x, _ in tab], M=[to_local(t, anchors[b]) for _, t in tab])
        if inputs:
            out["bodies"][b] = dict(anchor=anchors[b], inputs=inputs, pts=pts[b])

    # ---- springs / dampers (linear, chassis -> carrier), Blender frame
    for c in cons:
        if c.get("type") not in ("spring", "damper") or c.get("kind") == "torsional":
            continue
        cid = str(c.get("id", ""))
        if "stop" in cid or c.get("spring_type") == "rubber":
            continue
        fa, fb = cc._frame(doc, c, "a"), cc._frame(doc, c, "b")
        if fa is None or fb is None:
            continue
        out["springs"].append(dict(id=cid, type=c["type"], a=fr.point(fa[0]), b=fr.point(fb[0]),
                                   body_a=c.get("body_a"), body_b=c.get("body_b")))
    return out


def _station_report(ls, doc, sid, sols, centres):
    xs = sorted(sols)
    def al(x): return ls.alignment(doc, sid, sols[x]) or {}
    a0, am, ap = al(0.0), al(xs[0]), al(xs[-1])
    def wc(x):
        c = ls._wheel_centre(doc, ls._station_map(doc)[sid], sols[x]); return c
    return ("%s: travel %+.0f..%+.0f mm | camber %.2f deg -> %.2f (droop) / %.2f (bump) | toe %.2f -> %.2f / %.2f | track change %+.1f / %+.1f mm"
            % (sid, xs[0] * 1000, xs[-1] * 1000, a0.get("camber_deg", 0), am.get("camber_deg", 0), ap.get("camber_deg", 0),
               a0.get("toe_deg", 0), am.get("toe_deg", 0), ap.get("toe_deg", 0),
               (wc(xs[0])[1] - wc(0.0)[1]) * 1000, (wc(xs[-1])[1] - wc(0.0)[1]) * 1000))


if __name__ == "__main__":
    import time
    t0 = time.time()
    k = extract(sys.argv[1])
    print("wheelbase", round(k["wb"], 4), "half_track", k["half_track"], "time %.1fs" % (time.time() - t0))
    for line in k["report"]: print(" ", line)
    for b, d in k["bodies"].items():
        print(" body", b, "inputs", {i: len(v["x"]) for i, v in d["inputs"].items()})
    print(" springs", [(s["id"], s["body_b"]) for s in k["springs"]])
    print(" limits", k["limits"])
    for n in k["notes"][:6]: print(" note", n)
