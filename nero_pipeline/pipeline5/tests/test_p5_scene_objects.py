#!/usr/bin/env python3
"""Offline tests for step 7: the object in hand as slice boxes, the wrist camera box (geometry, in the grasp and place checks), MTC's contact order.
Run: /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_scene_objects.py"""
import math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.join(HERE, "..")
sys.path[:0] = [P5, os.path.join(P5, "core"), HERE]
import p5_scene_objects as SO
import p5_gripper
import p5_grasp_core as GC
import p5_place_core as PC
results = []
TABLE = -0.0109
G = dict(p5_gripper.DEFAULTS)


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


def box_top(c, L, W, H, yaw, step=0.003, floor=TABLE):
    xs, ys = np.meshgrid(np.arange(-L / 2, L / 2 + 1e-9, step), np.arange(-W / 2, W / 2 + 1e-9, step))
    u, v = xs.ravel(), ys.ravel()
    return np.column_stack([c[0] + u * math.cos(yaw) - v * math.sin(yaw), c[1] + u * math.sin(yaw) + v * math.cos(yaw), np.full(len(u), floor + H)])


# the object in hand
O = box_top((-0.35, 0.05), 0.12, 0.04, 0.05, math.radians(30))
sl = SO.object_slices(O, TABLE)
yaw_ok = all(abs(((math.degrees(b["yaw"]) - 30 + 90) % 180) - 90) < 3 for b in sl)
check("object in hand: 4 slices along its length, at its yaw", len(sl) == 4 and yaw_ok, "%d slices, yaw %.0f" % (len(sl), math.degrees(sl[0]["yaw"])))
check("  ... each as wide and tall as the object there (+3 mm), standing on the floor", all(abs(b["size"][1] - 0.046) < 0.005 and abs(b["size"][2] - 0.051) < 0.004
      and abs(b["centre"][2] - b["size"][2] / 2 - (TABLE + 0.002)) < 1e-6 for b in sl), [round(b["size"][1], 3) for b in sl])
L = sum(b["size"][0] for b in sl)
check("  ... together as long as the object", abs(L - 0.12 - 4 * 0.006) < 0.01, "%.3f m" % L)
# a tapered (eggplant-like) object: the slices follow its shape
u = np.linspace(-0.08, 0.08, 60); pts = []
for x in u:
    r = 0.012 + 0.018 * (x + 0.08) / 0.16                       # thin neck at -x, fat end at +x
    for y in np.linspace(-r, r, 12):
        pts.append([-0.35 + x, 0.05 + y, TABLE + 2 * r])
sl2 = SO.object_slices(np.array(pts), TABLE)
w = [b["size"][1] for b in sorted(sl2, key=lambda b: b["centre"][0])]
check("tapered object: the slices are thin at the neck, wide at the fat end", w == sorted(w) and w[-1] - w[0] > 0.02, [round(x, 3) for x in w])
check("too few points -> no object model (nothing added)", SO.object_slices(O[:5], TABLE) == [])

# 7k: on the table, the measured floor came out a few mm BELOW MoveIt's table surface -> the model was attached into the table
Ot = box_top((-0.35, 0.05), 0.08, 0.03, 0.028, 0.3)              # a 28 mm object on the table
sl_t = SO.object_slices(Ot, TABLE - 0.004, table_z=TABLE)        # floor measured 4 mm too low
check("object on the table: the model's bottom stays 3 mm above the table surface (no 'table_surface colliding')",
      all(b["centre"][2] - b["size"][2] / 2 >= TABLE + 0.003 - 1e-9 for b in sl_t) and len(sl_t) == 4,
      "lowest bottom %+.1f mm vs the table" % (1000 * min(b["centre"][2] - b["size"][2] / 2 - TABLE for b in sl_t)))
check("  ... and its top is unchanged (the object's real height)", all(abs(b["centre"][2] + b["size"][2] / 2 - (TABLE + 0.028 + 0.003)) < 0.003 for b in sl_t))
sl_s = SO.object_slices(Ot + np.array([0, 0, 0.033]), TABLE + 0.033, table_z=TABLE)   # in the sink (floor 33 mm up): as before
check("  ... in the sink the measured floor is used as before", all(abs(b["centre"][2] - b["size"][2] / 2 - (TABLE + 0.033 + 0.002)) < 1e-6 for b in sl_s))

# the camera box
cb = SO.camera_box(G)
check("camera box: 8 x 8 x 8 cm beside the gripper (tcp_link +x, from its side face out)", cb["size"] == [0.08, 0.08, 0.08]
      and abs(cb["centre"][0] - 0.0675) < 1e-9 and abs(cb["centre"][0] - 0.04 - 0.5 * G["BODY_X"]) < 1e-9)
check("your calibration (+89, -14, +22) mm is INSIDE it; the old spot behind tcp_link is not",
      SO.inside_box([0.089, -0.014, 0.022], cb, 0.0) and not SO.inside_box([0.0, 0.0, -0.05], cb, 0.02))
bx = p5_gripper.boxes(G, 0.06, sweep=0.08)
check("gripper model: the camera box is part of it (swept along the approach too)", "camera" in bx and abs(bx["camera"][0][2] - (0.022 - 0.04 - 0.08)) < 1e-9)
check("  ... and it does not overlap the gripper body", bx["camera"][0][0] >= bx["body"][1][0] - 1e-9)

# grasp check: the camera is on ONE side of the gripper - a tall post on that side is avoided by rolling the grasp 180 deg (same fingers)
import test_p5_grasp_core as T                                  # its Scene helper


def free_of(res, pts):
    for sel in res["selected"]:
        q = GC.to_frame(pts, sel["tcp"], sel["R"])
        for k, b in p5_gripper.boxes(G, sel["open"], sweep=G["APPROACH_M"]).items():
            if GC.in_box(q, b).any():
                return False
    return True


s = T.Scene().box((T.X0, T.Y0), 0.06, 0.04, 0.04, 0.0, 1).box((T.X0 - 0.065, T.Y0), 0.015, 0.015, 0.30, 0.0, 2)
r = s.run()
post = s.pts(0)[s.label == 2]
check("grasp check: a tall post on the camera's side -> the grasp is rolled so the camera is on the other side", r["ok"] and free_of(r, post)
      and any(x.get("camera_side_flipped") for x in r["selected"]), r["why"] or "G1 jaw %.0f" % r["selected"][0]["jaw_deg"])
s2 = T.Scene().box((T.X0, T.Y0), 0.06, 0.04, 0.04, 0.0, 1)
for dx, dy in ((0.065, 0), (-0.065, 0), (0, 0.075), (0, -0.075)):
    s2.box((T.X0 + dx, T.Y0 + dy), 0.015, 0.015, 0.30, 0.0, 2)
r2 = s2.run()
check("  ... posts on every side: rejected because of the camera, or a grasp clear of all of them",
      (r2["ok"] and free_of(r2, s2.pts(0)[s2.label == 2])) or any("camera" in c["why"] for c in r2["candidates"] if not c["ok"]),
      r2["why"][:80] if not r2["ok"] else "G1 jaw %.0f" % r2["selected"][0]["jaw_deg"])

# place check: something tall under the camera's footprint counts
c = np.zeros(3)
E0 = np.zeros((0, 3))
# jaw at 90 deg: tcp_link y = (0, 1), x = n = (-1, 0): the camera sits 2.75-10.75 cm on the -x side of the release point
side = np.array([[-0.065, 0.0, 0.30]]); other = np.array([[0.065, 0.0, 0.30]])
check("place check: a tall thing under the camera (its side) is in the way, the same on the other side is not",
      PC.footprint_hits(E0, E0, c, math.radians(90), 0.06, G, 0.0, None, side) == 1 and PC.footprint_hits(E0, E0, c, math.radians(90), 0.06, G, 0.0, None, other) == 0)

# MTC: contact with the object allowed BEFORE opening and the approach
mtc = open(os.path.join(P5, "core", "p3_mtc.py")).read()
check("MTC: the contact allowance names the real finger LINKS (a plain 'gripper' string is no link: closing failed)",
      'allowCollisions(["carrot"], ["gripper_base", "gripper_link1", "gripper_link2"' in mtc and 'GRIPPER_GROUP, True)' not in mtc)
check("MTC: gripper-object contact allowed before 'open gripper' and 'approach'", mtc.index('ModifyPlanningScene("allow gripper-object contact")')
      < mtc.index('stages.MoveTo("open gripper"') < mtc.index('stages.MoveRelative("approach"') and mtc.count('allowCollisions(') == 1)
print("%s (%s, %s): %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", SO.P5_SCENE_OBJECTS_VERSION, p5_gripper.P5_GRIPPER_VERSION, sum(results), len(results)))
sys.exit(0 if all(results) else 1)
