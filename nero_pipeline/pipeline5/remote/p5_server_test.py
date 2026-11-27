#!/usr/bin/env python3
"""Ask the running laptop server a question, from any machine (standard library only).

  python p5_server_test.py --image rgb.png --ask "Point to the eggplant." --ask "Point to the center of the basket."
  python p5_server_test.py --image rgb.png --segment 320 240
"""
import argparse
import base64
import json
import os
import sys
import time
import urllib.request


def post(url, obj, timeout=300):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default=os.environ.get("P5_SERVER", "http://127.0.0.1:8765").split(",")[0])
    ap.add_argument("--image", required=True)
    ap.add_argument("--ask", action="append", default=[], help="a pointing question (repeatable)")
    ap.add_argument("--segment", nargs=2, type=float, metavar=("U", "V"))
    ap.add_argument("--out", default="/tmp")
    a = ap.parse_args()
    with open(a.image, "rb") as f:
        b64 = base64.b64encode(f.read()).decode()
    with urllib.request.urlopen(a.server + "/health", timeout=10) as r:
        print("health:", r.read().decode())
    if a.ask:
        t0 = time.time()
        rep = post(a.server + "/point", {"image_b64": b64, "queries": [{"name": f"q{i}", "text": t} for i, t in enumerate(a.ask)]})
        print(f"/point took {time.time() - t0:.1f} s round trip")
        print(json.dumps(rep, indent=2)[:2000])
    if a.segment:
        t0 = time.time()
        rep = post(a.server + "/segment", {"image_b64": b64, "point": list(a.segment)})
        print(f"/segment took {time.time() - t0:.1f} s round trip, ok={rep.get('ok')}, scores={rep.get('scores')}")
        for i, m in enumerate(rep.get("masks_png_b64", [])):
            p = f"{a.out}/mask_{i}.png"
            with open(p, "wb") as f:
                f.write(base64.b64decode(m))
            print("saved", p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
