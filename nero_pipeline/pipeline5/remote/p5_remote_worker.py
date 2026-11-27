#!/usr/bin/env python3
"""Stand-in for the Jetson's GraspMolmo and SAM2 workers (PIPELINE 5, P5_REMOTE=1). Standard library only.
It speaks the SAME spool-folder protocol as core/p3_graspmolmo_worker.py and p5_sam2_worker.py, but sends each request to
p5_server.py on the laptop GPU, so the nodes cannot tell the difference. Started through p5_remote_graspmolmo.py or
p5_remote_sam2.py (which fill in --kind and --local), by core/worker_client.py (which adds --spool and --parent_pid).

    worker -> ready.json        once a laptop server answers /health   ("remote": true, "server": ...)
    client -> req_<id>.json     graspmolmo: {"cmd": "point", "image": path, "queries": [...], "max_new_tokens": n}
                                sam2:       {"cmd": "segment", "image": path, "point": [u, v], "out_dir": path}
                                both:       {"cmd": "ping"}
    worker -> res_<id>.json     the same replies as the local workers (SAM2 masks are written into out_dir as mask_<i>.png)
    client -> quit              asks the worker to exit (it also exits when its parent or the spool directory is gone)

SERVERS: P5_SERVER is a comma list, tried in order (default: the cable 10.10.0.1, then Wi-Fi 192.168.10.146).
FALLBACK:
  - no server answers at start (P5_REMOTE_WAIT s)  -> this process BECOMES the local Jetson worker (exec, same conda env,
                                                      same arguments), so the pipeline runs as before, only slower
  - a server stops answering mid-run               -> the next server in the list is tried; if none answers (or the answer
                                                      takes longer than P5_REMOTE_TIMEOUT s) the request gets {"ok": false}
                                                      and this worker EXITS; worker_client restarts it on the next request,
                                                      which then falls back to the local worker if the laptop is still down
  P5_REMOTE_NO_FALLBACK=1 turns the fallback off (exit code 2 instead).
"""
import argparse
import base64
import glob
import json
import os
import socket
import sys
import time
import traceback
import urllib.error
import urllib.request

P5_REMOTE_WORKER_VERSION = "p5-remote-worker-v2"   # v2: server list (cable first), local fallback, ping, per-request log | v1: one server
DEFAULT_SERVERS = "http://10.10.0.1:8765,http://192.168.10.146:8765"
DEFAULT_TIMEOUT = {"graspmolmo": 90.0, "sam2": 30.0}   # the nodes give up after 180 s (GraspMolmo) and 60 s (SAM2)
POLL_S = 0.03
EXIT_UNREACHABLE = 4


def log(msg):
    print(f"[p5 remote worker] {msg}", file=sys.stderr, flush=True)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


class Unreachable(Exception):
    """No server in the list answered (refused, unreachable, or no answer in time)."""


def _is_timeout(e):
    return isinstance(e, (socket.timeout, TimeoutError)) or (isinstance(e, urllib.error.URLError) and isinstance(e.reason, (socket.timeout, TimeoutError)))


def http_get(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


def http_post(url, obj, timeout):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


class Servers:
    def __init__(self, urls, timeout):
        self.urls = [u.strip().rstrip("/") for u in urls if u.strip()]
        self.active = None
        self.timeout = timeout

    def find(self, kind, wait_s):
        """First server (in list order) whose /health says ok and has this model. -> (url, health) or (None, last error)."""
        t0, last = time.time(), "no server listed"
        while True:
            for u in self.urls:
                try:
                    h = http_get(u + "/health", 2.0)
                    if h.get("ok") and h.get(kind):
                        self.active = u
                        return u, h
                    last = f"{u} is up but has not loaded {kind}"
                except (urllib.error.URLError, OSError, ValueError) as e:
                    last = f"{u}: {getattr(e, 'reason', e)}"
            if time.time() - t0 >= wait_s:
                return None, last
            time.sleep(0.5)

    def post(self, path, obj):
        """POST to the active server; on a connection failure try the others. A TIMEOUT is not retried (the GPU may still be
        busy with it). -> (reply, url). Raises Unreachable."""
        order = [self.active] + [u for u in self.urls if u != self.active] if self.active else list(self.urls)
        errors = []
        for u in order:
            try:
                rep = http_post(u + path, obj, self.timeout)
                if u != self.active:
                    log(f"now using {u} (was {self.active})")
                    self.active = u
                return rep, u
            except Exception as e:                         # noqa: BLE001 - every failure is reported, none hangs the pipeline
                if _is_timeout(e):
                    raise Unreachable(f"{u} gave no answer within {self.timeout:.0f} s")
                if isinstance(e, (urllib.error.URLError, OSError, ConnectionError)) and not isinstance(e, urllib.error.HTTPError):
                    errors.append(f"{u}: {getattr(e, 'reason', e)}")
                    continue
                raise
        raise Unreachable("no laptop server answered (" + "; ".join(errors) + ")")


def handle(kind, servers, req):
    cmd = req.get("cmd")
    if cmd == "ping":
        return {"ok": True, "version": P5_REMOTE_WORKER_VERSION, "remote": True, "server": servers.active}
    want = "point" if kind == "graspmolmo" else "segment"
    if cmd != want:
        return {"ok": False, "error": f"unknown cmd {cmd!r}"}
    with open(req["image"], "rb") as f:
        img_b64 = base64.b64encode(f.read()).decode()
    t0 = time.time()
    if kind == "graspmolmo":
        rep, url = servers.post("/point", {"image_b64": img_b64, "queries": req["queries"],
                                           "max_new_tokens": int(req.get("max_new_tokens", 256))})
        if rep.get("ok"):
            log("point by %s: %s | round trip %.2f s" % (url, ", ".join("%s %.2f s" % (r["name"], r.get("seconds", 0)) for r in rep["results"]),
                                                         time.time() - t0))
        return rep
    rep, url = servers.post("/segment", {"image_b64": img_b64, "point": [float(req["point"][0]), float(req["point"][1])]})
    if not rep.get("ok"):
        return rep
    out_dir = req["out_dir"]
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for i, b in enumerate(rep["masks_png_b64"]):
        p = os.path.join(out_dir, f"mask_{i}.png")
        with open(p, "wb") as f:
            f.write(base64.b64decode(b))
        paths.append(p)
    log("segment by %s: %.2f s on the GPU, round trip %.2f s" % (url, rep.get("seconds", 0), time.time() - t0))
    return {"ok": True, "masks": paths, "scores": rep["scores"], "seconds": rep["seconds"]}


def parent_alive(pid):
    if not pid:
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def become_local(local, passthrough, spool, parent_pid, why):
    """Replace this process with the local Jetson worker (same conda env, same working directory, same arguments)."""
    argv = [sys.executable, "-u", local] + passthrough + ["--spool", spool, "--parent_pid", str(parent_pid)]
    log(f"FALLBACK to the Jetson's local model ({why}): {' '.join(argv)}")
    sys.stderr.flush()
    os.execv(sys.executable, argv)


def main(argv=None):
    ap = argparse.ArgumentParser(allow_abbrev=False)
    ap.add_argument("--kind", required=True, choices=["graspmolmo", "sam2"])
    ap.add_argument("--spool", required=True)
    ap.add_argument("--parent_pid", type=int, default=0)
    ap.add_argument("--local", default="", help="the local worker script to become if no laptop server answers at start")
    ap.add_argument("--servers", default=os.environ.get("P5_SERVER", DEFAULT_SERVERS))
    ap.add_argument("--timeout", type=float, default=float(os.environ.get("P5_REMOTE_TIMEOUT", "0") or 0))
    ap.add_argument("--wait_s", type=float, default=float(os.environ.get("P5_REMOTE_WAIT", "8")))
    a, passthrough = ap.parse_known_args(argv)            # the local workers' own flags (--dtype, --model, --repo ...) pass through
    timeout = a.timeout or DEFAULT_TIMEOUT[a.kind]
    no_fallback = os.environ.get("P5_REMOTE_NO_FALLBACK", "0") == "1" or not a.local
    os.makedirs(a.spool, exist_ok=True)
    servers = Servers(a.servers.split(","), timeout)

    t0 = time.time()
    url, health = servers.find(a.kind, a.wait_s)
    if url is None:
        why = f"no laptop server answered within {a.wait_s:.0f} s: {health}"
        if not no_fallback and os.path.isfile(a.local):
            become_local(a.local, passthrough, a.spool, a.parent_pid, why)   # does not return
        log(f"ERROR: {why}. Start p5_server.py on the laptop (or unset P5_REMOTE). Tried: {', '.join(servers.urls)}")
        return 2
    log(f"{P5_REMOTE_WORKER_VERSION}: {a.kind} served by {url} (server {health.get('version')}, revision {health.get('revision', '?')}); "
        f"order {', '.join(servers.urls)}; timeout {timeout:.0f} s")
    write_json_atomic(os.path.join(a.spool, "ready.json"),
                      {"load_s": round(time.time() - t0, 1), "pid": os.getpid(), "fake": bool(health.get("fake", False)),
                       "dtype": "remote", "version": P5_REMOTE_WORKER_VERSION, "remote": True, "kind": a.kind,
                       "server": url, "server_version": health.get("version"), "revision": health.get("revision")})
    while True:
        if not os.path.isdir(a.spool) or os.path.exists(os.path.join(a.spool, "quit")) or not parent_alive(a.parent_pid):
            log("exiting")
            return 0
        for req_path in sorted(glob.glob(os.path.join(a.spool, "req_*.json"))):
            rid = os.path.basename(req_path)[4:-5]
            leave = False
            try:
                with open(req_path) as f:
                    req = json.load(f)
                os.remove(req_path)
                reply = handle(a.kind, servers, req)
            except Unreachable as e:
                reply = {"ok": False, "error": f"laptop server unreachable: {e}. This worker stops; the next request restarts it "
                                               f"(the Jetson's local model is used if the laptop is still down)."}
                leave = True
            except Exception as e:                         # noqa: BLE001
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-800:]}
            write_json_atomic(os.path.join(a.spool, f"res_{rid}.json"), reply)
            if leave:
                log("ERROR: " + reply["error"])
                return EXIT_UNREACHABLE
        time.sleep(POLL_S)


if __name__ == "__main__":
    sys.exit(main())
