"""
Client for scripts/grounded_sam2_worker.py: starts it once (it loads the models once) and sends it segmentation requests.
No ROS imports, so it can be tested on its own. Requests and replies are plain files in a spool directory (see the worker).
"""
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
from collections import deque


class WorkerError(RuntimeError):
    """The worker did not start, died, or did not answer in time."""


class Gsam2Worker:
    def __init__(self, cmd, cwd=None, env=None, log=print, startup_timeout=240.0):
        self.cmd = list(cmd)            # command up to (and including) the worker script; --spool/--parent_pid are appended
        self.cwd, self.env, self.log = cwd, env, log
        self.startup_timeout = startup_timeout
        self._lock = threading.Lock()
        self._proc = None
        self._spool = None
        self._next_id = 0
        self.ready_info = None

    # ------------------------------------------------------------------ lifecycle
    def alive(self):
        return self._proc is not None and self._proc.poll() is None

    def start(self):
        with self._lock:
            self._start_locked()

    def _start_locked(self):
        if self.alive():
            return
        self._cleanup_locked()
        self._spool = tempfile.mkdtemp(prefix="gsam2_spool_")
        cmd = self.cmd + ["--spool", self._spool, "--parent_pid", str(os.getpid())]
        self.log(f"starting the Grounded-SAM-2 worker (loads the models once, takes a while): {' '.join(cmd)}")
        try:
            self._proc = subprocess.Popen(cmd, cwd=self.cwd, env=self.env, stdin=subprocess.DEVNULL)
        except OSError as e:
            self._cleanup_locked()
            raise WorkerError(f"could not start the worker: {e}")
        t0 = time.time()
        ready = os.path.join(self._spool, "ready.json")
        while not os.path.exists(ready):
            if self._proc.poll() is not None:
                code = self._proc.returncode
                self._cleanup_locked()
                raise WorkerError(f"the worker exited during start-up (code {code}); see its messages above")
            if time.time() - t0 > self.startup_timeout:
                self._kill_locked()
                raise WorkerError(f"the worker was not ready after {self.startup_timeout:.0f} s")
            time.sleep(0.1)
        with open(ready) as f:
            self.ready_info = json.load(f)
        self.log(f"worker ready after {time.time() - t0:.1f} s: {self.ready_info}")

    def _kill_locked(self):
        if self._proc is not None:
            try:
                if self._spool and os.path.isdir(self._spool):
                    open(os.path.join(self._spool, "quit"), "w").close()      # a hung worker that wakes up later will exit
                self._proc.kill()
            except OSError:
                pass
        self._cleanup_locked(keep_spool=True)

    def _cleanup_locked(self, keep_spool=False):
        self._proc = None
        if self._spool and not keep_spool:
            shutil.rmtree(self._spool, ignore_errors=True)
        self._spool = None

    def stop(self):
        with self._lock:
            if self.alive() and self._spool and os.path.isdir(self._spool):
                open(os.path.join(self._spool, "quit"), "w").close()
                try:
                    self._proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
            self._cleanup_locked()

    # ------------------------------------------------------------------ requests
    def request(self, payload, timeout=120.0):
        """Send one request, return the worker's reply dict. Starts (or restarts) the worker if needed. Raises WorkerError."""
        with self._lock:
            self._start_locked()
            self._next_id += 1
            rid = f"{self._next_id:06d}"
            req = os.path.join(self._spool, f"req_{rid}.json")
            res = os.path.join(self._spool, f"res_{rid}.json")
            tmp = req + ".tmp"
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, req)
            t0 = time.time()
            while not os.path.exists(res):
                if self._proc.poll() is not None:
                    code = self._proc.returncode
                    self._cleanup_locked()
                    raise WorkerError(f"the worker died while processing the request (exit code {code})")
                if time.time() - t0 > timeout:
                    self._kill_locked()
                    raise WorkerError(f"the worker did not answer within {timeout:.0f} s and was stopped")
                time.sleep(0.02)
            with open(res) as f:
                reply = json.load(f)
            os.remove(res)
            return reply
