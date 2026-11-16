"""p3_jobs.py - a minimal background job: start a function in a thread now, collect its result (or its exception) later. No ROS in here."""
import threading
import time

P3_JOBS_VERSION = "p3-jobs-v1"


class JobTimeout(RuntimeError):
    pass


class BackgroundJob:
    """job = BackgroundJob(fn, *args); ... later: result = job.wait(timeout) - returns fn's result, or re-raises the exception fn raised."""

    def __init__(self, fn, *args, **kwargs):
        self.started = time.time()
        self.finished = None
        self._result = None
        self._error = None
        self._thread = threading.Thread(target=self._run, args=(fn, args, kwargs), daemon=True)
        self._thread.start()

    def _run(self, fn, args, kwargs):
        try:
            self._result = fn(*args, **kwargs)
        except BaseException as e:            # handed to wait(), never lost
            self._error = e
        finally:
            self.finished = time.time()

    def done(self):
        return not self._thread.is_alive()

    def wait(self, timeout):
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise JobTimeout("still running after %.0f s" % timeout)
        if self._error is not None:
            raise self._error
        return self._result

    def seconds(self):
        return (self.finished or time.time()) - self.started
