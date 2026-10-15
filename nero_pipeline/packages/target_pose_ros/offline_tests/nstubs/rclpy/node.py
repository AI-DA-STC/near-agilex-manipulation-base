class _Clock:
    def now(self):
        class T:
            def to_msg(self): return None
        return T()
class _Log:
    def __init__(self): self.lines = []
    def info(self, m, *a, **k): self.lines.append(("info", str(m)))
    def warn(self, m, *a, **k): self.lines.append(("warn", str(m)))
    def error(self, m, *a, **k): self.lines.append(("error", str(m)))
    def debug(self, m, *a, **k): pass
class _P:
    def __init__(self, v): self.value = v
class Node:
    def __init__(self, name): self._p = {}; self.published = []; self._log = _Log()
    def declare_parameter(self, k, v=None): self._p[k] = v
    def set_param(self, k, v): self._p[k] = v
    def get_parameter(self, k): return _P(self._p[k])
    def create_subscription(self, *a, **k): pass
    def create_service(self, *a, **k): pass
    def create_publisher(self, *a, **k):
        node = self
        class Pub:
            def publish(self, m): node.published.append(m)
        return Pub()
    def get_logger(self): return self._log
    def get_clock(self): return _Clock()
