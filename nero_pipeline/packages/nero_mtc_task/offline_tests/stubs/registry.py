LOG = []
class _Props:
    def __getattr__(self, name):
        if name.startswith("__"): raise AttributeError(name)
        return lambda *a, **k: None
class Rec:
    """records construction and settings of an MTC object"""
    def __init__(self, kind, *args, **kw):
        object.__setattr__(self, "_kind", kind); object.__setattr__(self, "_args", args)
        object.__setattr__(self, "properties", _Props()); object.__setattr__(self, "_kids", []); object.__setattr__(self, "_calls", []); object.__setattr__(self, "_named", {})
        LOG.append(self)
    def add(self, *stages):
        for s in stages:
            self._kids.append(s)
            if getattr(s, "_args", None): self._named[s._args[0] if isinstance(s._args[0], str) else None] = s
    def __getitem__(self, name): return self._named.get(name) or Rec("lookup", name)
    def plan(self, *a): return False
    def __getattr__(self, name):
        if name.startswith("__"): raise AttributeError(name)
        def f(*a, **k): self._calls.append((name, a, k)); return None
        return f
    def name(self): return self._args[0] if self._args else ""
