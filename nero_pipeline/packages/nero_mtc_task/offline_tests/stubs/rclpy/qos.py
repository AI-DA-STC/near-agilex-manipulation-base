class _E:
    TRANSIENT_LOCAL = 1; RELIABLE = 1
class QoSProfile:
    def __init__(self, **k): self.__dict__.update(k)
QoSDurabilityPolicy = _E; QoSReliabilityPolicy = _E
