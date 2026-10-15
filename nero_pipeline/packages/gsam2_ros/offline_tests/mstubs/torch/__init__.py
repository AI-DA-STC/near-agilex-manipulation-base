import numpy as np, contextlib
__version__ = "2.5.0-stub"
class _Cuda:
    @staticmethod
    def is_available(): return False
    @staticmethod
    def synchronize(): pass
cuda = _Cuda()
@contextlib.contextmanager
def no_grad(): yield
def argmax(x): return int(np.argmax(np.asarray(x)))
