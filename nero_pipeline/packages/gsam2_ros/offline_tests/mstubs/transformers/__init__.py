import numpy as np, time, os
_LOAD = float(os.environ.get('STUB_LOAD_S', '0.05'))
class _T(np.ndarray):
    def cpu(self): return self
    def numpy(self): return np.asarray(self)
class _Inputs(dict):
    input_ids = None
    def to(self, d): return self
class _Proc:
    @classmethod
    def from_pretrained(cls, name): time.sleep(_LOAD); return cls()
    def __call__(self, images=None, text=None, return_tensors=None):
        if 'crash' in str(text): os._exit(3)      # simulate the worker process dying mid-request
        i = _Inputs(); i.input_ids = text; return i
    def post_process_grounded_object_detection(self, outputs, ids, box_threshold, text_threshold, target_sizes):
        h, w = target_sizes[0]
        if 'nothing' in str(ids):
            return [{'boxes': np.zeros((0, 4)).view(_T), 'scores': np.zeros(0).view(_T), 'labels': []}]
        boxes = np.array([[10, 10, w // 2, h // 2], [5, 5, 20, 20]], dtype=float).view(_T)
        return [{"boxes": boxes, "scores": np.array([0.8, 0.3]).view(_T), "labels": ["plate", "other"]}]
class AutoProcessor(_Proc): pass
class _Model:
    def to(self, d): return self
    def __call__(self, **kw): time.sleep(0.02); return object()
class AutoModelForZeroShotObjectDetection:
    @classmethod
    def from_pretrained(cls, name): time.sleep(_LOAD); return _Model()
