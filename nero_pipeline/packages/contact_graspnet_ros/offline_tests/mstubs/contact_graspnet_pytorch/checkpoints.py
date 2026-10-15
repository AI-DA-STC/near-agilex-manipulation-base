import os, time
class CheckpointIO:
    def __init__(self, checkpoint_dir=None, model=None): pass
    def load(self, name):
        if os.environ.get("STUB_NO_CKPT"): raise FileExistsError(name)
        time.sleep(float(os.environ.get("STUB_LOAD_S", "0.05")))
