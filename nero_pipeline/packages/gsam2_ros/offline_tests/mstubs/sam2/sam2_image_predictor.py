import numpy as np
class SAM2ImagePredictor:
    def __init__(self, m): pass
    def set_image(self, img): self.shape = img.shape
    def predict(self, box=None, multimask_output=False):
        h, w = self.shape[:2]; m = np.zeros((1, h, w), bool); x0, y0, x1, y1 = [int(v) for v in box]; m[0, y0:y1, x0:x1] = True
        return m, np.array([0.93]), None
