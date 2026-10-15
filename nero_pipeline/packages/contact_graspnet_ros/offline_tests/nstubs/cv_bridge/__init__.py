class CvBridge:
    def imgmsg_to_cv2(self, m, *a, **k): return m
    def cv2_to_imgmsg(self, arr, encoding="bgr8"): return ("img", encoding, arr.shape)
