import os, time
def load_config(ckpt_dir, batch_size=1):
    time.sleep(float(os.environ.get("STUB_LOAD_S", "0.05")))
    return {"batch_size": batch_size}
