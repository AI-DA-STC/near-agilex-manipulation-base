MoSAIC-DUO hybrid, step 2: GraspMolmo + SAM2 on the laptop GPU, switched on with P5_REMOTE=1

WHAT CHANGES
  Without P5_REMOTE=1 pipeline 5 runs exactly as before (same models on the Jetson).
  With P5_REMOTE=1 the GraspMolmo node and the SAM2 worker send their questions to p5_server.py on the laptop.
  Measured on run 20261127_103100 (same picture, same prompts): grasp question 29.6 s -> 4.2 s, place question 18.6 s -> 2.4 s,
  same grasp pixel, place point 1.3 px apart, SAM2 object mask 750 px (Jetson) vs 756 px (laptop).

SERVER ORDER  P5_SERVER (comma list), default http://10.10.0.1:8765 (cable), then http://192.168.10.146:8765 (Wi-Fi)
FALLBACK      laptop down at start      -> the Jetson's own models start instead (slower, same results)
              laptop dies mid-run       -> other address tried; if none answers, that request fails cleanly and the
                                           next request uses the Jetson's own models
              P5_REMOTE_TIMEOUT (s)     -> how long one answer may take (default 90 GraspMolmo, 30 SAM2)
              P5_REMOTE_WAIT (s)        -> how long to look for a server at start (default 8)
              P5_REMOTE_NO_FALLBACK=1   -> no fallback: fail instead

FILES
  p5_remote/      LAPTOP   p5_server.py (p5-server-v2, GraspMolmo pinned to snapshot 9aa184d6), start_server.sh, p5_server_test.py
  jetson/remote/  JETSON   p5_remote_worker.py (v2) + p5_remote_graspmolmo.py / p5_remote_sam2.py -> pipeline5/remote/
  jetson/         JETSON   install_p5_remote.sh, uninstall_p5_remote.sh, p5_remote_patch.py
  tests/          BOTH     offline_test.py (no GPU, no models, no ROS)

THE PATCH (pipeline 5, backed up to pipeline5/backups/p5remote_<time>/ first; undo: uninstall_p5_remote.sh)
  start_pipeline5.sh  p5-start-v8a -> v8b: with P5_REMOTE=1, adds -p worker_script:=<pipeline5>/remote/p5_remote_graspmolmo.py
                      to the GraspMolmo node and prints which laptop address answers
  p5_grasp_node.py    p5-grasp-node-v6e -> v6f: with P5_REMOTE=1, starts remote/p5_remote_sam2.py instead of p5_sam2_worker.py
  p5_clean.sh         also stops p5_remote_* workers
  core/ is NOT changed (COPIED_FROM_PIPELINE4.txt checksums stay valid).

LAPTOP
  unzip -o ~/Downloads/p5_remote_step2.zip -d ~/p5_server
  cd ~/p5_server/p5_remote_step2 && conda activate p5_gpu && python tests/offline_test.py
  bash ~/p5_server/p5_remote_step2/p5_remote/start_server.sh        (stop the old server and pi0.5 first)

JETSON (pipeline stopped)
  scp the zip from the laptop: scp ~/Downloads/p5_remote_step2.zip near-nero@10.10.0.2:~/      (run on the laptop)
  unzip -o ~/p5_remote_step2.zip -d ~ && bash ~/p5_remote_step2/jetson/install_p5_remote.sh
  P5_REMOTE=1 bash ~/nero_pipeline_launch/pipeline5/start_pipeline5.sh
