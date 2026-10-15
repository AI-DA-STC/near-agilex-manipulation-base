"""Offline tests for ask_prompts.py: answer parsing, and the interactive flow against a FAKE ros2 that stores parameters in a temp folder.
Run:  python3 test_ask_prompts.py        (needs only Python 3)"""
import json, os, stat, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); SCRIPT = os.path.join(os.path.dirname(HERE), "ask_prompts.py")
sys.path.insert(0, os.path.dirname(HERE))
import ask_prompts as A
ok = True
def check(c, m):
    global ok; print(("  PASS  " if c else "  FAIL  ") + m); ok = ok and c

# ---- parsing
P = A.parse_answer
check(P("") == ("keep", {}) and P("y") == ("keep", {}) and P("  Y ") == ("keep", {}) and P("yes") == ("keep", {}), "empty / y / Y / yes keep the last prompts")
check(P("s:red mug t:blue tray") == ("set", {"source": "red mug.", "target": "blue tray."}), "both prompts")
check(P("t:tray s:cup") == ("set", {"target": "tray.", "source": "cup."}), "either order")
check(P("S:Red Cup T:The White Plate.") == ("set", {"source": "red cup.", "target": "the white plate."}), "lowercased, a typed period is not doubled")
check(P("s:  a  red   cup  ") == ("set", {"source": "a red cup."}), "extra spaces cleaned; only the source changes")
check(P("t:bowl") == ("set", {"target": "bowl."}), "only the target changes")
for bad in ("hello", "s:", "s:cup t:", "s:a s:b", "please s:cup", "s:cup!", "s:" + "x" * 90, "s:-rf", "s:bus: red"):
    try: P(bad); check(False, "should reject %r" % bad[:14])
    except ValueError: check(True, "rejects %r" % bad[:14])
check(A.normalize("Plate") == "plate." and A.normalize("plate..") == "plate.", "normalize: lowercase, exactly one trailing period")

# ---- interactive flow against a fake ros2
tmp = tempfile.mkdtemp(); bindir = os.path.join(tmp, "bin"); os.makedirs(bindir)
fake = os.path.join(bindir, "ros2")
open(fake, "w").write('''#!/bin/bash
ST="$FAKE_DIR/params.txt"; touch "$ST"
[ -f "$FAKE_DIR/node_down" ] && { echo "Node not found"; exit 1; }
if [ "$1 $2" = "param get" ]; then v=$(grep "^$4=" "$ST" | cut -d= -f2-); [ -z "$v" ] && { [ "$4" = "source_prompt" ] && v="carrot." || v="plate."; }; echo "String value is: $v"; exit 0; fi
if [ "$1 $2" = "param set" ]; then grep -v "^$4=" "$ST" > "$ST.tmp"; V="$5"; V="${V%\\"}"; V="${V#\\"}"; echo "$4=$V" >> "$ST.tmp"; mv "$ST.tmp" "$ST"; echo "Set parameter successful"; exit 0; fi
exit 2
''')
os.chmod(fake, os.stat(fake).st_mode | stat.S_IEXEC)
ENV = dict(os.environ, PATH=bindir + ":" + os.environ["PATH"], FAKE_DIR=tmp, NERO_PROMPT_FILE=os.path.join(tmp, "last.json"), ASK_PROMPTS="force")
def run(text, env=None):
    r = subprocess.run([sys.executable, SCRIPT], input=text, capture_output=True, text=True, env=env or ENV); return r.returncode, r.stdout
def node(): return dict(l.split("=", 1) for l in open(os.path.join(tmp, "params.txt")).read().split("\n") if l)
rc, out = run("y\n"); check(rc == 0 and "carrot." in out and node() == {"source_prompt": "carrot.", "target_prompt": "plate."}, "first run shows the node's current prompts and y keeps them")
rc, out = run("s:red mug t:blue tray\n"); check(node() == {"source_prompt": "red mug.", "target_prompt": "blue tray."}, "s: and t: change both on the node")
rc, out = run("\n"); check("red mug." in out and node()["source_prompt"] == "red mug.", "the next run shows the LAST prompts; Enter keeps them")
rc, out = run("s:green cube\n"); check(node() == {"source_prompt": "green cube.", "target_prompt": "blue tray."}, "only s: given: the target stays")
os.remove(os.path.join(tmp, "params.txt")); rc, out = run("y\n"); check(node() == {"source_prompt": "green cube.", "target_prompt": "blue tray."}, "after a node restart y puts the remembered prompts back")
rc, out = run("banana\ns:\nt:bowl\n"); check(rc == 0 and out.count("Use  y  or") == 2 and node()["target_prompt"] == "bowl.", "two bad answers are explained, the third is used")
rc, out = run("x\nx\nx\n"); check("keeping the last prompts" in out and node()["target_prompt"] == "bowl.", "three bad answers keep the last prompts")
rc, out = run("s:cup\n", dict(ENV, ASK_PROMPTS="0")); check(out == "" and node()["source_prompt"] == "green cube.", "ASK_PROMPTS=0: no question, nothing touched")
env2 = {k: v for k, v in ENV.items() if k != "ASK_PROMPTS"}; rc, out = run("s:cup\n", env2); check(out == "" and node()["source_prompt"] == "green cube.", "not a terminal: no question, nothing touched")
rc, out = run("s:5\n"); check(node()["source_prompt"] == "5.", "a numeric-looking prompt is sent as text")
open(os.path.join(tmp, "node_down"), "w").close(); rc, out = run("y\n"); check(rc == 1 and "Could not set" in out, "node unreachable: clear message and a failing exit code")
sys.exit(0 if ok else 1)
