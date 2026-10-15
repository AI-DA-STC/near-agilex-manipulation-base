#!/usr/bin/env python3
"""
ask_prompts.py - asks for the Grounded-SAM-2 source (pick) and target (place) prompts before a run, and sets them on the gsam2 node.  (system python3, ROS sourced)

    Enter or y            keep the last prompts
    s:red mug t:blue tray change both          (either order; s: = source/pick, t: = target/place)
    s:red mug             change only the source (the target stays as it was);  t:blue tray  changes only the target

Prompts are lowercased and given a trailing '.', which Grounding DINO expects. The last prompts are remembered in ~/.nero_last_prompts.json, also across restarts.
It asks only in an interactive terminal. Environment:  ASK_PROMPTS=0  skips the question and leaves the node's prompts untouched;  ASK_PROMPTS=force  asks even when stdin is not a terminal.
"""
import json, os, re, subprocess, sys

NODE = "/gsam2_segmentation_node"
STATE = os.path.expanduser(os.environ.get("NERO_PROMPT_FILE", "~/.nero_last_prompts.json"))
DEFAULTS = {"source": "carrot.", "target": "plate."}
MARK = re.compile(r"(?:^|\s)([stST]):")
KEEP = {"", "y", "yes", "k", "keep"}
PARAM = {"source": "source_prompt", "target": "target_prompt"}


def normalize(value):
    """-> prompt as Grounding DINO wants it, or raises ValueError with a message for the user"""
    v = " ".join(value.split()).strip(" \"'").lower().rstrip(". ").strip()
    if not v:
        raise ValueError("the prompt is empty")
    if len(v) > 80:
        raise ValueError("the prompt is longer than 80 characters")
    if not re.fullmatch(r"[a-z0-9][a-z0-9 ,'/()._-]*", v):
        raise ValueError("use letters, digits and spaces (no other symbols), starting with a letter or digit")
    return v + "."


def parse_answer(text):
    """-> ('keep', {}) | ('set', {'source': ..., 'target': ...}) with only the parts given ; raises ValueError for a bad answer"""
    t = text.strip()
    if t.lower() in KEEP:
        return "keep", {}
    marks = list(MARK.finditer(t))
    if not marks:
        raise ValueError("I did not understand that")
    if t[:marks[0].start()].strip():
        raise ValueError("start with s: or t:")
    out = {}
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(t)
        key = "source" if m.group(1).lower() == "s" else "target"
        if key in out:
            raise ValueError("%s: was given twice" % m.group(1).lower())
        out[key] = normalize(t[m.end():end])
    return "set", out


def load_state():
    try:
        d = json.load(open(STATE))
        return {k: d[k] for k in ("source", "target")}
    except Exception:
        return None


def save_state(d):
    try:
        json.dump(d, open(STATE, "w"))
    except OSError as e:
        print("  (could not remember the prompts: %s)" % e)


def ros(*args, timeout=25):
    r = subprocess.run(["ros2"] + list(args), capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr)


def node_value(key):
    rc, out = ros("param", "get", NODE, PARAM[key])
    m = re.search(r"String value is:\s*(.*)", out)
    return m.group(1).strip() if (rc == 0 and m) else None


def set_node(key, value):
    arg = value
    if re.fullmatch(r"[\d.+-]+", value) or value.lower().rstrip(".") in ("true", "false", "null", "yes", "no", "on", "off"):
        arg = '"%s"' % value                                    # keep YAML from reading it as a number or a bool
    rc, out = ros("param", "set", NODE, PARAM[key], arg)
    return rc == 0 and "Set parameter successful" in out, out.strip()


def main(argv=None, input_fn=input):
    mode = os.environ.get("ASK_PROMPTS", "")
    if mode == "0":
        return 0
    if not (sys.stdin.isatty() or mode == "force"):
        return 0                                                # not interactive (loops, pipes): leave everything as it is
    last = load_state()
    if last is None:                                            # first time: show what the node has now
        last = {k: node_value(k) or DEFAULTS[k] for k in ("source", "target")}
    print("\n=== Prompts for Grounded-SAM-2 ===")
    print("  source (pick):  %s" % last["source"]); print("  target (place): %s" % last["target"])
    print("Enter or y = keep these.   To change:  s:<source> t:<target>   e.g.  s:red mug t:blue tray   (or only s:red mug)")
    new = dict(last)
    for attempt in range(3):
        try:
            ans = input_fn("> ")
        except EOFError:
            ans = "y"
        try:
            kind, vals = parse_answer(ans)
            new.update(vals)
            break
        except ValueError as e:
            print("  %s. Use  y  or  s:<source> t:<target>." % e)
    else:
        print("  keeping the last prompts.")
    for key in ("source", "target"):
        ok, out = set_node(key, new[key])
        if not ok:
            print("\nCould not set %s on %s:\n  %s\nIs the pipeline running (start_pipeline.sh)?" % (PARAM[key], NODE, out.splitlines()[-1] if out else "no answer"))
            return 1
    save_state(new)
    print("Using:  source = '%s'   target = '%s'\n" % (new["source"], new["target"]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ncancelled"); sys.exit(130)
