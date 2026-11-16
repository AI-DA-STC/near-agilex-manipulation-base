#!/usr/bin/env python3
"""
p3_logfilter.py (a copy of pipeline 2's p2_logfilter.py, plus one message) - hides KNOWN harmless MoveIt / OMPL / rcutils noise from the motion-planning output, so the lines that matter are visible.
    ros2 launch ... 2>&1 | python3 p3_logfilter.py        (run_pick_and_place3.sh does this; P3_LOG_FILTER=0 shows everything)
It hides only lines that match the exact messages below (the same ones, hundreds of times per run):
  - OMPL: "Cannot find planning configuration for group '...' using planner '...'. Will use defaults instead."
  - OMPL: "It looks like the planning volume was not specified."
  - OMPL: "Planner configuration '...' will use planner 'geometric::...'. Additional configuration parameters will be set when the planner is constructed."
  - MoveIt: "Attached body 'carrot' not found"   (a leftover name of MTC's demo object: nothing is attached, nothing is wrong)
  - rcutils: the 7-line block about 'rosidl_typesupport_cpp' being overwritten by 'Expected lexeme type (19) not found' (harmless type-support noise); an rcutils block with any other content is shown.
Everything else passes through unchanged, line by line, immediately. At the end it says how many lines it hid.
"""
import re
import signal
import sys

P3_LOGFILTER_VERSION = "p3-logfilter-v1"

LINE_PATTERNS = [re.compile(p) for p in (
    r"Cannot find planning configuration for group '[^']*' using planner '[^']*'\. Will use defaults instead\.",
    r"It looks like the planning volume was not specified\.",
    r"Planner configuration '[^']*' will use planner 'geometric::[A-Za-z0-9_]+'\. Additional configuration parameters will be set when the planner is constructed\.",
    r"Attached body 'carrot' not found",
    r"Attempting to attach object 'carrot' to link '[^']*' but no geometry specified",   # pipeline 1's MTC uses the leftover name 'carrot' for the held object
)]
BLOCK_START = ">>> [rcutils|error_handling.c"
BLOCK_END = "<<<"
BLOCK_MUST_CONTAIN = ("rosidl_typesupport_cpp", "lexer_lookahead")
MAX_BLOCK_LINES = 40


def is_noise_line(line):
    return any(p.search(line) for p in LINE_PATTERNS)


def filter_stream(lines, out):
    """Writes the lines that are not known noise to out (flushing each); returns how many lines it hid."""
    hidden = 0
    block = None
    for line in lines:
        if block is not None:
            block.append(line)
            if line.rstrip().endswith(BLOCK_END):
                text = "".join(block)
                if all(m in text for m in BLOCK_MUST_CONTAIN):
                    hidden += len(block)
                else:
                    out.write(text)
                    out.flush()
                block = None
            elif len(block) > MAX_BLOCK_LINES:
                out.write("".join(block))
                out.flush()
                block = None
            continue
        if BLOCK_START in line:
            block = [line]
            continue
        if is_noise_line(line):
            hidden += 1
            continue
        out.write(line)
        out.flush()
    if block:
        out.write("".join(block))
        out.flush()
    return hidden


def main():
    signal.signal(signal.SIGINT, signal.SIG_IGN)       # Ctrl+C ends the program upstream; this one carries on until the pipe closes, so nothing is lost
    try:
        sys.stdin.reconfigure(errors="replace")
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    try:
        hidden = filter_stream(iter(sys.stdin.readline, ""), sys.stdout)
        if hidden:
            sys.stdout.write("(p3_logfilter: hid %d lines of known harmless MoveIt/OMPL/rcutils noise ('carrot' not found, OMPL planner defaults, rcutils type-support); P3_LOG_FILTER=0 shows everything)\n" % hidden)
            sys.stdout.flush()
    except BrokenPipeError:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
