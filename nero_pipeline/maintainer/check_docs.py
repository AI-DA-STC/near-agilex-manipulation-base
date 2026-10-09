#!/usr/bin/env python3
"""
check_docs.py - automatic quality checks for the Markdown in this folder.

    python3 maintainer/check_docs.py          (run from anywhere)

Checks: balanced code fences; table rows with the right number of cells (a pipe inside a code span must be written \\|);
relative links and #anchors resolve; every repo path mentioned in `backticks` exists (files that only exist after
collect_from_workspace.sh are reported as 'pending', not as errors); no TODO/FIXME placeholders; heading levels do not skip;
no bare `tools/` paths (they must be launch/tools/ or ~/nero_pipeline_launch/tools/).
Exit code 1 if there is any ERROR.
"""
import os, re, sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PENDING = {  # produced on the lab machine by maintainer/collect_from_workspace.sh
    "launch/start_pipeline.sh", "launch/run_pick_and_place.sh", "launch/ckpt.sh", "launch/scripts/add_table_collision.py",
    "launch/scripts/move_to_pose_90deg.py", "launch/scripts/approach_monitor.py", "launch/scripts/check_tcp_in_mtc.py",
    "launch/tools/adjust_calib_yaw.py", "docs/PACKAGE_VERSIONS.md",
}
PENDING_PREFIX = ("environments/", "patches/", "reference_copies/", "config_snapshots/")
GENERATED_OK = ("docs/PACKAGE_VERSIONS.md",)

errors, pending, warns = [], [], []


def md_files():
    out = []
    for d, _, fs in os.walk(ROOT):
        if ".git" in d.split(os.sep) or os.path.relpath(d, ROOT).split(os.sep)[0] == "packages":
            continue
        for f in fs:
            if f.endswith(".md"):
                out.append(os.path.join(d, f))
    return sorted(out)


def slug(h):
    h = re.sub(r"`", "", h.strip().lower())
    h = re.sub(r"[^\w\- ]", "", h)
    return h.replace(" ", "-")


def headings(path):
    hs, fence = [], False
    for line in open(path, encoding="utf-8"):
        if line.startswith("```"):
            fence = not fence
        if not fence:
            m = re.match(r"(#{1,6})\s+(.*)", line)
            if m:
                hs.append((len(m.group(1)), m.group(2)))
    return hs


def split_row(line):
    """split a table row on unescaped pipes that are outside code spans"""
    cells, cur, tick, i = [], "", False, 0
    s = line.strip()
    while i < len(s):
        c = s[i]
        if c == "\\" and i + 1 < len(s):
            cur += s[i:i + 2]; i += 2; continue
        if c == "`":
            tick = not tick
        if c == "|" and not tick:
            cells.append(cur); cur = ""
        else:
            cur += c
        i += 1
    cells.append(cur)
    if s.startswith("|"): cells = cells[1:]
    if s.endswith("|") and cells: cells = cells[:-1]
    return cells


def check(path):
    rel = os.path.relpath(path, ROOT)
    lines = open(path, encoding="utf-8").read().split("\n")
    # fences
    fence = False
    for n, line in enumerate(lines, 1):
        if line.startswith("```"):
            fence = not fence
    if fence:
        errors.append((rel, 0, "unbalanced ``` code fence"))
    # tables, links, paths, placeholders
    fence = False; ncols = None
    for n, line in enumerate(lines, 1):
        if line.startswith("```"):
            fence = not fence; ncols = None; continue
        if fence:
            continue
        if line.lstrip().startswith("|"):
            for span in re.findall(r"`[^`\n]*`", line):
                if re.search(r"(?<!\\)\|", span):
                    errors.append((rel, n, "unescaped | inside a code span in a table (GitHub splits the cell): write \\| or move the command out of the table: %s" % span[:60]))
            cells = split_row(line)
            if ncols is None:
                ncols = len(cells)
            elif re.match(r"^\s*\|?\s*:?-{2,}", line):
                pass
            elif len(cells) != ncols:
                errors.append((rel, n, "table row has %d cells, header has %d (an unescaped | inside a code span?)" % (len(cells), ncols)))
        else:
            ncols = None
        for m in re.finditer(r"(?<!!)\[([^\]]+)\]\(([^)]+)\)", line):
            tgt = m.group(2)
            if tgt.startswith(("http://", "https://", "mailto:")):
                continue
            file_part, _, anchor = tgt.partition("#")
            dest = path if not file_part else os.path.normpath(os.path.join(os.path.dirname(path), file_part))
            if not os.path.exists(dest):
                drel = os.path.relpath(dest, ROOT)
                (pending if (drel in PENDING or drel.startswith(PENDING_PREFIX)) else errors).append((rel, n, "link target missing: %s" % tgt))
            elif anchor and dest.endswith(".md"):
                if anchor not in [slug(t) for _, t in headings(dest)]:
                    errors.append((rel, n, "anchor not found: %s" % tgt))
        for m in re.finditer(r"`([^`\n]+)`", line):
            t = m.group(1).strip()
            if re.fullmatch(r"(docs|launch|maintainer|config_snapshots|reference_copies|environments|patches)/[\w./*-]+", t) and "*" not in t and "<" not in t:
                full = os.path.join(ROOT, t.rstrip("/"))
                if not os.path.exists(full):
                    if t in PENDING or t.startswith(PENDING_PREFIX):
                        pending.append((rel, n, "path produced later by the collector: %s" % t))
                    else:
                        errors.append((rel, n, "mentioned path does not exist: %s" % t))
            if not rel.startswith("launch" + os.sep) and re.search(r"(^|[\s(])tools/", t) and not re.search(r"(launch/tools/|nero_pipeline_launch/tools/)", t):
                errors.append((rel, n, "bare tools/ path (use launch/tools/ or ~/nero_pipeline_launch/tools/): %s" % t))
        if re.search(r"\[\[|\]\([^)]*\)\]\(|\[[^\]]*\[[^\]]*\]\(", line):
            errors.append((rel, n, "nested Markdown link (a link inside a link)"))
        if re.search(r"\b(TODO|FIXME|XXX|TBD)\b|<fill|lorem ipsum", line):
            errors.append((rel, n, "placeholder text"))
    # heading levels
    prev = 0
    for lvl, text in headings(path):
        if prev == 0 and lvl != 1:
            errors.append((rel, 0, "first heading is not H1: %s" % text))
        if prev and lvl > prev + 1:
            errors.append((rel, 0, "heading level jumps H%d -> H%d at: %s" % (prev, lvl, text)))
        prev = lvl


def main():
    files = md_files()
    for f in files:
        check(f)
    print("checked %d Markdown files" % len(files))
    for kind, items in (("ERROR", errors), ("pending", pending), ("warn", warns)):
        for rel, n, msg in items:
            print("%-7s %s%s: %s" % (kind, rel, (":%d" % n) if n else "", msg))
    print("\n%d error(s), %d pending (expected until collect_from_workspace.sh has been run), %d warning(s)" % (len(errors), len(pending), len(warns)))
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
