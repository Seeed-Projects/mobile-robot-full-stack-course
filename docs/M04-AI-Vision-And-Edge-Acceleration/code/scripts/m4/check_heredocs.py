#!/usr/bin/env python3
"""Extract every `<<PY` / `<<'PY'` heredoc body from a shell script and parse it.

Used to prove the phase0 verify script's embedded Python is at least
syntactically valid -- the file previously shipped a hard syntax error
(`masks[i]=masks[i]` as a keyword name) that nothing had ever parsed.
"""
import ast
import sys

path = sys.argv[1]
lines = open(path).read().splitlines()

blocks = []
i = 0
while i < len(lines):
    line = lines[i]
    if "<<" in line:
        # Take just the token after <<, not the redirections that follow it:
        #   STEP1_OUT=$("$PYTHON" - <<'PY' 2>&1 || echo "ERROR"
        after = line.split("<<", 1)[1].strip()
        if not after:
            i += 1
            continue
        marker = after.split()[0].strip("'\"")
        if marker in ("PY", "PYEOF"):
            start = i + 1
            j = start
            while j < len(lines) and lines[j].strip() != marker:
                j += 1
            blocks.append((start + 1, "\n".join(lines[start:j])))
            i = j
    i += 1

ok = bad = 0
for lineno, code in blocks:
    if not code.strip():
        continue
    try:
        ast.parse(code)
        ok += 1
    except SyntaxError as e:
        bad += 1
        print(f"  PY SYNTAX ERROR: file line {lineno}, py line {e.lineno}: {e.msg}")
        print("     >>>", (code.splitlines() or [""])[e.lineno - 1] if e.lineno else "")

print(f"{path}: {len(blocks)} heredocs, {ok} parse OK, {bad} bad")
sys.exit(1 if bad else 0)