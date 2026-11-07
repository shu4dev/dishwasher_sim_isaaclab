"""The benchmark (plans/2026-11-01-merged-benchmark.md; the sampler until 2026-11-01): one sample file per (roster, seed)
with four blocks, metadata, initial, goal and dag (plans/2026-11-02-sample-schema.md), then the four algorithms' episodes
in runs/ and the report page. CLI: code/planner/bench.py.

Stages are ``stage(state, params) -> (state, result, note)`` over the sample (sample.py writes it after each). Importing the
package imports nothing heavy; Kit stages run the existing Kit scripts by subprocess through code/util/run_kit.sh.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def rel(path):
    """A path as the container sees it: repo-relative when under the repo (code/util/run_*.sh map the cwd)."""
    path = Path(path).absolute()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


CLOSE_GRACE_S = 90.     # a Kit process still alive this long after its [RESULT] line hangs in app.close() (6.2, 2026-11-01)


def run(kind, argv, log):
    """Run one script through code/util/run_kit.sh (kind 'kit') or run_py.sh ('py') from the repo root, output to ``log``.
    Returns (ok, text): ok = '[RESULT] PASS' printed, no Traceback, no 'free(): invalid pointer' (the exit code of
    isaaclab.sh says nothing). On Isaac Sim 6.2 a Frigidaire Kit process can hang in app.close() after it printed its
    [RESULT] line and wrote its records: it is killed CLOSE_GRACE_S after the log last changed with that line in it."""
    import time
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as f:
        proc = subprocess.Popen([f"code/util/run_{kind}.sh", *map(str, argv)], cwd=ROOT, stdout=f, stderr=subprocess.STDOUT,
                                start_new_session=True)            # its own process group: python.sh forks the python
        while proc.poll() is None:
            time.sleep(5.)
            if kind == "kit" and time.time() - log.stat().st_mtime > CLOSE_GRACE_S \
                    and "[RESULT]" in log.read_text(errors="replace")[-20000:]:
                import os
                import signal
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    text = log.read_text(errors="replace")
    return ("[RESULT] PASS" in text and "Traceback" not in text and "free(): invalid pointer" not in text), text
