"""The sampler: sample(dishwasher, dish set, seed) -> one pair file (initial + goal configuration), stage by stage.

Stages are ``stage(state, params) -> (state, result, note)`` over the pair (sample.py writes it after each). Importing the
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


def run(kind, argv, log):
    """Run one script through code/util/run_kit.sh (kind 'kit') or run_py.sh ('py') from the repo root, output to ``log``.
    Returns (ok, text): ok = '[RESULT] PASS' printed, no Traceback, no 'free(): invalid pointer' (the exit code of
    isaaclab.sh says nothing)."""
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as f:
        subprocess.run([f"code/util/run_{kind}.sh", *map(str, argv)], cwd=ROOT, stdout=f, stderr=subprocess.STDOUT)
    text = log.read_text(errors="replace")
    return ("[RESULT] PASS" in text and "Traceback" not in text and "free(): invalid pointer" not in text), text
