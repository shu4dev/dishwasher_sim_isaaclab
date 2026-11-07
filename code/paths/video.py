"""video stage (plans/2026-11-01-video-port.md): a stop-motion video of every recorded episode that lacks one.

For each row with a record in the sample's runs file (<out>/runs/<sample>.json): frigidaire_planner_video.py (Isaac RTX
on the bench kit's scene, kinematic dishes at the record's poses, no physics) writes <media>/<sample>__<track>__<algo>.mp4,
the media folder being the runs file's, and the row's video path goes back into the runs file. An existing mp4 is
kept, so a rerun renders only the missing ones and deleting an mp4 forces a new render. A record with no executed move
(load-not-buildable: the episode never ran) has nothing to show and is skipped. The stage stops at the first failed
render. PASS when every record with an executed move has a video.
"""
import json
from pathlib import Path

from sampler import ROOT, rel, run
from sampler.sample import read_run, write_run

VIDEO = "code/paths/scripts/frigidaire_planner_video.py"


def stage_video(state, params):
    r = read_run(params["sample"]) or {}
    rows = [row for row in r.get("episodes") or [] if row.get("record")]
    if not rows:
        return state, "FAIL", "no run record (run the run stage first)"
    pid = Path(params["sample"]).stem
    media, logs = ROOT / r["media"], Path(params["work"]) / "run" / "logs"
    wanted, skipped, failed = [], [], None
    for row in rows:
        rec = json.loads((ROOT / row["record"]).read_text())
        if not any("poses_after" in m for m in rec.get("moves") or []):
            skipped.append(f"{row['algorithm']} ({rec.get('abort')}, 0 moves)")
            continue
        wanted.append(row)
        vid = media / f"{pid}__{row['track']}__{row['algorithm']}.mp4"
        if not vid.is_file() and failed is None:
            ok, _ = run("kit", [VIDEO, "--instance", r["instance"], "--episode", row["record"], "--out-dir", rel(media),
                                "--video-width", "640", "--headless", "--enable_cameras"], logs / f"video_{row['algorithm']}.log")
            if not ok or not vid.is_file():
                failed = row["algorithm"]
        row["video"] = rel(vid) if vid.is_file() else None
    write_run(params["sample"], r)
    done = [row["algorithm"] for row in wanted if row["video"]]
    note = f"{len(done)} of {len(wanted)} videos" + (f"; skipped {', '.join(skipped)}" if skipped else "")
    if failed:
        note += f"; stopped at {failed} (log {rel(logs / f'video_{failed}.log')})"
    return state, "PASS" if len(done) == len(wanted) else "FAIL", note
