#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Sample one dish in either Frigidaire rack and physically test retraction.

    scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py --headless

Host-only asset validation (no physics or accepted-pose claims):
    python3 frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py --preflight-only
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import math
from pathlib import Path
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_USD = ROOT/"build/frigidaire_collection/usd/fdpc4221as.usdc"


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def parser_for_experiment():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
    parser.add_argument("--samples-per-cell", type=positive_int, default=100,
                        help="Raw proposals per dish/rack combination, including initial collisions (default 100)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-wall-seconds", type=positive_seconds, default=1800.)
    parser.add_argument("--resume-from", type=Path,
                        help="Continue a finalized compatible run in a new output directory; retry unresolved samples")
    parser.add_argument("--out-dir", type=Path,
                        default=ROOT/"build/frigidaire_random_poses"/datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
    parser.add_argument("--preflight-only", action="store_true", help="Validate assets on the host; do not run physics")
    parser.add_argument("--no-plots", action="store_true", help="Omit summary plots")
    return parser


def add_minimal_launcher_arguments(parser):
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--device", default="cpu", help="Physics device (default cpu, matching existing Frigidaire evidence)")


def add_paths():
    sys.path.insert(0, str(ROOT/"src"))
    sys.path.insert(0, str(ROOT/"frigidaire/src"))


def source_hashes():
    sources = [Path(__file__).resolve(), ROOT/"src/dishsim/quats.py"]
    directory = ROOT/"frigidaire/src/dishsim_frigidaire"
    sources += sorted(directory.glob("random_pose*.py"))
    sources += [directory/(name+".py") for name in ("asset", "geometry", "tableware", "loading", "load_validation")]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}


def run(args, started, *, launcher_class=None, import_error=None):
    # Pure imports are safe without Kit in preflight/failure reporting. During a
    # physics run AppLauncher precedes every project/scene/USD import.
    app = None
    launch_error = import_error
    if launcher_class is not None:
        try:
            app = launcher_class(args).app
        except Exception as exc:
            launch_error = exc
    add_paths()
    from dishsim_frigidaire.random_pose_reports import ExperimentReport
    from dishsim_frigidaire.random_pose_assets import validate_inputs
    from dishsim_frigidaire.random_poses import KINDS, RACKS, LIMITS, source_geometry_domains

    args.usd, args.out_dir = args.usd.resolve(), args.out_dir.resolve()
    metadata = {
        "started_utc": datetime.now(timezone.utc).isoformat(), "seed": args.seed,
        "samples_per_cell": args.samples_per_cell, "max_wall_seconds": args.max_wall_seconds,
        "kinds": list(KINDS), "racks": list(RACKS), "thresholds": LIMITS,
        "source_hashes": source_hashes(), "door_closure_tested": False,
        "sampling": "independent uniform XYZ rotated-mesh-bounds center and uniform SO(3) rotation",
        "interpretation": "Counts are raw random drops accepted by this protocol, not unique resting arrangements or maximum capacity.",
        "runtime_status": "NOT_RUN", "arguments": {key: str(value) if isinstance(value, Path) else value
                                                      for key, value in vars(args).items()},
    }
    baselines = {rack: {"result": "NOT_RUN"} for rack in RACKS}
    outcome = {"status": "not_run", "baselines": baselines}
    report = None
    resume_state = None
    inherited_count = 0
    summary = None
    exit_code = 1
    try:
        if args.resume_from is not None:
            args.resume_from = args.resume_from.resolve()
            if (args.out_dir == args.resume_from or args.resume_from in args.out_dir.parents
                    or args.out_dir in args.resume_from.parents):
                raise ValueError("Resume output and previous run must be separate, non-nested directories")
        report = ExperimentReport(args.out_dir, samples_per_cell=args.samples_per_cell)
        report.write_metadata(metadata)
        if args.seed < 0:
            raise ValueError("seed must be nonnegative")
        metadata["inputs"] = validate_inputs(args.usd)
        metadata["domains"] = source_geometry_domains()
        if args.resume_from is not None:
            from dishsim_frigidaire.random_pose_resume import load_resume
            resume_state = load_resume(args.resume_from, metadata)
            metadata["resume"] = dict(resume_state.provenance, applied=False)
        report.write_metadata(metadata)
        if args.preflight_only:
            outcome["status"] = "preflight_only"
            metadata["preflight"] = "PASS"
            print("[PREFLIGHT] PASS: current assets, catalog, and sampling domains validated; physics NOT_RUN", flush=True)
        elif launch_error is not None:
            outcome.update(status="runtime_unavailable", error=repr(launch_error))
            metadata["runtime_error"] = repr(launch_error)
        else:
            from dishsim_frigidaire.random_pose_experiment import execute_trials, check_deadline
            from dishsim_frigidaire.random_pose_runtime import IsaacPoseBackend
            deadline = started+args.max_wall_seconds
            check_deadline(deadline)
            backend = IsaacPoseBackend(args.usd, args.out_dir, device=args.device,
                                       domains=metadata["domains"], deadline=deadline, app=app)
            version_path = Path("/isaac-sim/VERSION")
            metadata["runtime"] = dict(backend.runtime_settings,
                isaac_sim=version_path.read_text().strip() if version_path.is_file() else "unknown",
                isaac_lab=importlib.metadata.version("isaaclab"))
            metadata["runtime_status"] = "STARTED"
            if resume_state is not None:
                from dishsim_frigidaire.random_pose_resume import (
                    seed_report, validate_runtime, validate_sample_stream,
                )
                validate_runtime(resume_state, metadata["runtime"])
                validate_sample_stream(resume_state, backend.points, metadata["domains"])
                seed_report(report, resume_state)
                inherited_count = len(resume_state.completed_trials)
                metadata["resume"]["applied"] = True
                print("[RESUME] inherited {} classified trials; retrying {} unresolved samples".format(
                    inherited_count, len(resume_state.unresolved_trials)), flush=True)
            report.write_metadata(metadata)
            outcome = execute_trials(backend, report, domains=metadata["domains"],
                                      samples_per_cell=args.samples_per_cell, seed=args.seed,
                                      deadline=deadline, progress=lambda text: print(text, flush=True),
                                      completed_trials=resume_state.completed_trials if resume_state is not None else None)
    except KeyboardInterrupt:
        outcome.update(status="interrupted", error="User interrupted initialization")
    except Exception as exc:
        traceback.print_exc()
        outcome.update(status="budget_exhausted" if isinstance(exc, TimeoutError) else "error", error=repr(exc))
    finally:
        try:
            metadata["wall_seconds"] = time.monotonic()-started
            metadata["cumulative_wall_seconds"] = metadata["wall_seconds"] + (
                resume_state.prior_wall_seconds if resume_state is not None else 0.)
            if resume_state is not None and report is not None:
                # A failed history copy may have imported only part of the prior
                # evidence; imported rows are never new evaluation attempts.
                inherited_count = sum(trial["trial_id"] in resume_state.completed_trials
                                      for trial in report.trials)
            new_attempts = len(report.trials)-inherited_count if report is not None else 0
            metadata["inherited_classified_trials"] = inherited_count
            metadata["new_evaluation_attempts"] = new_attempts
            metadata["cumulative_evaluation_attempts"] = new_attempts + (
                resume_state.prior_evaluation_attempts if resume_state is not None else 0)
            metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
            metadata["run_status"] = outcome["status"]
            if report is not None:
                report.write_metadata(metadata)
                summary = report.finalize(
                    baselines=outcome["baselines"], status=outcome["status"],
                    extra={"wall_seconds": metadata["wall_seconds"],
                           "cumulative_wall_seconds": metadata["cumulative_wall_seconds"],
                           "inherited_classified_trials": inherited_count,
                           "new_evaluation_attempts": new_attempts,
                           "cumulative_evaluation_attempts": metadata["cumulative_evaluation_attempts"],
                           "error": outcome.get("error")},
                    plots=not args.no_plots)
                totals = summary["totals"]
                print("[COUNTS] accepted={accepted} attempted={attempted} unresolved={unresolved} "
                      "unattempted={unattempted}".format(**totals), flush=True)
                print("[REPORT] "+str(args.out_dir/"report.md"), flush=True)
            status = outcome["status"]
            marker = "NOT_RUN" if status in {"preflight_only", "runtime_unavailable", "not_run"} else (
                "PASS" if status == "complete" and summary is not None and not summary["totals"]["unresolved"]
                else "INCOMPLETE")
            print("[RESULT] {}: {}{}".format(
                marker, status, "; "+outcome["error"] if outcome.get("error") else ""), flush=True)
            exit_code = 0 if status in {"complete", "preflight_only"} and marker != "INCOMPLETE" else 1
        finally:
            if report is not None:
                report.close()
            if app is not None:
                from dishsim.media import release_sim_for_close
                release_sim_for_close()
                app.close()
    return exit_code


def main(argv=None):
    started = time.monotonic()
    argv = sys.argv[1:] if argv is None else argv
    parser = parser_for_experiment()
    if any(flag in argv for flag in ("--preflight-only", "--help", "-h")):
        add_minimal_launcher_arguments(parser)
        return run(parser.parse_args(argv), started)
    try:
        from isaaclab.app import AppLauncher
    except ImportError as exc:
        add_minimal_launcher_arguments(parser)
        return run(parser.parse_args(argv), started, import_error=exc)
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    return run(parser.parse_args(argv), started, launcher_class=AppLauncher)


if __name__ == "__main__":
    sys.exit(main())
