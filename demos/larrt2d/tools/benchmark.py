#!/usr/bin/env python3
"""Benchmark LA-RRT (and baselines) on the 2D rearrangement scenes.

For every scene x planner, run the driver R times (LA-RRT is randomized), validate each
solution with the independent checker, and tabulate success rate, best/median action count,
and median solve time. This is the pipeline you extend when trying new planners or LA-RRT
improvements: add a planner name to PLANNERS (must be handled by the C++ driver) and rerun.

Usage:
    python3 demos/larrt2d/tools/benchmark.py [--runs R] [--time T] [--planners a,b] [--scenes x,y]

Run from the repository root.
"""
import argparse
import glob
import json
import os
import statistics
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DRIVER = os.path.join(ROOT, "build", "demos", "demo_LARRT2D_puzzle")
SCENES_DIR = os.path.join(ROOT, "demos", "larrt2d", "scenes")
OUT_DIR = os.path.join(ROOT, "demos", "larrt2d", "out")
VALIDATOR = os.path.join(ROOT, "demos", "larrt2d", "tools", "validate_puzzle.py")


def run_once(scene_path, planner, plan_time):
    """Run the driver once; return (solved, action_count, solve_time, valid, out_json)."""
    name = os.path.splitext(os.path.basename(scene_path))[0]
    out_json = os.path.join(OUT_DIR, name + ".json")
    try:
        subprocess.run([DRIVER, scene_path, str(plan_time), planner],
                       cwd=ROOT, capture_output=True, timeout=plan_time + 30)
    except subprocess.TimeoutExpired:
        return (False, -1, plan_time, False, out_json)
    if not os.path.exists(out_json):
        return (False, -1, 0.0, False, out_json)
    with open(out_json) as f:
        d = json.load(f)
    solved = bool(d.get("solved", False))
    ac = d.get("action_count", -1)
    t = d.get("solve_time_s", 0.0)
    valid = False
    if solved:
        v = subprocess.run([sys.executable, VALIDATOR, out_json],
                           cwd=ROOT, capture_output=True, text=True)
        valid = (v.returncode == 0)
    return (solved, ac, t, valid, out_json)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--time", type=float, default=10.0)
    ap.add_argument("--planners", default="larrt,rrtconnect")
    ap.add_argument("--scenes", default="")
    args = ap.parse_args()

    if not os.path.exists(DRIVER):
        sys.exit(f"driver not built: {DRIVER}\n  build it: cmake --build build --target demo_LARRT2D_puzzle -j4")

    planners = [p.strip() for p in args.planners.split(",") if p.strip()]
    if args.scenes:
        scenes = [os.path.join(SCENES_DIR, s if s.endswith(".json") else s + ".json")
                  for s in args.scenes.split(",")]
    else:
        scenes = sorted(glob.glob(os.path.join(SCENES_DIR, "*.json")))

    print(f"driver: {DRIVER}")
    print(f"runs/config: {args.runs}   plan_time: {args.time}s   planners: {planners}\n")
    hdr = f"{'scene':<18} {'planner':<12} {'solved':>7} {'valid':>6} {'best':>5} {'median':>7} {'med_time_s':>11}"
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for scene in scenes:
        sname = os.path.splitext(os.path.basename(scene))[0]
        for planner in planners:
            acs, times, nsolved, nvalid = [], [], 0, 0
            for _ in range(args.runs):
                solved, ac, t, valid, _ = run_once(scene, planner, args.time)
                if solved:
                    nsolved += 1
                    acs.append(ac)
                    times.append(t)
                    nvalid += int(valid)
            best = min(acs) if acs else -1
            med = int(statistics.median(acs)) if acs else -1
            medt = round(statistics.median(times), 3) if times else -1
            print(f"{sname:<18} {planner:<12} {nsolved:>4}/{args.runs} {nvalid:>4}/{max(nsolved,0):<1} "
                  f"{best:>5} {med:>7} {medt:>11}")
            rows.append(dict(scene=sname, planner=planner, solved=nsolved, runs=args.runs,
                             valid=nvalid, best=best, median=med, median_time_s=medt))

    out = os.path.join(OUT_DIR, "benchmark_results.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
