#!/usr/bin/env python3
"""Certification benchmark for generated scene families (HANDOFF T1 step 2/3).

For each scene it records the full Q5 bracket and the T1 cross-check:

  * derived optimum        -- from the generator's _meta (proven or conjectured)
  * sound lower bound (LB)  -- monotonicity.py analyse() on the SOUND graph
  * best planner UB         -- best-of-N validated LA-RRT action counts
  * certified?              -- LB == UB (and the best run validated)
  * LB-vs-optimum check     -- flags a mismatch between the generator's claimed
                               optimum and the sound LB (exactly one is buggy;
                               that is the investigation trigger of T1 step 3).

Results go to out/benchmark_certify.json and a table to stdout.

Run from the repository root:
    python3 demos/larrt2d/tools/bench_certify.py [--runs N] [--time T]
            [--glob 'scenes/gen/*.json'] [--planner larrt]

Heavy batch execution over ~hundreds of instances is the part to DELEGATE to a
cheaper subagent (see HANDOFF §0); the bracket logic and any mismatch analysis
stay with the orchestrator.
"""
import argparse
import glob
import json
import os
import shutil
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)

from benchmark import run_once                      # noqa: E402
from monotonicity import build_graphs, analyse, mrb_refine, SWEEP_SAMPLES  # noqa: E402


def sound_lb(scene, mrb=False):
    """Sound-graph action lower bound for a parsed scene dict (T4-refined if mrb)."""
    info, sound_arcs, _swept, warns = build_graphs(scene, SWEEP_SAMPLES)
    names = [o["name"] for o in scene["objects"]]
    res = analyse(names, info, sound_arcs)
    lb = res["action_lower_bound"]
    if mrb:
        lb, _ = mrb_refine(scene, lb)
    return lb, warns


def derived_optimum(meta):
    """(value, status) of the generator's claimed optimum, or (None, '')."""
    if not meta:
        return None, ""
    if "optimum" in meta:
        return meta["optimum"], meta.get("optimum_status", "proven")
    if "optimum_conjecture" in meta:
        return meta["optimum_conjecture"], meta.get("optimum_status", "conjectured")
    return None, meta.get("optimum_status", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--time", type=float, default=8.0)
    ap.add_argument("--glob", default="demos/larrt2d/scenes/gen/*.json")
    ap.add_argument("--planner", default="larrt")
    ap.add_argument("--out", default="demos/larrt2d/out/benchmark_certify.json")
    ap.add_argument("--mrb", action="store_true",
                    help="use the T4 MRB-refined sound LB (buffer-scarcity tightening)")
    args = ap.parse_args()

    scenes = sorted(glob.glob(os.path.join(ROOT, args.glob)))
    if not scenes:
        sys.exit("no scenes matched %r" % args.glob)

    print("planner=%s  runs=%d  time=%.1fs  scenes=%d\n" %
          (args.planner, args.runs, args.time, len(scenes)))
    hdr = ("%-22s %-8s %4s %4s %4s %5s %-9s %s" %
           ("scene", "family", "opt", "LB", "UB", "cert?", "opt_stat", "note"))
    print(hdr); print("-" * len(hdr))

    rows, n_cert, n_mismatch = [], 0, 0
    for path in scenes:
        with open(path) as f:
            scene = json.load(f)
        sname = os.path.splitext(os.path.basename(path))[0]
        meta = scene.get("_meta", {})
        fam = meta.get("family", "?")
        opt, ostat = derived_optimum(meta)
        lb, warns = sound_lb(scene, mrb=args.mrb)

        acs, times, nvalid = [], [], 0
        best_ac, best_solution = None, None
        for _ in range(args.runs):
            solved, ac, t, valid, out_json = run_once(path, args.planner, args.time)
            if solved and valid:
                acs.append(ac); times.append(t); nvalid += 1
                # run_once overwrites out/<scene>.json each run, so preserve the
                # best VALIDATED solution the moment it is produced (T1 item 7:
                # reproducibility + figures need the best, not the last, run).
                if best_ac is None or ac < best_ac:
                    best_ac = ac
                    best_solution = os.path.join(os.path.dirname(out_json),
                                                 sname + "_best.json")
                    shutil.copyfile(out_json, best_solution)
        ub = min(acs) if acs else -1

        certified = (ub > 0 and lb == ub)
        n_cert += int(certified)
        # T1 step-3 cross-check: proven optimum must equal the sound LB.
        note = ""
        if opt is not None and ostat.startswith("proven") and opt != lb:
            note = "LB!=proven_opt(%d)" % opt; n_mismatch += 1
        elif opt is not None and ub > 0 and opt != ub:
            note = "UB!=opt(%d)" % opt
        if warns:
            note = (note + " " if note else "") + "warn:%d" % len(warns)

        print("%-22s %-8s %4s %4d %4d %5s %-9s %s" %
              (sname, fam, opt if opt is not None else "-", lb, ub,
               "YES" if certified else "no", ostat[:9], note))
        rows.append(dict(scene=sname, family=fam, params={k: v for k, v in meta.items()
                                                          if k not in ("family",)},
                         derived_optimum=opt, optimum_status=ostat,
                         sound_lb=lb, best_ub=ub, valid_runs=nvalid, runs=args.runs,
                         certified=certified,
                         best_solution=(os.path.relpath(best_solution, ROOT)
                                        if best_solution else None),
                         median_time_s=round(statistics.median(times), 3) if times else -1,
                         note=note))

    out = os.path.join(ROOT, args.out)
    with open(out, "w") as f:
        json.dump(dict(planner=args.planner, runs=args.runs, time_s=args.time,
                       n_scenes=len(scenes), n_certified=n_cert,
                       n_mismatch=n_mismatch, rows=rows), f, indent=2)
    print("\ncertified %d/%d   mismatches %d   -> %s" %
          (n_cert, len(scenes), n_mismatch, out))


if __name__ == "__main__":
    main()
