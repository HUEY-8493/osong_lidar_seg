#!/usr/bin/env python3
"""Offline A/B evaluator for a *post-association* first-order IIR filter.

Input is a time-ordered CSV with at least ``stamp,id,x,y``.  Optional columns
are ``z,length,width,height,truth_x,truth_y,truth_z,truth_id,detected``.
One row represents one associated track measurement at one LiDAR stamp.
The program intentionally does not change the perception node or rosbag.
"""
import argparse
import csv
import math
import statistics
from collections import defaultdict


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def percentile(values, p):
    if not values:
        return float("nan")
    values = sorted(values)
    k = (len(values) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return values[lo] + (values[hi] - values[lo]) * (k - lo)


def rms(values):
    return math.sqrt(mean([v * v for v in values])) if values else float("nan")


def parse_alphas(value):
    result = []
    for token in value.split(","):
        alpha = float(token)
        if not 0.0 < alpha <= 1.0:
            raise argparse.ArgumentTypeError("all alpha values must be in (0, 1]")
        result.append(alpha)
    return result


def field(row, name, default=None):
    value = row.get(name, "")
    return default if value is None or value == "" else float(value)


def load_rows(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    need = {"stamp", "id", "x", "y"}
    if not rows or not need.issubset(rows[0]):
        raise ValueError("CSV must contain: stamp,id,x,y")
    parsed = []
    for row in rows:
        parsed.append({
            "stamp": field(row, "stamp"), "id": row["id"], "x": field(row, "x"),
            "y": field(row, "y"), "z": field(row, "z", 0.0),
            "truth_x": field(row, "truth_x"), "truth_y": field(row, "truth_y"),
            "truth_z": field(row, "truth_z"), "truth_id": row.get("truth_id", ""),
            "detected": row.get("detected", "1") not in ("0", "false", "False"),
        })
    return sorted(parsed, key=lambda r: (r["stamp"], r["id"]))


def evaluate(rows, alpha, reset_gap):
    state, previous_stamp = {}, {}
    raw_steps, filtered_steps, raw_errors, filtered_errors = [], [], [], []
    samples_by_truth, ids_by_truth = defaultdict(list), defaultdict(set)
    filtered = []
    for r in rows:
        key, stamp = r["id"], r["stamp"]
        measurement = (r["x"], r["y"], r["z"])
        reset = key not in state or stamp - previous_stamp[key] > reset_gap
        value = measurement if reset else tuple(
            alpha * m + (1.0 - alpha) * old for m, old in zip(measurement, state[key]))
        if not reset:
            raw_steps.append(math.dist(measurement, state["raw:" + key]))
            filtered_steps.append(math.dist(value, state[key]))
        state[key], state["raw:" + key], previous_stamp[key] = value, measurement, stamp
        q = dict(r)
        q["fx"], q["fy"], q["fz"] = value
        filtered.append(q)
        if r["truth_x"] is not None and r["truth_y"] is not None:
            raw_errors.append(math.dist(measurement, (r["truth_x"], r["truth_y"], r["truth_z"] or 0.0)))
            filtered_errors.append(math.dist(value, (r["truth_x"], r["truth_y"], r["truth_z"] or 0.0)))
        if r["truth_id"]:
            samples_by_truth[r["truth_id"]].append(r)
            ids_by_truth[r["truth_id"]].add(key)
    # A 90%-response delay needs truth data.  Find the first point after each
    # truth displacement and measure when the filtered output reaches 90% of it.
    delays = []
    by_id = defaultdict(list)
    for r in filtered:
        if r["truth_x"] is not None and r["truth_y"] is not None:
            by_id[r["id"]].append(r)
    for series in by_id.values():
        for a, b in zip(series, series[1:]):
            truth_step = math.dist((a["truth_x"], a["truth_y"]), (b["truth_x"], b["truth_y"]))
            if truth_step < 0.25:
                continue
            target = 0.9 * truth_step
            for c in series[series.index(b):]:
                if math.dist((c["fx"], c["fy"]), (a["fx"], a["fy"])) >= target:
                    delays.append(1000.0 * (c["stamp"] - b["stamp"]))
                    break
    return {
        "raw_jitter_m": rms(raw_steps), "iir_jitter_m": rms(filtered_steps),
        "raw_rmse_m": rms(raw_errors), "iir_rmse_m": rms(filtered_errors),
        "delay90_ms": percentile(delays, 0.5),
        "truth_id_switch_proxy": sum(max(0, len(v) - 1) for v in ids_by_truth.values()),
        "samples": len(rows),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="time-ordered associated-track measurements")
    parser.add_argument("--alphas", default="1.0,0.8,0.6,0.4,0.2,0.1", type=parse_alphas)
    parser.add_argument("--reset-gap-sec", type=float, default=0.25,
                        help="reset state after this per-ID measurement gap (default: 0.25)")
    args = parser.parse_args()
    rows = load_rows(args.csv)
    print("alpha,samples,raw_jitter_m,iir_jitter_m,raw_rmse_m,iir_rmse_m,delay90_ms,truth_id_switch_proxy")
    for alpha in args.alphas:
        m = evaluate(rows, alpha, args.reset_gap_sec)
        print("{alpha:.3f},{samples},{raw_jitter_m:.6f},{iir_jitter_m:.6f},{raw_rmse_m:.6f},{iir_rmse_m:.6f},{delay90_ms:.3f},{truth_id_switch_proxy}".format(alpha=alpha, **m))


if __name__ == "__main__":
    main()
