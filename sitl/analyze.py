#!/usr/bin/env python3
"""Measure the SITL runs against the fence, and optionally animate them.

    analyze.py <out_dir> [--check] [--gif docs/sitl.gif]

Reads <out_dir>/{unguarded,guarded}.csv as written by planner.py. Prints, for
each run, how long the vehicle spent outside the fence and how far out it
got, measured from the logged odometry. --check exits non-zero if the guarded
run ever left the fence; that is the CI assertion. --gif renders both runs
side by side from the same logs.
"""

import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
AIRBORNE_Z = 1.0


def load_envelope(path):
    env = {"fence": [], "zones": {}}
    current = None
    with open(path) as f:
        for raw in f:
            t = raw.split("#", 1)[0].split()
            if not t:
                continue
            if current is not None:
                if t[0] == "end":
                    current = None
                else:
                    current.append((float(t[0]), float(t[1])))
            elif t[0] == "fence":
                current = env["fence"]
            elif t[0] == "exclude":
                current = env["zones"].setdefault(t[1], [])
            elif t[0] == "margin":
                env["margin"] = float(t[1])
    return env


def inside(p, poly):
    x, y = p
    result = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < xi + (y - yi) * (xj - xi) / (yj - yi):
            result = not result
        j = i
    return result


def dist_to_boundary(p, poly):
    best = math.inf
    for i in range(len(poly)):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % len(poly)]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy))
    return best


def clearance(p, poly, keep_in=True):
    d = dist_to_boundary(p, poly)
    return d if inside(p, poly) == keep_in else -d


def load_run(out_dir, name):
    rows = []
    with open(os.path.join(out_dir, f"{name}.csv")) as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": float(r["t"]), "x": float(r["x"]), "y": float(r["y"]), "z": float(r["z"]),
                "wp": int(r["wp"]), "target": (float(r["tx"]), float(r["ty"]), float(r["tz"])),
                "action": r["action"], "codes": r["codes"], "mode": r["mode"],
            })
    with open(os.path.join(out_dir, f"{name}-events.json")) as f:
        events = json.load(f)
    return rows, events


def measure(rows, env):
    outside_s = 0.0
    worst_out = 0.0
    min_clear = math.inf
    zone_min = math.inf
    prev_t = None
    for r in rows:
        if r["z"] < AIRBORNE_Z:
            prev_t = r["t"]
            continue
        c = clearance((r["x"], r["y"]), env["fence"])
        min_clear = min(min_clear, c)
        for poly in env["zones"].values():
            zone_min = min(zone_min, clearance((r["x"], r["y"]), poly, keep_in=False))
        if c < 0:
            worst_out = max(worst_out, -c)
            if prev_t is not None:
                outside_s += r["t"] - prev_t
        prev_t = r["t"]
    return {
        "seconds_outside_fence": round(outside_s, 1),
        "furthest_outside_m": round(worst_out, 1),
        "min_fence_clearance_m": round(min_clear, 2),
        "min_exclusion_clearance_m": round(zone_min, 2),
    }


# --- animation ---------------------------------------------------------------

def summary_for(title, results):
    r = results["guarded" if "guard" in title.lower() and "without" not in title.lower() else "unguarded"]
    if r["seconds_outside_fence"] > 0:
        return (f"left the fence for {r['seconds_outside_fence']:.0f} s, "
                f"up to {r['furthest_outside_m']:.0f} m outside")
    return f"never left the fence (closest {r['min_fence_clearance_m']:.1f} m)"


def animate(runs, env, path, results, speed=6.0, fps=10):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon as Patch

    ok, bad, band, ink, muted = "#1a7f37", "#cf222e", "#d4a72c", "#24292f", "#57606a"
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 5.6), dpi=80)
    fig.patch.set_facecolor("white")
    margin = env["margin"]

    states = []
    for ax, (title, rows, events) in zip(axes, runs):
        ax.set_aspect("equal")
        ax.set_xlim(-30, 310)
        ax.set_ylim(-30, 260)
        ax.axis("off")
        fence = Patch(env["fence"], closed=True, facecolor="#f0f7f0", edgecolor=ink, lw=1.4, zorder=1)
        ax.add_patch(fence)
        edge = Patch(env["fence"], closed=True, fill=False, edgecolor=band, alpha=0.45, zorder=2,
                     joinstyle="round")
        ax.add_patch(edge)
        edge.set_clip_path(fence)
        for name, poly in env["zones"].items():
            ax.add_patch(Patch(poly, closed=True, fill=False, edgecolor=band, alpha=0.45, zorder=2,
                               joinstyle="round", lw=1))
            ax.add_patch(Patch(poly, closed=True, facecolor="#ffebe9", edgecolor=bad, lw=1.2, zorder=3))
            cx = sum(p[0] for p in poly) / len(poly)
            cy = max(p[1] for p in poly)
            ax.text(cx, cy + 8, name, color=bad, fontsize=8, ha="center", zorder=3)
        ax.set_title(title, fontsize=13, fontweight="bold", color=ink, loc="left")

        arm_t = next(e["t"] for e in events if e["event"] == "armed in OFFBOARD")
        end_t = next((e["t"] for e in events if e["event"] in ("disarmed", "land timed out")),
                     rows[-1]["t"])
        flight = [r for r in rows if arm_t <= r["t"] <= end_t]
        for r in flight:
            r["out"] = clearance((r["x"], r["y"]), env["fence"])
        trail_in, = ax.plot([], [], color=ok, lw=2.2, zorder=4)
        trail_out, = ax.plot([], [], color=bad, lw=2.6, zorder=5)
        aim, = ax.plot([], [], color=muted, lw=1.2, ls=(0, (4, 3)), zorder=4)
        target, = ax.plot([], [], marker="x", color=muted, ms=9, mew=2, zorder=6, ls="none")
        # The refused leg stays on screen once the guard has refused it.
        refused, = ax.plot([], [], color=bad, lw=1.6, ls=(0, (6, 4)), zorder=4)
        refused_x, = ax.plot([], [], marker="x", color=bad, ms=10, mew=2.5, zorder=6, ls="none")
        refused_label = ax.text(0, 0, "", fontsize=9, color=bad, fontweight="bold", zorder=8,
                                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=bad, lw=0.8))
        vehicle, = ax.plot([], [], marker="o", color=ink, ms=8, zorder=7, ls="none")
        status = ax.text(-25, -27, "", fontsize=10, color=ink, va="bottom", zorder=8)
        clock = ax.text(305, -27, "", fontsize=9, color=muted, ha="right", va="bottom", zorder=8)
        states.append({"ax": ax, "rows": flight, "arm": arm_t, "edge": edge, "trail_in": trail_in,
                       "refused": refused, "refused_x": refused_x, "refused_label": refused_label,
                       "refused_leg": None, "summary": summary_for(title, results),
                       "summary_colour": ok if summary_for(title, results).startswith("never") else bad,
                       "trail_out": trail_out, "aim": aim, "target": target, "vehicle": vehicle,
                       "status": status, "clock": clock, "events": events})

    # Margin band width in points: data metres → display points.
    def set_band_widths():
        for s in states:
            ax = s["ax"]
            p0 = ax.transData.transform((0, 0))
            p1 = ax.transData.transform((margin * 2, 0))
            s["edge"].set_linewidth((p1[0] - p0[0]) * 72.0 / fig.dpi)
        for ax in axes:
            for patch in ax.patches[2:]:
                if patch.get_fill() is False:
                    p0 = ax.transData.transform((0, 0))
                    p1 = ax.transData.transform((margin * 2, 0))
                    patch.set_linewidth((p1[0] - p0[0]) * 72.0 / fig.dpi)

    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.canvas.draw()
    set_band_widths()
    fig.text(0.5, 0.02,
             "PX4 1.15 SITL telemetry, replayed at %gx (the refusal at 1x). Same planner, same "
             "mission; the right side routes its setpoints through geofence-guard." % speed,
             ha="center", fontsize=9, color=muted)

    duration = max(s["rows"][-1]["t"] - s["arm"] for s in states)
    # Play at `speed`, except while any run is being refused: those seconds
    # play in real time so the hold is visible. End on 4 s of the final frame.
    holding = [r["t"] - s["arm"] for s in states for r in s["rows"] if r["action"] == "hold"]
    times, t = [], 0.0
    while t < duration:
        times.append(t)
        slow = any(h - 0.5 <= t <= h + 0.5 for h in holding)
        t += (1.0 if slow else speed) / fps
    times += [duration] * (fps * 4)

    def frame(i):
        t_rel = times[i]
        artists = []
        for s in states:
            rows = [r for r in s["rows"] if r["t"] - s["arm"] <= t_rel]
            if not rows:
                continue
            xin, yin, xout, yout = [], [], [], []
            for r in rows:
                out = r["out"] < 0
                (xout if out else xin).append(r["x"])
                (yout if out else yin).append(r["y"])
                # Break the other trail so it does not draw a chord across.
                (xin if out else xout).append(math.nan)
                (yin if out else yout).append(math.nan)
            s["trail_in"].set_data(xin, yin)
            s["trail_out"].set_data(xout, yout)
            r = rows[-1]
            s["vehicle"].set_data([r["x"]], [r["y"]])
            if r["action"] == "hold" and s["refused_leg"] is None:
                s["refused_leg"] = ((r["x"], r["y"]), r["target"], r["codes"])
            if s["refused_leg"] is not None:
                (fx, fy), (tx, ty, _), codes = s["refused_leg"]
                s["refused"].set_data([fx, tx], [fy, ty])
                s["refused_x"].set_data([tx], [ty])
                s["refused_label"].set_position(((fx + tx) / 2 + 6, (fy + ty) / 2 + 8))
                s["refused_label"].set_text("refused: " + codes)
            landed = any(e["event"] == "mission done; landing" and e["t"] <= r["t"] for e in s["events"])
            if landed:
                s["aim"].set_data([], [])
                s["target"].set_data([], [])
                done = r is s["rows"][-1]
                s["status"].set_text(s["summary"] if done else "mission done, landing")
                s["status"].set_color(s["summary_colour"] if done else ink)
            else:
                tx, ty, _ = r["target"]
                s["aim"].set_data([r["x"], tx], [r["y"], ty])
                s["target"].set_data([tx], [ty])
                out_now = r["out"] < 0
                if r["action"] in ("hold", "drop"):
                    s["status"].set_text(f"waypoint {r['wp'] + 1}: REFUSED, holding")
                    s["status"].set_color(bad)
                elif out_now:
                    d = -r["out"]
                    s["status"].set_text(f"waypoint {r['wp'] + 1}: OUTSIDE THE FENCE by {d:.0f} m")
                    s["status"].set_color(bad)
                else:
                    s["status"].set_text(f"waypoint {r['wp'] + 1}")
                    s["status"].set_color(ink)
            s["clock"].set_text(f"t = {r['t'] - s['arm']:5.1f} s   alt {r['z']:4.1f} m")
            artists += [s["refused"], s["refused_x"], s["refused_label"], s["trail_in"], s["trail_out"], s["vehicle"], s["aim"], s["target"],
                        s["status"], s["clock"]]
        return artists

    anim = FuncAnimation(fig, frame, frames=len(times), blit=False)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("--envelope", default=os.path.join(HERE, "field.fence"))
    parser.add_argument("--check", action="store_true",
                        help="fail unless the guarded run stayed inside the fence")
    parser.add_argument("--gif", help="write a side-by-side animation here")
    args = parser.parse_args()

    env = load_envelope(args.envelope)
    results = {}
    runs = []
    for name, title in (("unguarded", "Without the guard"), ("guarded", "With geofence-guard")):
        if not os.path.exists(os.path.join(args.out_dir, f"{name}.csv")):
            continue
        rows, events = load_run(args.out_dir, name)
        results[name] = measure(rows, env)
        results[name]["refused_waypoints"] = [e["event"] for e in events if "refused" in e["event"]]
        runs.append((title, rows, events))

    print(json.dumps(results, indent=2))
    with open(os.path.join(args.out_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)

    if args.gif:
        if len(runs) != 2:
            sys.exit("--gif needs both runs")
        animate(runs, env, args.gif, results)
        print(f"wrote {args.gif}")

    if args.check:
        g = results.get("guarded")
        if g is None:
            sys.exit("no guarded run to check")
        if g["seconds_outside_fence"] > 0 or g["min_fence_clearance_m"] < 0:
            sys.exit(f"FAIL: guarded run left the fence: {g}")
        if g["min_exclusion_clearance_m"] < 0:
            sys.exit(f"FAIL: guarded run entered an exclusion zone: {g}")
        if not g["refused_waypoints"]:
            sys.exit("FAIL: the guard refused nothing, so this run did not exercise it")
        # The control: the same mission without the guard has to breach, or the
        # scenario has drifted and the guarded result proves nothing.
        u = results.get("unguarded")
        if u is not None and u["seconds_outside_fence"] == 0:
            sys.exit(f"FAIL: the unguarded control never left the fence: {u}")
        print("check: guarded run stayed inside the fence and out of every exclusion zone")


if __name__ == "__main__":
    main()
