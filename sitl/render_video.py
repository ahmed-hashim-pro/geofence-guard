#!/usr/bin/env python3
"""Turn the two Gazebo recordings into one side-by-side clip.

    render_video.py <out_dir> <video.mp4> [--gif docs/sitl-gazebo.gif]

Needs the RECORD=1 output of run.sh: <run>.mp4, <run>-video-start, and the
planner's telemetry. Each recording is trimmed from arming to touchdown,
captioned from the telemetry (the captions are computed, not typed), sped up
and placed side by side. Runs where ffmpeg and libass are available; the
geofence-guard-sitl image has both.
"""

import argparse
import csv
import json
import os
import subprocess
import tempfile

from analyze import clearance, load_envelope, measure

HERE = os.path.dirname(os.path.abspath(__file__))
SPEED = 4.0
WIDTH = 640           # per panel in the mp4
HOLD_END_S = 3.0      # freeze on the last frame
PLAY_W, PLAY_H = 1310, 1050  # the recorded crop

RUNS = (("unguarded", "Without the guard"), ("guarded", "With geofence-guard"))


def ass_time(t):
    t = max(t, 0.0)
    h, rem = divmod(t, 3600)
    m, s = divmod(rem, 60)
    return f"{int(h)}:{int(m):02d}:{s:05.2f}"


def status_of(row, env, landing_wall):
    wp = int(row["wp"]) + 1
    if landing_wall is not None and float(row["wall"]) >= landing_wall:
        return "Mission done, landing", "white"
    if row["action"] in ("hold", "drop"):
        return f"Waypoint {wp}: REFUSED, holding", "red"
    c = clearance((float(row["x"]), float(row["y"])), env["fence"])
    if c < 0:
        return f"Waypoint {wp}: OUTSIDE THE FENCE by {-c:.0f} m", "red"
    return f"Waypoint {wp}", "white"


def captions(name, title, out_dir, env, t0, t1, summary):
    """ASS subtitles for one run, on the trimmed timeline [t0, t1] (wall)."""
    with open(os.path.join(out_dir, f"{name}.csv")) as f:
        rows = [r for r in csv.DictReader(f) if t0 <= float(r["wall"]) <= t1]
    with open(os.path.join(out_dir, f"{name}-events.json")) as f:
        events = json.load(f)
    landing = next((e["wall"] for e in events if e["event"] == "mission done; landing"), None)

    colour = {"white": "&H00FFFFFF", "red": "&H004040FF", "green": "&H0060D060"}
    lines = []

    def line(start, end, text, style, col="white"):
        lines.append(f"Dialogue: 0,{ass_time(start)},{ass_time(end)},{style},,0,0,0,,"
                     f"{{\\c{colour[col]}}}{text}")

    end = t1 - t0 + HOLD_END_S * SPEED
    line(0, end, title, "Title")

    seg_text, seg_col, seg_start = None, None, 0.0
    first_refusal = None
    for r in rows:
        t = float(r["wall"]) - t0
        text, col = status_of(r, env, landing)
        if r["action"] == "hold" and first_refusal is None:
            first_refusal = (t, r["codes"])
        if text != seg_text:
            if seg_text is not None:
                line(seg_start, t, seg_text, "Status", seg_col)
            seg_text, seg_col, seg_start = text, col, t
    last_t = float(rows[-1]["wall"]) - t0
    if seg_text is not None:
        line(seg_start, last_t, seg_text, "Status", seg_col)

    if first_refusal is not None:
        line(first_refusal[0], end, f"Shortcut refused: {first_refusal[1]}", "Note", "red")
    ok = summary.startswith("Never")
    line(last_t, end, summary, "Status", "green" if ok else "red")

    header = f"""[Script Info]
ScriptType: v4.00+
PlayResX: {PLAY_W}
PlayResY: {PLAY_H}

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Title,DejaVu Sans,58,&H00FFFFFF,&H00FFFFFF,&H80202020,&H80202020,1,0,0,0,100,100,0,0,3,14,0,7,30,30,28,1
Style: Note,DejaVu Sans,44,&H00FFFFFF,&H00FFFFFF,&H80202020,&H80202020,1,0,0,0,100,100,0,0,3,12,0,7,30,30,120,1
Style: Status,DejaVu Sans,50,&H00FFFFFF,&H00FFFFFF,&H80202020,&H80202020,1,0,0,0,100,100,0,0,3,14,0,1,30,30,30,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    return header + "\n".join(lines) + "\n"


def summary_line(m):
    if m["seconds_outside_fence"] > 0:
        return (f"Left the fence for {m['seconds_outside_fence']:.0f} s, "
                f"up to {m['furthest_outside_m']:.0f} m outside")
    return f"Never left the fence (closest {m['min_fence_clearance_m']:.1f} m)"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("video")
    parser.add_argument("--gif")
    parser.add_argument("--envelope", default=os.path.join(HERE, "field.fence"))
    args = parser.parse_args()
    env = load_envelope(args.envelope)
    tmp = tempfile.mkdtemp()

    inputs, filters, durations = [], [], []
    for i, (name, title) in enumerate(RUNS):
        with open(os.path.join(args.out_dir, f"{name}-video-start")) as f:
            start = float(f.read())
        with open(os.path.join(args.out_dir, f"{name}-events.json")) as f:
            events = json.load(f)
        armed = next(e["wall"] for e in events if e["event"] == "armed in OFFBOARD")
        done = next(e["wall"] for e in events if e["event"] in ("disarmed", "land timed out"))
        t0, t1 = armed - 1.0, done + 1.0

        from analyze import load_run  # same measurement as analyze.py
        rows, _ = load_run(args.out_dir, name)
        ass = os.path.join(tmp, f"{name}.ass")
        with open(ass, "w") as f:
            f.write(captions(name, title, args.out_dir, env, t0, t1, summary_line(measure(rows, env))))

        inputs += ["-i", os.path.join(args.out_dir, f"{name}.mp4")]
        durations.append((t1 - t0) / SPEED)
        filters.append(
            f"[{i}:v]trim=start={t0 - start:.3f}:end={t1 - start:.3f},setpts=PTS-STARTPTS,"
            f"subtitles={ass},setpts=PTS/{SPEED},scale={WIDTH}:-2[s{i}]"
        )

    total = max(durations) + HOLD_END_S
    for i, d in enumerate(durations):
        filters.append(f"[s{i}]tpad=stop_mode=clone:stop_duration={total - d:.3f}[p{i}]")
    filters.append("[p0]pad=iw+6:ih:0:0:white[l]")
    filters.append("[l][p1]hstack=inputs=2,fps=20[v]")

    subprocess.run(["ffmpeg", "-v", "error", "-y", *inputs, "-filter_complex", ";".join(filters),
                    "-map", "[v]", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24",
                    "-movflags", "+faststart", args.video], check=True)
    print(f"wrote {args.video} ({total:.1f} s)")

    if args.gif:
        palette = os.path.join(tmp, "palette.png")
        scale = "fps=10,scale=960:-2:flags=lanczos"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", args.video, "-vf",
                        f"{scale},palettegen=max_colors=96", palette], check=True)
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", args.video, "-i", palette,
                        "-lavfi", f"{scale}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4",
                        args.gif], check=True)
        print(f"wrote {args.gif}")


if __name__ == "__main__":
    main()
