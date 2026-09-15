#!/usr/bin/env python3
"""Plot mvl motor position / velocity / acceleration / jerk curves (no dependencies).

Reads the CSV produced by the demo server's `mvl` command (mvl_motor_log.csv)
and renders four stacked subplots (position / velocity / acceleration / jerk)
into an SVG file using only the Python standard library.

Usage:
    python3 plot_mvl.py [path/to/mvl_motor_log.csv]

The output is written next to the CSV as `<name>.svg`.
"""
import csv
import sys

PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
           "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f"]


def read_csv(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [list(map(float, r)) for r in reader if r]
    return header, rows


def central_diff(y, dt):
    n = len(y)
    out = [0.0] * n
    if n <= 1:
        return out
    out[0] = (y[1] - y[0]) / dt
    out[-1] = (y[-1] - y[-2]) / dt
    for i in range(1, n - 1):
        out[i] = (y[i + 1] - y[i - 1]) / (2.0 * dt)
    return out


def fmt(v):
    if abs(v) >= 1e4 or (v != 0 and abs(v) < 1e-4):
        return f"{v:.3e}"
    return f"{v:.4g}"


def build_svg(t, series, path):
    W, H = 1400, 1650
    LEFT, RIGHT = 100, 40
    TOP = 90
    BOTTOM = 55
    SUB_GAP = 30
    SUB_H = (H - TOP - BOTTOM - 3 * SUB_GAP) / 4.0

    parts = []
    parts.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}">'
    )
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    parts.append(
        f'<text x="{W/2}" y="40" text-anchor="middle" font-size="26" '
        f'font-family="Helvetica,Arial,sans-serif" font-weight="bold" fill="#111">'
        f'{path}</text>'
    )
    dt = t[1] - t[0] if len(t) > 1 else 0.0
    parts.append(
        f'<text x="{W/2}" y="68" text-anchor="middle" font-size="15" '
        f'font-family="Helvetica,Arial,sans-serif" fill="#555">'
        f'{len(t)} samples @ dt={dt:.4g}s</text>'
    )

    titles = ["position", "velocity", "acceleration", "jerk"]
    ylabels = ["pos (rad)", "vel (rad/s)", "acc (rad/s^2)", "jerk (rad/s^3)"]
    n_joint = len(series[0])

    x0, x1 = LEFT, W - RIGHT
    t_lo, t_hi = t[0], t[-1]
    if t_hi == t_lo:
        t_hi = t_lo + 1.0

    for si in range(4):
        y0 = TOP + si * (SUB_H + SUB_GAP)
        y1 = y0 + SUB_H

        data = series[si]
        all_vals = [v for joint in data for v in joint]
        lo, hi = min(all_vals), max(all_vals)
        if hi - lo < 1e-12:
            pad = 1.0 if hi == 0 else abs(hi) * 0.1
            lo, hi = lo - pad, hi + pad
        else:
            pad = (hi - lo) * 0.06
            lo, hi = lo - pad, hi + pad

        def sx(v):
            return x0 + (v - t_lo) * (x1 - x0) / (t_hi - t_lo)

        def sy(v):
            return y1 - (v - lo) * (y1 - y0) / (hi - lo)

        parts.append(
            f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{x1-x0:.1f}" '
            f'height="{y1-y0:.1f}" fill="#fdfdfd" stroke="#bbb" stroke-width="1"/>'
        )
        for g in range(5):
            gy = y0 + (y1 - y0) * g / 4.0
            gx = x0 + (x1 - x0) * g / 4.0
            parts.append(
                f'<line x1="{x0:.1f}" y1="{gy:.1f}" x2="{x1:.1f}" y2="{gy:.1f}" '
                f'stroke="#e5e5e5" stroke-width="1"/>'
            )
            parts.append(
                f'<line x1="{gx:.1f}" y1="{y0:.1f}" x2="{gx:.1f}" y2="{y1:.1f}" '
                f'stroke="#e5e5e5" stroke-width="1"/>'
            )

        for g in range(5):
            gy = y0 + (y1 - y0) * g / 4.0
            val = hi - (hi - lo) * g / 4.0
            parts.append(
                f'<text x="{x0-8:.1f}" y="{gy+4:.1f}" text-anchor="end" font-size="12" '
                f'font-family="Helvetica,Arial,sans-serif" fill="#333">{fmt(val)}</text>'
            )

        if si == 3:
            for g in range(5):
                gx = x0 + (x1 - x0) * g / 4.0
                val = t_lo + (t_hi - t_lo) * g / 4.0
                parts.append(
                    f'<text x="{gx:.1f}" y="{y1+20:.1f}" text-anchor="middle" font-size="12" '
                    f'font-family="Helvetica,Arial,sans-serif" fill="#333">{fmt(val)}</text>'
                )

        parts.append(
            f'<text x="{LEFT}" y="{y0+18}" text-anchor="start" font-size="15" '
            f'font-family="Helvetica,Arial,sans-serif" font-weight="bold" fill="#111">'
            f'{titles[si]}</text>'
        )
        parts.append(
            f'<text x="{x1+4}" y="{y0-8}" text-anchor="end" font-size="13" '
            f'font-family="Helvetica,Arial,sans-serif" fill="#666">{ylabels[si]}</text>'
        )

        for j in range(n_joint):
            color = PALETTE[j % len(PALETTE)]
            pts = []
            step = max(1, len(t) // 4000)
            for i in range(0, len(t), step):
                pts.append(f"{sx(t[i]):.2f},{sy(data[j][i]):.2f}")
            parts.append(
                f'<polyline points="{" ".join(pts)}" fill="none" '
                f'stroke="{color}" stroke-width="1.4"/>'
            )

    lx = LEFT
    ly = H - 18
    parts.append(
        f'<text x="{lx}" y="{ly}" font-size="13" font-family="Helvetica,Arial,sans-serif" '
        f'fill="#333">joints:</text>'
    )
    lx += 50
    for j in range(n_joint):
        color = PALETTE[j % len(PALETTE)]
        parts.append(
            f'<line x1="{lx}" y1="{ly-4}" x2="{lx+18}" y2="{ly-4}" '
            f'stroke="{color}" stroke-width="3"/>'
        )
        parts.append(
            f'<text x="{lx+22}" y="{ly}" font-size="13" '
            f'font-family="Helvetica,Arial,sans-serif" fill="#333">j{j}</text>'
        )
        lx += 46

    parts.append("</svg>")
    return "\n".join(parts)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "mvl_motor_log.csv"
    header, rows = read_csv(path)
    if not rows:
        print(f"no data in {path}")
        return

    t = [r[1] for r in rows]
    n_joint = len(header) - 2
    pos = [[r[2 + j] for j in range(n_joint)] for r in rows]
    pos_j = [[pos[i][j] for i in range(len(rows))] for j in range(n_joint)]

    dt = (t[-1] - t[0]) / (len(t) - 1) if len(t) > 1 else 1e-3
    vel_j = [central_diff(y, dt) for y in pos_j]
    acc_j = [central_diff(y, dt) for y in vel_j]
    jerk_j = [central_diff(y, dt) for y in acc_j]

    series = [pos_j, vel_j, acc_j, jerk_j]
    svg = build_svg(t, series, path)
    out = path.rsplit(".", 1)[0] + ".svg" if "." in path else path + ".svg"
    with open(out, "w") as f:
        f.write(svg)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
