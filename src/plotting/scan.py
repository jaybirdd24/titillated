#!/opt/homebrew/opt/python@3.14/Frameworks/Python.framework/Versions/3.14/bin/python3
"""
Robot Spin Analysis - Distance vs Heading
Finds wall-facing headings from ultrasonic minima and compares to logged target headings.

CSV format:
  ms, state, heading, us_cm, target_hdg, target_dist_cm, ir_front_mm, ir_right_mm, ir_rear_mm, ir_left_mm
"""

import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from pathlib import Path

# ── colour palette ────────────────────────────────────────────────────────────
BG      = "#0a0e17"
PANEL   = "#0f141e"
GRID    = "#1a2030"
C_US    = "#63b3ed"
C_HDG   = "#f6ad55"
C_TGT   = "#fc8181"
C_STATE = "#9f7aea"
C_MIN   = "#48bb78"
C_TEXT  = "#ffffff"
C_DIM   = "#b0c4de"

STATE_COLORS = {
    "HOMING_SCAN":   "#9f7aea",
    "HOMING_ALIGN":  "#63b3ed",
    "HOMING_DRIVE":  "#48bb78",
    "IDLE":          "#4a5a6a",
    "SPIN":          "#f6ad55",
}

# ── helpers ───────────────────────────────────────────────────────────────────

def style_ax(ax, xlabel="", ylabel="", title=""):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=C_DIM, labelsize=8)
    ax.xaxis.label.set_color(C_DIM)
    ax.yaxis.label.set_color(C_DIM)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID)
    ax.grid(True, color=GRID, linewidth=0.5, linestyle="--")
    if xlabel: ax.set_xlabel(xlabel, fontsize=9)
    if ylabel: ax.set_ylabel(ylabel, fontsize=9)
    if title:  ax.set_title(title, color=C_TEXT, fontsize=10, pad=8, fontweight="bold")


def find_local_minima(headings, distances, min_separation_deg=50, smooth_window=5):
    """
    Bin ultrasonic data by heading degree, average per bin,
    then find local minima that are at least min_separation_deg apart.
    Returns list of (heading, distance) tuples.
    """
    # bin into 2-degree buckets and average
    bins = {}
    for h, d in zip(headings, distances):
        b = round(h / 2) * 2
        bins.setdefault(b, []).append(d)
    
    angles  = sorted(bins.keys())
    avg_d   = [np.mean(bins[a]) for a in angles]

    # simple moving average smoothing
    kernel  = np.ones(smooth_window) / smooth_window
    smoothed = np.convolve(avg_d, kernel, mode="same")

    # find minima
    minima = []
    for i in range(1, len(smoothed) - 1):
        if smoothed[i] < smoothed[i-1] and smoothed[i] < smoothed[i+1]:
            # check it's meaningfully separated from any already-found minimum
            if all(abs(angles[i] - m[0]) > min_separation_deg for m in minima):
                minima.append((angles[i], smoothed[i]))

    # sort by distance (closest first)
    minima.sort(key=lambda x: x[1])
    return minima, angles, smoothed


def load_csv(path: str) -> pd.DataFrame:
    cols = ["ms","state","heading","us_cm","target_hdg","target_dist_cm",
            "ir_front_mm","ir_right_mm","ir_rear_mm","ir_left_mm"]
    
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # skip header-like rows
            if line.lower().startswith("ms") or line.lower().startswith("time"):
                continue
            parts = line.split(",")
            if len(parts) < len(cols):
                continue
            try:
                row = {
                    "ms":            float(parts[0]),
                    "state":         parts[1].strip(),
                    "heading":       float(parts[2]),
                    "us_cm":         float(parts[3]),
                    "target_hdg":    float(parts[4]),
                    "target_dist_cm":float(parts[5]),
                }
                rows.append(row)
            except ValueError:
                continue

    if not rows:
        sys.exit("No valid data rows found. Check your CSV format.")

    df = pd.DataFrame(rows)
    df["time_s"] = (df["ms"] - df["ms"].iloc[0]) / 1000.0

    # wrap heading to 0–360
    df["heading"] = df["heading"] % 360

    # clamp ultrasonic outliers (sensor returns 0 or >300 when it loses signal)
    df = df[(df["us_cm"] > 1) & (df["us_cm"] < 290)].copy()

    return df


def print_summary(df, minima):
    states = df["state"].value_counts()
    print("\n── Data Summary ─────────────────────────────")
    print(f"  Rows          : {len(df)}")
    print(f"  Duration      : {df['time_s'].iloc[-1]:.1f} s")
    print(f"  Heading range : {df['heading'].min():.1f}° – {df['heading'].max():.1f}°")
    print(f"  US range      : {df['us_cm'].min():.1f} – {df['us_cm'].max():.1f} cm")
    print(f"\n── States ───────────────────────────────────")
    for s, n in states.items():
        print(f"  {s:<25} {n:>5} samples")

    print(f"\n── Ultrasonic Minima (wall candidates) ──────")
    for i, (h, d) in enumerate(minima[:4]):
        print(f"  Wall {i+1}: heading {h:.1f}°,  min distance {d:.1f} cm")

    if "HOMING_SCAN" in df["state"].values or "SPIN" in df["state"].values:
        spin_df = df[df["state"].isin(["HOMING_SCAN","SPIN"])]
        tgt = spin_df["target_hdg"].dropna()
        print(f"\n── Target Heading during scan ───────────────")
        print(f"  Range: {tgt.min():.1f}° – {tgt.max():.1f}°")

    print()


# ── main plot ─────────────────────────────────────────────────────────────────

def plot(df, minima, angles, smoothed, output_path=None):
    states     = df["state"].unique()
    state_list = list(df["state"])

    fig = plt.figure(figsize=(16, 11), facecolor=BG)
    fig.suptitle("Robot Spin Analysis  ·  Ultrasonic Distance vs Heading",
                 color=C_TEXT, fontsize=13, fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(3, 2, figure=fig,
                           left=0.07, right=0.97,
                           top=0.94, bottom=0.07,
                           hspace=0.42, wspace=0.30)

    # ── 1. Distance vs Wrapped Heading (main chart) ────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    style_ax(ax1, xlabel="Heading (°)", ylabel="Distance (cm)",
             title="Ultrasonic Distance vs Heading  —  colour = robot state")

    # scatter coloured by state
    for state in states:
        mask = df["state"] == state
        c = STATE_COLORS.get(state, "#888888")
        ax1.scatter(df.loc[mask, "heading"], df.loc[mask, "us_cm"],
                    s=4, color=c, alpha=0.6, linewidths=0, label=state, zorder=3)

    # smoothed average curve
    ax1.plot(angles, smoothed, color="white", linewidth=1.2,
             alpha=0.4, zorder=4, label="smoothed avg")

    # mark minima
    for i, (h, d) in enumerate(minima[:4]):
        ax1.axvline(h, color=C_MIN, linewidth=1.5, linestyle="--", alpha=0.8, zorder=5)
        ax1.text(h + 1.5, ax1.get_ylim()[1] * 0.92 if ax1.get_ylim()[1] > 0 else 200,
                 f"Wall {i+1}\n{h:.0f}°\n{d:.1f}cm",
                 color=C_MIN, fontsize=7.5, va="top", zorder=6)

    # target heading lines (deduplicated)
    tgt_headings = df.loc[df["target_hdg"] > 0, "target_hdg"].unique()
    for th in tgt_headings:
        ax1.axvline(th % 360, color=C_TGT, linewidth=1.2,
                    linestyle=":", alpha=0.7, zorder=5)

    ax1.set_xlim(0, 360)
    ax1.set_xticks(range(0, 361, 30))

    # legend
    handles = [Line2D([0],[0], marker='o', color='w', markerfacecolor=STATE_COLORS.get(s,"#888"),
                      markersize=6, label=s, linestyle='None') for s in states]
    handles += [Line2D([0],[0], color="white", alpha=0.4, linewidth=1.5, label="smoothed avg"),
                Line2D([0],[0], color=C_MIN, linewidth=1.5, linestyle="--", label="wall minima"),
                Line2D([0],[0], color=C_TGT, linewidth=1.2, linestyle=":", label="target heading")]
    ax1.legend(handles=handles, loc="upper right", framealpha=0.2,
               facecolor=PANEL, edgecolor=GRID, fontsize=7.5, ncol=3)

    # ── 2. Distance vs Time ────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    style_ax(ax2, xlabel="Time (s)", ylabel="Distance (cm)", title="Distance vs Time")

    for state in states:
        mask = df["state"] == state
        c = STATE_COLORS.get(state, "#888888")
        ax2.scatter(df.loc[mask, "time_s"], df.loc[mask, "us_cm"],
                    s=3, color=c, alpha=0.7, linewidths=0)

    # ── 3. Heading vs Time ─────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    style_ax(ax3, xlabel="Time (s)", ylabel="Heading (°)", title="Heading vs Time")

    ax3.plot(df["time_s"], df["heading"], color=C_HDG, linewidth=1, alpha=0.9)

    # target heading overlay
    tgt_mask = df["target_hdg"] > 0
    if tgt_mask.any():
        ax3.scatter(df.loc[tgt_mask, "time_s"], df.loc[tgt_mask, "target_hdg"] % 360,
                    s=3, color=C_TGT, alpha=0.7, linewidths=0, label="target hdg")
        ax3.legend(fontsize=7.5, framealpha=0.2, facecolor=PANEL, edgecolor=GRID)

    ax3.set_ylim(0, 360)
    ax3.set_yticks(range(0, 361, 45))

    # ── 4. Polar plot ──────────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0], projection="polar")
    ax4.set_facecolor(PANEL)
    ax4.tick_params(colors=C_DIM, labelsize=7)
    ax4.set_title("Polar: Distance by Heading", color=C_TEXT, fontsize=10,
                  fontweight="bold", pad=14)

    # polar expects radians, 0=north, clockwise → convert
    theta = np.deg2rad(df["heading"])  # matplotlib polar: 0=east, CCW
    # remap: heading 0°=north, CW → polar θ = 90° - heading
    theta_plot = np.deg2rad(90 - df["heading"].values) % (2 * np.pi)

    for state in states:
        mask = df["state"] == state
        c = STATE_COLORS.get(state, "#888888")
        ax4.scatter(theta_plot[mask.values], df.loc[mask, "us_cm"],
                    s=2, color=c, alpha=0.5, linewidths=0)

    # mark minima
    for i, (h, d) in enumerate(minima[:4]):
        t = np.deg2rad(90 - h) % (2 * np.pi)
        ax4.plot([t, t], [0, d], color=C_MIN, linewidth=2, alpha=0.9)
        ax4.text(t, d + 8, f"W{i+1}\n{h:.0f}°", color=C_MIN, fontsize=7,
                 ha="center", va="bottom")

    ax4.set_theta_zero_location("N")
    ax4.set_theta_direction(-1)
    ax4.set_rlim(0, df["us_cm"].max() + 20)
    ax4.grid(True, color=GRID, linewidth=0.5)
    for label in ax4.get_xticklabels() + ax4.get_yticklabels():
        label.set_color(C_DIM)
        label.set_fontsize(7)

    # ── 5. Per-state distance distribution ────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    style_ax(ax5, xlabel="Distance (cm)", ylabel="Count",
             title="Distance Distribution by State")

    for state in states:
        mask = df["state"] == state
        c = STATE_COLORS.get(state, "#888888")
        vals = df.loc[mask, "us_cm"]
        ax5.hist(vals, bins=40, color=c, alpha=0.55, label=state, edgecolor="none")

    ax5.legend(fontsize=7.5, framealpha=0.2, facecolor=PANEL, edgecolor=GRID)

    # ── finish ─────────────────────────────────────────────────────────────
    for ax in [ax1, ax2, ax3, ax5]:
        ax.set_facecolor(PANEL)

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight", facecolor=BG)
        print(f"  Saved → {output_path}")
    else:
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Analyse robot spin CSV: ultrasonic distance vs heading.")
    parser.add_argument("csv", help="Path to the CSV log file")
    parser.add_argument("-o", "--output", default=None,
                        help="Save plot to file (e.g. out.png). Omit to show interactively.")
    parser.add_argument("--min-sep", type=float, default=50,
                        help="Minimum angular separation (°) between wall minima (default 50)")
    parser.add_argument("--smooth", type=int, default=5,
                        help="Smoothing window in bins (default 5)")
    parser.add_argument("--state-filter", default=None,
                        help="Comma-separated list of states to include, e.g. HOMING_SCAN,SPIN")
    args = parser.parse_args()

    df = load_csv(args.csv)

    if args.state_filter:
        keep = [s.strip() for s in args.state_filter.split(",")]
        df = df[df["state"].isin(keep)].copy()
        if df.empty:
            sys.exit(f"No rows left after filtering for states: {keep}")

    minima, angles, smoothed = find_local_minima(
        df["heading"].values, df["us_cm"].values,
        min_separation_deg=args.min_sep,
        smooth_window=args.smooth)

    print_summary(df, minima)
    plot(df, minima, angles, smoothed, output_path=args.output)


if __name__ == "__main__":
    main()