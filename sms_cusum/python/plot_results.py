"""
Publication-quality visualization for SMS-CUSUM validation results.

Generates overlay plots showing:
  - tau_ext_norm signal with ground-truth state shading
  - CUSUM statistic S_n overlaid (secondary y-axis)
  - Detected transitions as vertical markers
  - Detection latency annotation
"""

from __future__ import annotations

import csv
import os
import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python.sms_cusum import SMSCusum, GraspState
from python.config import SMSCusumConfig

# State colors matching existing analysis scripts
STATE_COLORS = {
    "UNKNOWN": "#C0C0C0",
    "BASELINE": "#4477AA",
    "CLOSING_COMMAND": "#228B22",
    "CLOSING": "#228B22",
    "CONTACT_CONFIRMED": "#00CED1",
    "CONTACT": "#FFA500",
    "GRASPING": "#9370DB",
    "UPLIFT": "#DC143C",
    "EVALUATE": "#8B4513",
    "DOWNLIFT": "#808080",
    "SETTLING": "#808080",
    "SUCCESS": "#32CD32",
    "FAILED": "#FF0000",
    "SLIP": "#FF69B4",
}


def plot_trial_overlay(
    rows: list[dict],
    result,
    trial_name: str,
    output_dir: str,
    config: SMSCusumConfig,
) -> None:
    """Generate a publication-quality overlay plot for one trial.

    Parameters
    ----------
    rows : list[dict]
        Loaded CSV rows.
    result : TrialResult
        Validation result for this trial.
    trial_name : str
        Trial identifier for the title.
    output_dir : str
        Directory to save the plot.
    config : SMSCusumConfig
        Detector configuration (for annotation).
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot generation.")
        return

    n = len(rows)

    # Extract signals
    time_s = []
    tau_ext_norm = []
    gt_labels = []
    for row in rows:
        time_s.append(float(row["time_float"]))
        tau_ext_norm.append(float(row["tau_ext_norm"]))
        gt_labels.append(row["state_label"])

    # Convert to relative time
    t0 = time_s[0]
    time_rel = [t - t0 for t in time_s]

    # Re-run detector to get per-sample CUSUM statistic
    detector = SMSCusum(config)
    cusum_stats = []
    entered_closing = False

    for i, row in enumerate(rows):
        tau = float(row["tau_ext_norm"])
        gt = row["state_label"]

        if not entered_closing and gt in ("CLOSING_COMMAND", "CLOSING"):
            if detector.baseline_ready:
                detector.enter_closing()
                entered_closing = True

        detector.update(tau)
        cusum_stats.append(detector.contact_cusum.statistic)

    # Create figure
    fig, ax1 = plt.subplots(figsize=(14, 5))

    # Ground truth state background shading
    prev_label = gt_labels[0]
    start_idx = 0
    for i in range(1, n):
        if gt_labels[i] != prev_label or i == n - 1:
            end_idx = i if i < n - 1 else n
            color = STATE_COLORS.get(prev_label, "#FFFFFF")
            ax1.axvspan(time_rel[start_idx], time_rel[min(end_idx, n - 1)],
                       alpha=0.15, color=color, label=None)
            start_idx = i
            prev_label = gt_labels[i]

    # Plot tau_ext_norm
    ax1.plot(time_rel, tau_ext_norm, color="black", linewidth=0.5, alpha=0.8,
             label="tau_ext_norm")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Tau Ext Norm (Nm)", color="black")
    ax1.tick_params(axis="y", labelcolor="black")

    # Baseline reference line
    if detector.baseline.ready:
        ax1.axhline(y=result.baseline_mean, color="blue", linestyle="--",
                    linewidth=0.8, alpha=0.6, label=f"baseline={result.baseline_mean:.3f}")

    # CUSUM statistic on secondary y-axis
    ax2 = ax1.twinx()
    ax2.plot(time_rel, cusum_stats, color="red", linewidth=0.8, alpha=0.7,
             label="CUSUM S_n")
    ax2.axhline(y=config.contact_stage.h, color="red", linestyle=":",
                linewidth=0.8, alpha=0.5, label=f"h={config.contact_stage.h}")
    ax2.set_ylabel("CUSUM Statistic S_n", color="red")
    ax2.tick_params(axis="y", labelcolor="red")

    # Detection markers
    if result.contact_det_index is not None:
        t_det = time_rel[result.contact_det_index]
        ax1.axvline(x=t_det, color="orange", linewidth=2, linestyle="-",
                   alpha=0.8, label=f"CONTACT detected (idx={result.contact_det_index})")

    if result.contact_gt_index is not None:
        t_gt = time_rel[result.contact_gt_index]
        ax1.axvline(x=t_gt, color="cyan", linewidth=2, linestyle="--",
                   alpha=0.8, label=f"CONTACT ground truth (idx={result.contact_gt_index})")

    # Latency annotation
    if result.contact_latency_ms is not None:
        t_mid = (time_rel[result.contact_gt_index] + time_rel[result.contact_det_index]) / 2
        y_mid = max(tau_ext_norm) * 0.9
        sign = "+" if result.contact_latency_samples >= 0 else ""
        ax1.annotate(
            f"Latency: {sign}{result.contact_latency_samples} samples "
            f"({sign}{result.contact_latency_ms:.0f}ms)",
            xy=(t_mid, y_mid),
            fontsize=9, ha="center",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
        )

    # Title and legend
    status = "DETECTED" if result.contact_detected else ("NO CONTACT" if not result.has_contact_gt else "MISSED")
    fp_str = " [FALSE POSITIVE]" if result.false_positive else ""
    ax1.set_title(
        f"SMS-CUSUM Validation: {trial_name} -- {status}{fp_str}\n"
        f"k_eff={result.k_effective:.4f}, sigma={result.baseline_sigma:.4f}, "
        f"h={config.contact_stage.h}",
        fontsize=11,
    )

    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=8)

    plt.tight_layout()
    safe_name = trial_name.replace("/", "_")
    out_path = os.path.join(output_dir, f"sms_cusum_validation_{safe_name}.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"    Plot saved: {out_path}")


def plot_all_trials(data_dir: str, config: Optional[SMSCusumConfig] = None) -> None:
    """Generate validation plots for all trials in a data directory."""
    from python.validate_offline import discover_trials, load_trial, run_trial

    config = config or SMSCusumConfig()
    trials = discover_trials(data_dir)

    for name, csv_path in trials:
        print(f"Plotting {name}...")
        rows = load_trial(csv_path)
        result = run_trial(rows, name, config)
        output_dir = str(Path(csv_path).parent)
        plot_trial_overlay(rows, result, name, output_dir, config)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="SMS-CUSUM result plotting")
    parser.add_argument(
        "--data-dir",
        default=str(Path(__file__).resolve().parent.parent.parent / "kitting_bags2"),
    )
    args = parser.parse_args()
    plot_all_trials(args.data_dir)
