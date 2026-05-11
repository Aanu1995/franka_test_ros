#!/usr/bin/env python3
"""Phase 2 – State-Based Signal Analysis & Plotting for kitting bag trials."""

import glob
import os
import shutil
import sys

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

STATE_COLORS = {
    "BASELINE": "blue",
    "CONTACT": "orange",
    "CONTACT_CONFIRMED": "cyan",
    "CLOSING": "green",
    "UPLIFT": "red",
    "EVALUATE": "brown",
}

# Purple gradient for GRASP_1 … GRASP_N (light → dark)
_GRASP_CMAP = matplotlib.colormaps["Purples"]

# Canonical legend order — GRASP_N slots in before UPLIFT
_LEGEND_ORDER = [
    "BASELINE",
    "CLOSING",
    "CONTACT_CONFIRMED",
    "CONTACT",
    "_GRASP_",
    "UPLIFT",
    "EVALUATE",
]

REQUIRED_COLUMNS = {
    "time_float",
    "state_label",
    "wrench_fx",
    "wrench_fy",
    "wrench_fz",
    "wrench_norm",
    "tau_ext_norm",
    "ee_vz",
    *{f"tau_J_{i}" for i in range(1, 8)},
    *{f"tau_ext_{i}" for i in range(1, 8)},
}

NUMERIC_REQUIRED_COLUMNS = sorted(REQUIRED_COLUMNS - {"state_label"})

REPORT_COLUMNS = [
    "parent_folder",
    "trial_folder",
    "csv_path",
    "status",
    "output_or_quarantine_path",
    "error_message",
]


def _grasp_max_from_states(states):
    """Return the highest k found among GRASP_k labels, or 1 if none."""
    nums = [int(s.split("_", 1)[1]) for s in states if s.startswith("GRASP_")]
    return max(nums) if nums else 1


def _get_state_color(state, grasp_max=1):
    """Return the colour for *state*.  GRASP_<k> states get a purple gradient."""
    if state.startswith("GRASP_"):
        k = int(state.split("_", 1)[1])
        frac = 0.3 + 0.6 * ((k - 1) / max(grasp_max - 1, 1))
        return _GRASP_CMAP(frac)
    return STATE_COLORS.get(state, "black")


def _add_state_indicator(ax, active_states=None):
    """Add a state color legend in the bottom-right corner of the axes."""
    if active_states is None:
        active_states = set(STATE_COLORS.keys())
    gmax = _grasp_max_from_states(active_states)

    # Build ordered list following _LEGEND_ORDER, expanding GRASP_N in place
    grasp_states = sorted(
        [s for s in active_states if s.startswith("GRASP_")],
        key=lambda s: int(s.split("_", 1)[1]),
    )
    ordered = []
    for slot in _LEGEND_ORDER:
        if slot == "_GRASP_":
            ordered.extend(grasp_states)
        elif slot in active_states:
            ordered.append(slot)

    patches = [
        mpatches.Patch(color=_get_state_color(s, gmax), label=s) for s in ordered
    ]
    ax.legend(
        handles=patches,
        loc="upper left",
        fontsize=7,
        framealpha=0.85,
        title="States",
        title_fontsize=8,
    )


def load_and_preprocess(csv_path):
    """Load CSV and compute relative time."""
    df = pd.read_csv(csv_path)

    missing = sorted(REQUIRED_COLUMNS - set(df.columns))
    if missing:
        raise ValueError(f"missing required columns: {', '.join(missing)}")

    # Drop UNKNOWN state rows
    df = df[df["state_label"] != "UNKNOWN"].reset_index(drop=True)
    if df.empty:
        raise ValueError("no usable rows remain after filtering UNKNOWN states")
    if len(df) < 2:
        raise ValueError("fewer than two usable rows remain after filtering")

    try:
        df[NUMERIC_REQUIRED_COLUMNS] = df[NUMERIC_REQUIRED_COLUMNS].apply(
            pd.to_numeric, errors="raise"
        )
    except Exception as exc:
        raise ValueError(f"required numeric column contains non-numeric data: {exc}")

    numeric_values = df[NUMERIC_REQUIRED_COLUMNS].to_numpy(dtype=float)
    if not np.isfinite(numeric_values).all():
        raise ValueError("required numeric columns contain NaN or infinite values")

    df["time"] = df["time_float"] - df["time_float"].iloc[0]
    # Force decomposition: e_lift = [0,0,1] (vertical lift in base frame)
    df["Fn"] = df["wrench_fz"].abs()
    df["Ft"] = np.sqrt(df["wrench_fx"] ** 2 + df["wrench_fy"] ** 2)
    return df


def find_state_transitions(df):
    """Return indices where state_label changes."""
    transitions = df.index[df["state_label"] != df["state_label"].shift()].tolist()
    return transitions


def compute_state_statistics(df):
    """Compute per-state statistics and return summary dataframe."""
    groups = df.groupby("state_label")
    rows = []
    for state, grp in groups:
        rows.append(
            {
                "state_label": state,
                "mean_wrench_norm": grp["wrench_norm"].mean(),
                "std_wrench_norm": grp["wrench_norm"].std(),
                "mean_tau_ext_norm": grp["tau_ext_norm"].mean(),
                "std_tau_ext_norm": grp["tau_ext_norm"].std(),
                "mean_ee_vz": grp["ee_vz"].mean(),
                "std_ee_vz": grp["ee_vz"].std(),
                "count": len(grp),
            }
        )
    return pd.DataFrame(rows)


def plot_signal_vs_time(df, transitions, signal, title, ylabel, save_path, yticks=None):
    """Plot a signal vs time with state transition lines and background shading."""
    fig, ax = plt.subplots(figsize=(12, 5))

    # Background shading per state
    boundaries = transitions + [len(df)]
    active_states = set()
    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        active_states.add(state)

    gmax = _grasp_max_from_states(active_states)

    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        color = _get_state_color(state, gmax)
        t0 = seg["time"].iloc[0]
        t1 = seg["time"].iloc[-1]
        ax.axvspan(t0, t1, alpha=0.10, color=color)

    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        color = _get_state_color(state, gmax)
        ax.plot(seg["time"], seg[signal], linewidth=0.8, color=color)
    for idx in transitions[1:]:
        t = df["time"].iloc[idx]
        ax.axvline(x=t, color="red", linestyle="--", linewidth=0.7, alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xlim(0, df["time"].iloc[-1])
    if yticks is not None:
        ax.set_yticks(yticks)
        ymax = int(np.ceil(df[signal].max())) + 1
        ax.set_ylim(yticks[0], max(yticks[-1], ymax))
    ax.grid(True, alpha=0.3)
    _add_state_indicator(ax, active_states)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


def plot_state_colored(df, transitions, save_path):
    """Plot wrench_norm colored by state."""
    fig, ax = plt.subplots(figsize=(12, 5))

    # Plot each state segment with its color
    boundaries = transitions + [len(df)]
    active_states = set()
    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        active_states.add(state)

    gmax = _grasp_max_from_states(active_states)

    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        color = _get_state_color(state, gmax)
        ax.plot(seg["time"], seg["wrench_norm"], linewidth=0.8, color=color)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Wrench Norm (N)")
    ax.set_title("Wrench Norm by State")
    ax.set_xlim(0, df["time"].iloc[-1])
    ymax = int(np.ceil(df["wrench_norm"].max())) + 1
    ax.set_yticks(range(0, ymax + 1))
    ax.set_ylim(0, ymax)
    ax.grid(True, alpha=0.3)
    _add_state_indicator(ax, active_states)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


JOINT_COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
]


def plot_per_joint(df, transitions, prefix, num_joints, title, ylabel, save_path):
    """Plot per-joint signals (e.g. tau_J_1..7 or tau_ext_1..7) on subplots."""
    fig, axes = plt.subplots(num_joints, 1, figsize=(14, 2.5 * num_joints), sharex=True)
    if num_joints == 1:
        axes = [axes]

    boundaries = transitions + [len(df)]
    active_states = set()
    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        active_states.add(state)

    gmax = _grasp_max_from_states(active_states)

    # Collect state segments for labelling
    state_segments = []
    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        t0 = seg["time"].iloc[0]
        t1 = seg["time"].iloc[-1]
        state_segments.append((state, t0, t1))

    for j in range(num_joints):
        ax = axes[j]
        col = f"{prefix}_{j + 1}"
        # Background shading + state-colored signal line
        for state, t0, t1 in state_segments:
            color = _get_state_color(state, gmax)
            ax.axvspan(t0, t1, alpha=0.10, color=color)
        for i in range(len(transitions)):
            start = transitions[i]
            end = boundaries[i + 1]
            seg = df.iloc[start:end]
            state = seg["state_label"].iloc[0]
            color = _get_state_color(state, gmax)
            ax.plot(seg["time"], seg[col], linewidth=0.8, color=color)
        # Transition lines
        for idx in transitions[1:]:
            t = df["time"].iloc[idx]
            ax.axvline(x=t, color="red", linestyle="--", linewidth=0.7, alpha=0.7)
        ax.set_ylabel(f"J{j + 1}", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        _add_state_indicator(ax, active_states)

    axes[0].set_title(title)
    axes[-1].set_xlabel("Time (s)")
    axes[0].set_xlim(0, df["time"].iloc[-1])

    fig.supylabel(ylabel, fontsize=11)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


def plot_per_joint_overlay(
    df, transitions, prefix, num_joints, title, ylabel, save_path
):
    """Plot all joint signals overlaid on a single axes."""
    fig, ax = plt.subplots(figsize=(14, 6))

    boundaries = transitions + [len(df)]
    active_states = set()
    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        active_states.add(state)

    gmax = _grasp_max_from_states(active_states)

    for i in range(len(transitions)):
        start = transitions[i]
        end = boundaries[i + 1]
        seg = df.iloc[start:end]
        state = seg["state_label"].iloc[0]
        color = _get_state_color(state, gmax)
        t0 = seg["time"].iloc[0]
        t1 = seg["time"].iloc[-1]
        ax.axvspan(t0, t1, alpha=0.08, color=color)

    for j in range(num_joints):
        col = f"{prefix}_{j + 1}"
        ax.plot(
            df["time"],
            df[col],
            linewidth=0.8,
            color=JOINT_COLORS[j],
            label=f"Joint {j + 1}",
        )

    for idx in transitions[1:]:
        t = df["time"].iloc[idx]
        ax.axvline(x=t, color="red", linestyle="--", linewidth=0.5, alpha=0.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xlim(0, df["time"].iloc[-1])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=num_joints)
    _add_state_indicator(ax, active_states)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {save_path}")


def load_transfer_analysis(df):
    """Compute load transfer delta_F during UPLIFT state."""
    uplift = df[df["state_label"] == "UPLIFT"]
    if uplift.empty:
        print("  UPLIFT state not found — skipping load transfer analysis.")
        return

    t_start = uplift["time"].iloc[0]
    # mu_pre: mean wrench_norm in the 0.3s before uplift
    pre = df[(df["time"] >= t_start - 0.3) & (df["time"] < t_start)]
    if pre.empty:
        print("  Not enough data before UPLIFT for mu_pre.")
        return

    mu_pre = pre["wrench_norm"].mean()

    # mu_hold: mean wrench_norm during the top hold (last 0.3s of UPLIFT)
    t_end = uplift["time"].iloc[-1]
    hold = uplift[uplift["time"] >= t_end - 0.3]
    mu_hold = hold["wrench_norm"].mean()

    delta_F = mu_hold - mu_pre
    print(f"  Load transfer delta_F = {delta_F:.4f} N")
    print(f"    mu_pre  = {mu_pre:.4f} N  (0.3s before uplift)")
    print(f"    mu_hold = {mu_hold:.4f} N  (last 0.3s of uplift)")


def slip_detection(df):
    """Compute slip drop_ratio during EVALUATE state."""
    evaluate = df[df["state_label"] == "EVALUATE"]
    if evaluate.empty:
        print("  EVALUATE state not found — skipping slip detection.")
        return

    t_start = evaluate["time"].iloc[0]
    early = evaluate[(evaluate["time"] >= t_start) & (evaluate["time"] < t_start + 0.3)]
    late = evaluate[
        (evaluate["time"] >= t_start + 0.3) & (evaluate["time"] < t_start + 0.6)
    ]

    mu_early = early["wrench_norm"].mean() if not early.empty else 0.0
    mu_late = late["wrench_norm"].mean() if not late.empty else 0.0

    drop_ratio = (mu_early - mu_late) / max(mu_early, 1e-6)
    print(f"  Slip drop_ratio = {drop_ratio:.4f}")
    print(f"    mu_early = {mu_early:.4f} N  (first 0.3s)")
    print(f"    mu_late  = {mu_late:.4f} N  (next 0.3s)")

    if drop_ratio > 0.20:
        print("  >> Slip Detected")
    else:
        print("  >> Grasp Stable")


def analyze_csv(csv_path):
    """Run statistics and plot generation for one trial CSV."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    out_dir = os.path.dirname(os.path.abspath(csv_path))
    trial_name = os.path.splitext(os.path.basename(csv_path))[0]

    print(f"\nLoading: {csv_path}")
    df = load_and_preprocess(csv_path)
    print(f"  Rows: {len(df)}  |  Duration: {df['time'].iloc[-1]:.2f} s")
    print(f"  States: {df['state_label'].unique().tolist()}")

    transitions = find_state_transitions(df)

    # --- Statistical analysis ---
    print("\n=== Per-State Statistics ===")
    stats = compute_state_statistics(df)
    print(stats.to_string(index=False))
    stats_path = os.path.join(out_dir, f"{trial_name}_state_statistics.csv")
    stats.to_csv(stats_path, index=False)
    print(f"\n  Saved: {stats_path}")

    # --- Plots ---
    print("\n=== Generating Plots ===")
    plot_signal_vs_time(
        df,
        transitions,
        "wrench_norm",
        "Wrench Norm vs Time (State Segmented)",
        "Wrench Norm (N)",
        os.path.join(out_dir, "wrench_norm_plot.png"),
        yticks=list(range(0, int(np.ceil(df["wrench_norm"].max())) + 2)),
    )
    plot_signal_vs_time(
        df,
        transitions,
        "tau_ext_norm",
        "External Torque Norm vs Time (State Segmented)",
        "Tau Ext Norm (Nm)",
        os.path.join(out_dir, "tau_ext_norm_plot.png"),
        yticks=list(range(0, int(np.ceil(df["tau_ext_norm"].max())) + 2)),
    )
    plot_signal_vs_time(
        df,
        transitions,
        "ee_vz",
        "End-Effector Vz vs Time (State Segmented)",
        "ee_vz (m/s)",
        os.path.join(out_dir, "ee_vz_plot.png"),
    )
    plot_signal_vs_time(
        df,
        transitions,
        "Fn",
        "Normal Force (Fn) vs Time (State Segmented)",
        "Fn (N)",
        os.path.join(out_dir, "normal_force_plot.png"),
        yticks=list(range(0, int(np.ceil(df["Fn"].max())) + 2)),
    )
    plot_signal_vs_time(
        df,
        transitions,
        "Ft",
        "Tangential Force (Ft) vs Time (State Segmented)",
        "Ft (N)",
        os.path.join(out_dir, "tangential_force_plot.png"),
        yticks=list(range(0, int(np.ceil(df["Ft"].max())) + 2)),
    )
    plot_state_colored(
        df,
        transitions,
        os.path.join(out_dir, "wrench_norm_by_state.png"),
    )

    # --- Per-joint torque plots ---
    print("\n=== Per-Joint Plots ===")
    plot_per_joint(
        df,
        transitions,
        "tau_J",
        7,
        "Joint Torques (tau_J) per Joint vs Time",
        "Torque (Nm)",
        os.path.join(out_dir, "joint_torques_subplots.png"),
    )
    plot_per_joint_overlay(
        df,
        transitions,
        "tau_J",
        7,
        "Joint Torques (tau_J) — All Joints Overlaid",
        "Torque (Nm)",
        os.path.join(out_dir, "joint_torques_overlay.png"),
    )
    plot_per_joint(
        df,
        transitions,
        "tau_ext",
        7,
        "External Torques (tau_ext) per Joint vs Time",
        "Torque (Nm)",
        os.path.join(out_dir, "ext_torques_subplots.png"),
    )
    plot_per_joint_overlay(
        df,
        transitions,
        "tau_ext",
        7,
        "External Torques (tau_ext) — All Joints Overlaid",
        "Torque (Nm)",
        os.path.join(out_dir, "ext_torques_overlay.png"),
    )

    # --- Load transfer ---
    print("\n=== Load Transfer Analysis (UPLIFT) ===")
    load_transfer_analysis(df)

    # --- Slip detection ---
    print("\n=== Slip Detection (EVALUATE) ===")
    slip_detection(df)

    print("\nDone.")
    return out_dir


def expected_csv_for_trial(trial_dir):
    """Return the trial_NNN_signals.csv path expected inside a trial_NNN folder."""
    trial_name = os.path.basename(os.path.abspath(trial_dir))
    return os.path.join(trial_dir, f"{trial_name}_signals.csv")


def discover_trial_dirs(root_dir):
    """Find current workspace trial directories under object/trial folders."""
    patterns = [
        os.path.join(root_dir, "obj*", "trial_*"),
        os.path.join(root_dir, "t*", "trial_*"),
    ]
    trial_dirs = {
        path
        for pattern in patterns
        for path in glob.glob(pattern)
        if os.path.isdir(path)
    }
    return sorted(trial_dirs)


def unique_quarantine_path(root_dir, trial_dir):
    """Build a unique _corrupt_trials destination for a trial directory."""
    parent = os.path.basename(os.path.dirname(os.path.abspath(trial_dir)))
    trial_name = os.path.basename(os.path.abspath(trial_dir))
    quarantine_root = os.path.join(root_dir, "_corrupt_trials")
    base_dest = os.path.join(quarantine_root, f"{parent}__{trial_name}")
    dest = base_dest
    suffix = 2
    while os.path.exists(dest):
        dest = f"{base_dest}_{suffix:02d}"
        suffix += 1
    return dest


def quarantine_trial(root_dir, trial_dir):
    """Move a corrupt trial directory into _corrupt_trials."""
    dest = unique_quarantine_path(root_dir, trial_dir)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.move(trial_dir, dest)
    return dest


def record_for_trial(trial_dir, csv_path, status, output_path="", error_message=""):
    """Create one row for the batch report."""
    return {
        "parent_folder": os.path.basename(os.path.dirname(os.path.abspath(trial_dir))),
        "trial_folder": os.path.basename(os.path.abspath(trial_dir)),
        "csv_path": os.path.abspath(csv_path),
        "status": status,
        "output_or_quarantine_path": os.path.abspath(output_path) if output_path else "",
        "error_message": error_message,
    }


def discover_already_quarantined(root_dir, recorded_destinations):
    """Return report rows for corrupt trials already moved on a prior run."""
    quarantine_root = os.path.join(root_dir, "_corrupt_trials")
    if not os.path.isdir(quarantine_root):
        return []

    rows = []
    recorded = {os.path.abspath(path) for path in recorded_destinations if path}
    for path in sorted(glob.glob(os.path.join(quarantine_root, "*__trial_*"))):
        if not os.path.isdir(path) or os.path.abspath(path) in recorded:
            continue
        name = os.path.basename(path)
        parent, _, trial = name.partition("__")
        csv_path = os.path.join(path, f"{trial}_signals.csv")
        rows.append(
            {
                "parent_folder": parent,
                "trial_folder": trial,
                "csv_path": os.path.abspath(csv_path),
                "status": "already_quarantined",
                "output_or_quarantine_path": os.path.abspath(path),
                "error_message": "trial directory was already in _corrupt_trials",
            }
        )
    return rows


def run_batch(root_dir):
    """Run analysis for every trial in the current workspace."""
    root_dir = os.path.abspath(root_dir)
    trial_dirs = discover_trial_dirs(root_dir)
    records = []
    recorded_destinations = []
    success_count = 0
    quarantined_count = 0

    print(f"Workspace root: {root_dir}")
    print(f"Found {len(trial_dirs)} trial directories.")

    for trial_dir in trial_dirs:
        csv_path = expected_csv_for_trial(trial_dir)
        label = os.path.relpath(trial_dir, root_dir)
        print(f"\n--- Processing {label} ---")

        try:
            output_dir = analyze_csv(csv_path)
        except Exception as exc:
            error_message = f"{type(exc).__name__}: {exc}"
            print(f"  Corrupt/unplottable trial: {error_message}")
            try:
                quarantine_path = quarantine_trial(root_dir, trial_dir)
                recorded_destinations.append(quarantine_path)
                quarantined_count += 1
                records.append(
                    record_for_trial(
                        trial_dir,
                        csv_path,
                        "quarantined",
                        quarantine_path,
                        error_message,
                    )
                )
                print(f"  Moved to: {quarantine_path}")
            except Exception as quarantine_exc:
                quarantine_error = (
                    f"{error_message} | quarantine failed: "
                    f"{type(quarantine_exc).__name__}: {quarantine_exc}"
                )
                records.append(
                    record_for_trial(
                        trial_dir,
                        csv_path,
                        "quarantine_failed",
                        "",
                        quarantine_error,
                    )
                )
                print(f"  Quarantine failed: {quarantine_exc}")
            continue

        success_count += 1
        records.append(record_for_trial(trial_dir, csv_path, "success", output_dir))

    records.extend(discover_already_quarantined(root_dir, recorded_destinations))

    report_path = os.path.join(root_dir, "plot_generation_report.csv")
    pd.DataFrame(records, columns=REPORT_COLUMNS).to_csv(report_path, index=False)

    print("\n=== Batch Summary ===")
    print(f"  Trials attempted: {len(trial_dirs)}")
    print(f"  Successful plots: {success_count}")
    print(f"  Quarantined: {quarantined_count}")
    print(f"  Already quarantined: {sum(r['status'] == 'already_quarantined' for r in records)}")
    print(f"  Report: {report_path}")


def main():
    if len(sys.argv) > 1:
        csv_path = os.path.abspath(sys.argv[1])
        try:
            analyze_csv(csv_path)
        except Exception as exc:
            print(f"Error: {type(exc).__name__}: {exc}")
            sys.exit(1)
        return

    run_batch(os.path.dirname(os.path.abspath(__file__)))


if __name__ == "__main__":
    main()
