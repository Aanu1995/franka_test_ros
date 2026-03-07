"""
Offline validation of SMS-CUSUM contact detection against labeled CSV trial data.

Runs the SMS-CUSUM detector on each trial CSV file, compares detected contact
transitions to ground-truth `state_label` column, and reports:
  - Detection latency (samples and ms between ground truth and detected transition)
  - False positives (contact alarm when no contact exists)
  - False negatives (missed real contacts)
  - Per-trial summary table

Usage:
    python validate_offline.py [--data-dir PATH] [--plot]
"""

from __future__ import annotations

import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python.sms_cusum import SMSCusum, GraspState, DetectionEvent
from python.config import SMSCusumConfig, CusumStageConfig


@dataclass
class TrialResult:
    """Result of running SMS-CUSUM on one trial."""
    trial_name: str
    total_samples: int
    has_contact_gt: bool  # Does ground truth have CONTACT state?
    contact_detected: bool
    contact_gt_index: Optional[int]  # Ground truth contact transition index
    contact_det_index: Optional[int]  # Detected contact transition index
    contact_latency_samples: Optional[int]
    contact_latency_ms: Optional[float]
    false_positive: bool
    baseline_mean: float
    baseline_sigma: float
    k_effective: float
    cusum_at_alarm: float
    events: list[DetectionEvent]


def load_trial(csv_path: str) -> list[dict]:
    """Load a trial CSV file, return list of row dicts."""
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        return list(reader)


def find_gt_contact_index(rows: list[dict]) -> Optional[int]:
    """Find the first index of CONTACT_CONFIRMED or CONTACT in ground truth."""
    for i, row in enumerate(rows):
        if row["state_label"] in ("CONTACT_CONFIRMED", "CONTACT"):
            return i
    return None


def run_trial(
    rows: list[dict],
    trial_name: str,
    config: SMSCusumConfig,
    sample_rate: float = 250.0,
) -> TrialResult:
    """Run SMS-CUSUM on a single trial and compare to ground truth."""
    detector = SMSCusum(config)

    gt_contact_idx = find_gt_contact_index(rows)
    has_contact_gt = gt_contact_idx is not None

    contact_det_idx = None
    entered_closing = False

    for i, row in enumerate(rows):
        tau_ext_norm = float(row["tau_ext_norm"])
        gt_label = row["state_label"]

        # Enter closing when ground truth indicates gripper is closing
        if not entered_closing and gt_label in ("CLOSING_COMMAND", "CLOSING"):
            if detector.baseline_ready:
                detector.enter_closing()
                entered_closing = True

        event = detector.update(tau_ext_norm)

        if event is not None:
            if event.new_state == GraspState.CONTACT and contact_det_idx is None:
                contact_det_idx = i

    # Compute metrics
    contact_latency_samples = None
    contact_latency_ms = None
    if has_contact_gt and contact_det_idx is not None:
        contact_latency_samples = contact_det_idx - gt_contact_idx
        contact_latency_ms = (contact_latency_samples / sample_rate) * 1000.0

    false_positive = (not has_contact_gt) and (contact_det_idx is not None)

    return TrialResult(
        trial_name=trial_name,
        total_samples=len(rows),
        has_contact_gt=has_contact_gt,
        contact_detected=contact_det_idx is not None,
        contact_gt_index=gt_contact_idx,
        contact_det_index=contact_det_idx,
        contact_latency_samples=contact_latency_samples,
        contact_latency_ms=contact_latency_ms,
        false_positive=false_positive,
        baseline_mean=detector.baseline.mean,
        baseline_sigma=detector.baseline.sigma,
        k_effective=detector.contact_cusum.k_effective,
        cusum_at_alarm=detector.contact_cusum.statistic,
        events=detector.events,
    )


def discover_trials(data_dir: str) -> list[tuple[str, str]]:
    """Discover all trial CSV files. Returns [(trial_name, csv_path)]."""
    trials = []
    data_path = Path(data_dir)
    for shape_dir in sorted(data_path.iterdir()):
        if not shape_dir.is_dir():
            continue
        for trial_dir in sorted(shape_dir.iterdir()):
            if not trial_dir.is_dir():
                continue
            csv_files = list(trial_dir.glob("*_signals.csv"))
            if csv_files:
                name = f"{shape_dir.name}/{trial_dir.name}"
                trials.append((name, str(csv_files[0])))
    return trials


def print_results_table(results: list[TrialResult]) -> None:
    """Print a formatted results summary table."""
    print("\n" + "=" * 100)
    print("SMS-CUSUM OFFLINE VALIDATION RESULTS")
    print("=" * 100)
    print(
        f"{'Trial':<25s} {'Samples':>7s} {'Contact':>7s} {'Det?':>5s} "
        f"{'GT idx':>7s} {'Det idx':>7s} {'Latency':>10s} {'FP?':>4s} "
        f"{'mu':>6s} {'sigma':>6s} {'k_eff':>6s}"
    )
    print("-" * 100)

    for r in results:
        contact_str = "YES" if r.has_contact_gt else "no"
        det_str = "YES" if r.contact_detected else "no"
        gt_str = str(r.contact_gt_index) if r.contact_gt_index is not None else "-"
        det_idx_str = str(r.contact_det_index) if r.contact_det_index is not None else "-"
        latency_str = (
            f"{r.contact_latency_samples:d} ({r.contact_latency_ms:.0f}ms)"
            if r.contact_latency_ms is not None
            else "-"
        )
        fp_str = "FP!" if r.false_positive else "-"

        print(
            f"{r.trial_name:<25s} {r.total_samples:>7d} {contact_str:>7s} {det_str:>5s} "
            f"{gt_str:>7s} {det_idx_str:>7s} {latency_str:>10s} {fp_str:>4s} "
            f"{r.baseline_mean:>6.3f} {r.baseline_sigma:>6.4f} {r.k_effective:>6.4f}"
        )

    print("-" * 100)

    # Summary statistics
    with_contact = [r for r in results if r.has_contact_gt]
    without_contact = [r for r in results if not r.has_contact_gt]
    detected = [r for r in with_contact if r.contact_detected]
    false_pos = [r for r in without_contact if r.false_positive]
    latencies = [r.contact_latency_ms for r in detected if r.contact_latency_ms is not None]

    print(f"\nTrials with contact:    {len(with_contact)}")
    print(f"Contacts detected:     {len(detected)}/{len(with_contact)} "
          f"({100*len(detected)/max(len(with_contact),1):.0f}%)")
    print(f"False positives:       {len(false_pos)}/{len(without_contact)}")
    if latencies:
        print(f"Detection latency:     min={min(latencies):.0f}ms, "
              f"max={max(latencies):.0f}ms, mean={sum(latencies)/len(latencies):.0f}ms")
    print()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="SMS-CUSUM offline validation")
    parser.add_argument(
        "--data-dir",
        default=str(Path(__file__).resolve().parent.parent.parent / "kitting_bags2"),
        help="Path to kitting_bags2 directory with trial data",
    )
    parser.add_argument("--plot", action="store_true", help="Generate overlay plots")
    args = parser.parse_args()

    trials = discover_trials(args.data_dir)
    if not trials:
        print(f"No trials found in {args.data_dir}")
        return

    print(f"Found {len(trials)} trial(s) in {args.data_dir}")

    config = SMSCusumConfig()
    results = []

    for name, csv_path in trials:
        print(f"  Processing {name}...")
        rows = load_trial(csv_path)
        result = run_trial(rows, name, config)
        results.append(result)

    print_results_table(results)

    if args.plot:
        try:
            from python.plot_results import plot_trial_overlay
            for (name, csv_path), result in zip(trials, results):
                rows = load_trial(csv_path)
                output_dir = str(Path(csv_path).parent)
                plot_trial_overlay(rows, result, name, output_dir, config)
        except ImportError:
            print("Warning: plot_results.py not available, skipping plots.")


if __name__ == "__main__":
    main()
