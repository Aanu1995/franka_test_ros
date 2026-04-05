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
from python.config import SMSCusumConfig, CusumStageConfig, WrenchSlopeConfig
from python.wrench_slope import WrenchSlopeDetector


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
    # Secure grasp fields
    secure_grasp_detected: bool = False
    secure_grasp_step: Optional[int] = None


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
    method: str = "ewma",
    wrench_slope_config: Optional[WrenchSlopeConfig] = None,
) -> TrialResult:
    """Run SMS-CUSUM on a single trial and compare to ground truth."""
    detector = SMSCusum(config)
    ws_detector = None
    if method == "wrench_slope":
        ws_detector = WrenchSlopeDetector(wrench_slope_config or WrenchSlopeConfig())

    gt_contact_idx = find_gt_contact_index(rows)
    has_contact_gt = gt_contact_idx is not None

    contact_det_idx = None
    entered_closing = False
    entered_grasping = False
    current_grasp_step = 0
    prev_grasp_label = None
    secure_grasp_detected = False
    secure_grasp_step = None
    step_sample_count = 0
    step_total_samples = 0  # estimated total samples per step (set from first step)

    # Validate that wrench_norm column exists when wrench_slope method is selected
    if method == "wrench_slope" and rows and "wrench_norm" not in rows[0]:
        raise KeyError(
            f"Trial '{trial_name}' CSV is missing 'wrench_norm' column, "
            f"which is required for method='wrench_slope'. "
            f"Use --method ewma for CSVs without wrench data."
        )

    for i, row in enumerate(rows):
        tau_ext_norm = float(row["tau_ext_norm"])
        wrench_norm = float(row["wrench_norm"]) if method == "wrench_slope" else 0.0
        gt_label = row["state_label"]

        # Enter closing when ground truth indicates gripper is closing
        if not entered_closing and gt_label in ("CLOSING_COMMAND", "CLOSING"):
            if detector.baseline_ready:
                detector.enter_closing()
                entered_closing = True

        # Enter grasping when ground truth transitions to first GRASP state
        if not entered_grasping and gt_label.startswith("GRASP_"):
            if method == "wrench_slope" and ws_detector is not None:
                ws_detector.reset()
                ws_detector.begin_step(0)
            else:
                detector.enter_grasping()
            entered_grasping = True
            current_grasp_step = 0
            prev_grasp_label = gt_label

        # Detect grasp step transitions
        if entered_grasping and gt_label.startswith("GRASP_"):
            if gt_label != prev_grasp_label:
                # Record total samples for the step that just ended
                if prev_grasp_label is not None and step_total_samples == 0:
                    step_total_samples = step_sample_count  # learn from first step
                # Finalize previous step
                if prev_grasp_label is not None:
                    if method == "wrench_slope" and ws_detector is not None:
                        ws_result = ws_detector.finalize_step()
                        if ws_result.secure and not secure_grasp_detected:
                            secure_grasp_detected = True
                            secure_grasp_step = current_grasp_step + 1
                    else:
                        result = detector.finalize_grasp_step()
                        if result.detected and not secure_grasp_detected:
                            secure_grasp_detected = True
                            secure_grasp_step = current_grasp_step + 1
                # Begin new step
                current_grasp_step += 1
                if method == "wrench_slope" and ws_detector is not None:
                    ws_detector.begin_step(current_grasp_step)
                else:
                    detector.begin_grasp_step(current_grasp_step)
                step_sample_count = 0
                prev_grasp_label = gt_label

        # Feed only late-half samples to detectors (matching controller behavior)
        if entered_grasping and gt_label.startswith("GRASP_"):
            step_sample_count += 1
            # For the first step, we don't know total length yet — feed all samples
            # (the detector seeds EWMA on step 0, so extra early samples are harmless).
            # For subsequent steps, use the first step's length as the reference.
            if step_total_samples > 0:
                midpoint = step_total_samples // 2
                in_late_half = step_sample_count > midpoint
            else:
                in_late_half = True  # first step: feed all samples
            if in_late_half:
                if method == "wrench_slope" and ws_detector is not None:
                    ws_detector.update(wrench_norm)
                else:
                    detector.update(tau_ext_norm)
            event = None
        else:
            event = detector.update(tau_ext_norm)

        if event is not None:
            if event.new_state == GraspState.CONTACT and contact_det_idx is None:
                contact_det_idx = i

    # Finalize last grasp step
    if entered_grasping and not secure_grasp_detected:
        if method == "wrench_slope" and ws_detector is not None:
            ws_result = ws_detector.finalize_step()
            if ws_result.secure:
                secure_grasp_detected = True
                secure_grasp_step = current_grasp_step + 1
        else:
            result = detector.finalize_grasp_step()
            if result.detected:
                secure_grasp_detected = True
                secure_grasp_step = current_grasp_step + 1

    # Compute contact metrics
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
        secure_grasp_detected=secure_grasp_detected,
        secure_grasp_step=secure_grasp_step,
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


def print_secure_grasp_table(results: list[TrialResult], method: str) -> None:
    """Print secure grasp detection results per trial."""
    print(f"\n{'='*80}")
    print(f"SECURE GRASP DETECTION RESULTS (method: {method})")
    print(f"{'='*80}")
    print(f"{'Trial':<30s} {'Det?':>5s} {'Step':>6s} {'Note'}")
    print("-" * 60)

    for r in results:
        det_str = "YES" if r.secure_grasp_detected else "no"
        step_str = str(r.secure_grasp_step) if r.secure_grasp_step is not None else "—"
        note = ""
        if "object6" in r.trial_name and r.secure_grasp_detected:
            if r.secure_grasp_step is not None and r.secure_grasp_step <= 5:
                note = "FALSE POSITIVE"
        print(f"  {r.trial_name:<28s} {det_str:>5s} {step_str:>6s}  {note}")

    # Summary
    detected = [r for r in results if r.secure_grasp_detected]
    obj6_fp = [r for r in results
               if "object6" in r.trial_name and r.secure_grasp_detected
               and r.secure_grasp_step is not None and r.secure_grasp_step <= 5]
    print(f"\nSecure grasps detected: {len(detected)}/{len(results)}")
    print(f"Object6 false positives: {len(obj6_fp)}")
    print()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="SMS-CUSUM offline validation")
    parser.add_argument(
        "--data-dir",
        default=str(Path(__file__).resolve().parent.parent.parent / "kitting_bags"),
        help="Path to kitting_bags directory with trial data",
    )
    parser.add_argument("--plot", action="store_true", help="Generate overlay plots")
    parser.add_argument(
        "--method",
        choices=["ewma", "wrench_slope"],
        default="wrench_slope",
        help="Secure grasp detection method (default: wrench_slope)",
    )
    args = parser.parse_args()

    trials = discover_trials(args.data_dir)
    if not trials:
        print(f"No trials found in {args.data_dir}")
        return

    print(f"Found {len(trials)} trial(s) in {args.data_dir}")
    print(f"Secure grasp method: {args.method}")

    config = SMSCusumConfig()
    ws_config = WrenchSlopeConfig() if args.method == "wrench_slope" else None
    results = []

    for name, csv_path in trials:
        print(f"  Processing {name}...")
        rows = load_trial(csv_path)
        result = run_trial(rows, name, config, method=args.method,
                           wrench_slope_config=ws_config)
        results.append(result)

    print_results_table(results)
    print_secure_grasp_table(results, args.method)

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
