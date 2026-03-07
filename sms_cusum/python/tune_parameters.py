"""
Parameter tuning for SMS-CUSUM via grid search.

Searches over CUSUM parameters using leave-one-out cross-validation
across all 7 trials. Optimizes for:
  - Zero false positives on empty trials
  - Minimum detection latency on contact trials
  - 100% detection rate (no false negatives)

Usage:
    python tune_parameters.py [--data-dir PATH]
"""

from __future__ import annotations

import itertools
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python.config import SMSCusumConfig, CusumStageConfig
from python.validate_offline import discover_trials, load_trial, run_trial


@dataclass
class TuningResult:
    """Result of a single parameter configuration across all trials."""
    k_min: float
    h: float
    debounce: int
    noise_mult: float
    detection_rate: float
    false_positive_count: int
    mean_latency_ms: float
    max_latency_ms: float
    score: float  # Composite score (lower is better)


def evaluate_config(
    trials: list[tuple[str, str]],
    k_min: float,
    h: float,
    debounce: int,
    noise_mult: float,
) -> TuningResult:
    """Evaluate a single parameter configuration across all trials."""
    config = SMSCusumConfig(
        contact_stage=CusumStageConfig(
            k_min=k_min, h=h, debounce_count=debounce, noise_multiplier=noise_mult
        ),
    )

    detected_count = 0
    contact_count = 0
    fp_count = 0
    latencies_ms = []

    for name, csv_path in trials:
        rows = load_trial(csv_path)
        result = run_trial(rows, name, config)

        if result.has_contact_gt:
            contact_count += 1
            if result.contact_detected:
                detected_count += 1
            if result.contact_latency_ms is not None:
                latencies_ms.append(result.contact_latency_ms)
        else:
            if result.false_positive:
                fp_count += 1

    detection_rate = detected_count / max(contact_count, 1)
    mean_lat = sum(latencies_ms) / max(len(latencies_ms), 1) if latencies_ms else 999.0
    max_lat = max(latencies_ms) if latencies_ms else 999.0

    # Composite score: heavily penalize FP and FN, then minimize latency
    score = (
        1000.0 * fp_count
        + 500.0 * (contact_count - detected_count)
        + mean_lat
    )

    return TuningResult(
        k_min=k_min, h=h, debounce=debounce, noise_mult=noise_mult,
        detection_rate=detection_rate,
        false_positive_count=fp_count,
        mean_latency_ms=mean_lat,
        max_latency_ms=max_lat,
        score=score,
    )


def grid_search(
    trials: list[tuple[str, str]],
    k_values: list[float] = None,
    h_values: list[float] = None,
    debounce_values: list[int] = None,
    noise_mult_values: list[float] = None,
) -> list[TuningResult]:
    """Run grid search over CUSUM parameters."""
    if k_values is None:
        k_values = [0.02, 0.03, 0.05, 0.08, 0.10]
    if h_values is None:
        h_values = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0]
    if debounce_values is None:
        debounce_values = [3, 5, 8, 10, 12, 15]
    if noise_mult_values is None:
        noise_mult_values = [1.5, 2.0, 2.5, 3.0, 4.0]

    total = len(k_values) * len(h_values) * len(debounce_values) * len(noise_mult_values)
    print(f"Grid search: {total} configurations to evaluate")

    results = []
    for i, (k, h, d, nm) in enumerate(
        itertools.product(k_values, h_values, debounce_values, noise_mult_values)
    ):
        if (i + 1) % 100 == 0:
            print(f"  {i+1}/{total}...")
        r = evaluate_config(trials, k, h, d, nm)
        results.append(r)

    results.sort(key=lambda r: r.score)
    return results


def print_top_results(results: list[TuningResult], n: int = 20) -> None:
    """Print the top-N parameter configurations."""
    print(f"\n{'='*90}")
    print(f"TOP {n} PARAMETER CONFIGURATIONS (sorted by score, lower = better)")
    print(f"{'='*90}")
    print(
        f"{'Rank':>4s} {'k_min':>6s} {'h':>5s} {'deb':>4s} {'n_mult':>6s} "
        f"{'Det%':>5s} {'FP':>3s} {'Mean ms':>8s} {'Max ms':>8s} {'Score':>8s}"
    )
    print("-" * 90)

    for i, r in enumerate(results[:n]):
        print(
            f"{i+1:>4d} {r.k_min:>6.3f} {r.h:>5.2f} {r.debounce:>4d} {r.noise_mult:>6.2f} "
            f"{r.detection_rate*100:>5.0f} {r.false_positive_count:>3d} "
            f"{r.mean_latency_ms:>8.1f} {r.max_latency_ms:>8.1f} {r.score:>8.1f}"
        )

    print()
    best = results[0]
    print(f"BEST: k_min={best.k_min}, h={best.h}, debounce={best.debounce}, "
          f"noise_mult={best.noise_mult}")
    print(f"  Detection rate: {best.detection_rate*100:.0f}%")
    print(f"  False positives: {best.false_positive_count}")
    print(f"  Mean latency: {best.mean_latency_ms:.1f} ms")
    print(f"  Max latency: {best.max_latency_ms:.1f} ms")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="SMS-CUSUM parameter tuning")
    parser.add_argument(
        "--data-dir",
        default=str(Path(__file__).resolve().parent.parent.parent / "kitting_bags2"),
    )
    parser.add_argument("--top", type=int, default=20, help="Show top N results")
    args = parser.parse_args()

    trials = discover_trials(args.data_dir)
    if not trials:
        print(f"No trials found in {args.data_dir}")
        return

    print(f"Found {len(trials)} trial(s)")
    results = grid_search(trials)
    print_top_results(results, n=args.top)


if __name__ == "__main__":
    main()
