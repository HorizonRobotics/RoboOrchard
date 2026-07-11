#!/usr/bin/env python3
"""Fit Coulomb friction from bidirectional constant-speed sweeps."""

from __future__ import annotations
import statistics
from dataclasses import dataclass


@dataclass
class CoulombFit:
    """Result of :func:`fit_coulomb` over the per-speed half-gaps."""

    speeds: list[float]
    half_gaps: list[float]
    intercept_rad: float
    slope_rad_per_rad_s: float
    r2: float


def hysteresis_half_gap(
    q_cmd: list[float],
    q_meas: list[float],
    v_sign: list[int],
    q_lo: float,
    q_hi: float,
    exclude_frac: float = 0.15,
    bins: int = 16,
    min_per_bin: int = 5,
) -> float:
    """Half the mean up-vs-down deflection gap of one sweep (radians).

    Samples are binned by commanded position inside the central band of
    the stroke (``exclude_frac`` of the range trimmed off each end, so
    post-reversal transients never contribute), and the deflection gap
    ``mean(d | v>0) - mean(d | v<0)`` is taken per bin — pairing by
    position is what cancels the position-dependent gravity sag. Bins
    that lack ``min_per_bin`` samples in either direction are skipped.

    Raises ``ValueError`` if no bin has enough samples both ways.
    """
    if not (len(q_cmd) == len(q_meas) == len(v_sign)):
        raise ValueError("q_cmd, q_meas and v_sign must be equally long")
    span = q_hi - q_lo
    if span <= 0.0:
        raise ValueError("q_hi must exceed q_lo")
    lo = q_lo + exclude_frac * span
    hi = q_hi - exclude_frac * span
    width = (hi - lo) / bins
    up_bins: list[list[float]] = [[] for _ in range(bins)]
    down_bins: list[list[float]] = [[] for _ in range(bins)]
    for qc, qm, s in zip(q_cmd, q_meas, v_sign, strict=True):
        if s == 0 or not lo <= qc < hi:
            continue
        b = min(int((qc - lo) / width), bins - 1)
        (up_bins if s > 0 else down_bins)[b].append(qc - qm)
    gaps = [
        statistics.fmean(u) - statistics.fmean(d)
        for u, d in zip(up_bins, down_bins, strict=True)
        if len(u) >= min_per_bin and len(d) >= min_per_bin
    ]
    if not gaps:
        raise ValueError(
            "no position bin has enough samples in both directions "
            "(sweep too short or exclude_frac too large)"
        )
    return statistics.fmean(gaps) / 2.0


def fit_coulomb(speeds: list[float], half_gaps: list[float]) -> CoulombFit:
    """Least-squares line half_gap = slope * speed + intercept.

    The v -> 0 intercept isolates the Coulomb deflection from every
    speed-proportional contribution. Needs at least two distinct
    speeds; with exactly two the fit is exact and r2 is reported as 1.0
    (use three or more speeds for a meaningful linearity check).
    """
    if len(speeds) != len(half_gaps) or len(speeds) < 2:
        raise ValueError("need matching speeds/half_gaps, at least 2")
    if len(set(speeds)) < 2:
        raise ValueError("speeds must not all be identical")
    n = len(speeds)
    mx = statistics.fmean(speeds)
    my = statistics.fmean(half_gaps)
    sxx = sum((x - mx) ** 2 for x in speeds)
    sxy = sum(
        (x - mx) * (y - my) for x, y in zip(speeds, half_gaps, strict=True)
    )
    slope = sxy / sxx
    intercept = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in half_gaps)
    ss_res = sum(
        (y - (slope * x + intercept)) ** 2
        for x, y in zip(speeds, half_gaps, strict=True)
    )
    r2 = 1.0 if ss_tot == 0.0 or n == 2 else 1.0 - ss_res / ss_tot
    return CoulombFit(
        speeds=list(speeds),
        half_gaps=list(half_gaps),
        intercept_rad=intercept,
        slope_rad_per_rad_s=slope,
        r2=r2,
    )


def tau_from_deflection(
    intercept_rad: float, stiffness_nm_per_rad: float, cap: float = 1.5
) -> tuple[float, bool]:
    """Convert a Coulomb deflection to a torque, clamped to [0, cap] Nm.

    A negative intercept (no measurable friction, or noise) maps to
    0.0. Returns ``(tau_c, clamped)`` where ``clamped`` reports that the
    raw value fell outside [0, cap].
    """
    if stiffness_nm_per_rad <= 0.0:
        raise ValueError("stiffness must be positive")
    raw = intercept_rad * stiffness_nm_per_rad
    if raw < 0.0:
        return 0.0, True
    if raw > cap:
        return cap, True
    return raw, False
