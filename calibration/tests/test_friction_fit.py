from __future__ import annotations

import pytest

from calibration.friction_fit import (
    fit_coulomb,
    hysteresis_half_gap,
    tau_from_deflection,
)


def test_hysteresis_half_gap_pairs_directions_at_same_position():
    positions = [0.2 + 0.05 * i for i in range(13)]
    q_cmd = []
    q_meas = []
    signs = []
    for sign in (1, -1):
        for position in positions:
            for _ in range(5):
                gravity_sag = 0.01 * position
                friction_sag = 0.02 * sign
                q_cmd.append(position)
                q_meas.append(position - gravity_sag - friction_sag)
                signs.append(sign)

    gap = hysteresis_half_gap(
        q_cmd,
        q_meas,
        signs,
        q_lo=0.0,
        q_hi=1.0,
        bins=10,
        min_per_bin=5,
    )

    assert gap == pytest.approx(0.02)


def test_hysteresis_half_gap_rejects_unpaired_data():
    with pytest.raises(ValueError, match="no position bin"):
        hysteresis_half_gap(
            [0.5] * 5,
            [0.49] * 5,
            [1] * 5,
            q_lo=0.0,
            q_hi=1.0,
        )


def test_fit_coulomb_recovers_zero_speed_intercept():
    fit = fit_coulomb(
        [0.05, 0.1, 0.2],
        [0.011, 0.012, 0.014],
    )

    assert fit.intercept_rad == pytest.approx(0.01)
    assert fit.slope_rad_per_rad_s == pytest.approx(0.02)
    assert fit.r2 == pytest.approx(1.0)


def test_fit_coulomb_requires_distinct_speeds():
    with pytest.raises(ValueError, match="must not all be identical"):
        fit_coulomb([0.1, 0.1], [0.01, 0.02])


@pytest.mark.parametrize(
    ("deflection", "stiffness", "cap", "expected"),
    [
        (0.01, 50.0, 1.5, (0.5, False)),
        (-0.01, 50.0, 1.5, (0.0, True)),
        (0.10, 50.0, 1.5, (1.5, True)),
    ],
)
def test_tau_from_deflection_clamps_to_safe_range(
    deflection, stiffness, cap, expected
):
    assert tau_from_deflection(deflection, stiffness, cap) == expected
