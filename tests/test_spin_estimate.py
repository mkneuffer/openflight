"""Tests for the kinematic calculated-spin model."""

import math

import pytest

from openflight.spin_estimate import (
    MAX_SPIN_RPM,
    SPIN_COEFF_RPM_PER_MPH,
    SPIN_LA_EXPONENT,
    calculated_spin_rpm,
)


class TestCalculatedSpin:
    def test_seven_iron_regime(self):
        """115 mph at 17.8 deg (TrackMan 7i medians) lands in 7i spin range."""
        spin = calculated_spin_rpm(115.0, 17.8)
        assert spin == pytest.approx(
            SPIN_COEFF_RPM_PER_MPH * 115.0 * math.sin(math.radians(17.8)) ** SPIN_LA_EXPONENT
        )
        assert 3500 <= spin <= 5500

    def test_wedge_spins_more_than_long_iron(self):
        """At realistic speed/LA pairs, wedges out-spin long irons."""
        wedge = calculated_spin_rpm(92.0, 28.0)
        long_iron = calculated_spin_rpm(130.0, 12.0)
        assert wedge > long_iron

    def test_monotonic_in_launch_angle(self):
        spins = [calculated_spin_rpm(110.0, la) for la in (10, 15, 20, 25, 30)]
        assert spins == sorted(spins)

    def test_monotonic_in_ball_speed(self):
        spins = [calculated_spin_rpm(v, 20.0) for v in (80, 100, 120, 140)]
        assert spins == sorted(spins)

    def test_capped_at_physical_maximum(self):
        assert calculated_spin_rpm(200.0, 55.0) == MAX_SPIN_RPM

    def test_none_outside_calibrated_launch_range(self):
        assert calculated_spin_rpm(110.0, 1.0) is None
        assert calculated_spin_rpm(110.0, -5.0) is None
        assert calculated_spin_rpm(110.0, 65.0) is None

    def test_none_for_missing_or_invalid_inputs(self):
        assert calculated_spin_rpm(None, 20.0) is None
        assert calculated_spin_rpm(110.0, None) is None
        assert calculated_spin_rpm(0.0, 20.0) is None
        assert calculated_spin_rpm(-10.0, 20.0) is None
