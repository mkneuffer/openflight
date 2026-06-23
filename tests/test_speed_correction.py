"""Tests for the ball-speed cosine correction."""

import pytest

from openflight.speed_correction import correct_ball_speed, radial_speed_factor

D_FT = 5.0
H_FT = -4.0 / 12.0


class TestRadialSpeedFactor:
    def test_factor_below_one_for_lofted_launch(self):
        # A ball departing upward always reads slow on a low radar
        f = radial_speed_factor(19.0, 110.0, D_FT, H_FT)
        assert 0.95 < f < 1.0

    def test_typical_iron_compression_matches_observed_bias(self):
        # The validated datasets showed ~2.1-2.6 mph of compression at
        # iron speeds (~2-2.5% of ball speed)
        f = radial_speed_factor(19.0, 108.0, D_FT, H_FT)
        compression_mph = 108.0 * (1.0 - f)
        assert 1.5 < compression_mph < 3.5

    def test_higher_launch_compresses_more(self):
        f_wedge = radial_speed_factor(30.0, 90.0, D_FT, H_FT)
        f_iron = radial_speed_factor(17.0, 110.0, D_FT, H_FT)
        f_driver = radial_speed_factor(11.0, 150.0, D_FT, H_FT)
        assert f_wedge < f_iron < f_driver

    def test_farther_tee_compresses_more(self):
        # LOS flattens with distance while the velocity stays pitched up
        assert radial_speed_factor(18.0, 110.0, 6.5, H_FT) < radial_speed_factor(
            18.0, 110.0, 5.0, H_FT
        )

    def test_zero_launch_is_nearly_uncorrected(self):
        f = radial_speed_factor(0.0, 110.0, D_FT, H_FT)
        assert f > 0.995

    def test_degenerate_inputs_clamp(self):
        assert radial_speed_factor(19.0, 0.0, D_FT, H_FT) == 1.0
        assert 0.5 <= radial_speed_factor(44.0, 60.0, D_FT, H_FT) <= 1.0


class TestCorrectBallSpeed:
    def test_correction_raises_speed(self):
        corrected = correct_ball_speed(108.0, 19.0, D_FT, H_FT)
        assert corrected > 108.0
        assert corrected == pytest.approx(110.3, abs=0.8)

    def test_roundtrip_consistency(self):
        # Correcting then re-deriving the radial reading lands back
        true_speed = correct_ball_speed(108.0, 19.0, D_FT, H_FT)
        radial = true_speed * radial_speed_factor(19.0, true_speed, D_FT, H_FT)
        assert radial == pytest.approx(108.0, abs=0.15)
