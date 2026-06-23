"""Calculated spin from measured ball speed and launch angle.

The OPS243 return carries no usable spin line: golf-ball dimples
(~0.3 mm) are Rayleigh-smooth at 24 GHz (λ = 12.4 mm) and the specular
point does not rotate with the ball, so rotation barely modulates the
echo within our ~65 ms ball dwell. Offline analysis of the 2026-06-08
TrackMan session (131 paired shots, PW-3h) found no spectral line at
the TrackMan spin frequency on any club after dechirp and carrier
removal — and the production envelope estimator's output had ~zero
within-club correlation with TrackMan spin (r ≈ +0.19).

What does work is impact kinematics. Spin loft tracks launch angle and
the friction impulse gives spin ∝ v·sin(spin loft), yielding a single
global formula with no club input:

    spin_rpm = 170 · ball_speed_mph · sin(LA)^1.2

Validation against the 2026-06-08 TrackMan truth set:
- TrackMan inputs (physics ceiling): 10.5% median error across the
  bag — better than an oracle per-club median table (12.0%)
- Leave-one-club-out blind: 11.3% median, 97/131 within 25%; the
  fitted (coefficient, exponent) were stable across all folds
- Simulated with production launch-angle noise (2.5° MAE two-ray):
  ~21% median expected live

Caveats: calibrated on one player/session (range balls). Attack-angle
style shifts spin loft at a fixed launch angle, so cross-player error
is likely a few points worse. Accuracy degrades at low launch angles
(cot(LA) error amplification) — the same clubs where measured launch
angle is weakest.
"""

import math
from typing import Optional

# Fitted on the 2026-06-08 TrackMan session (131 shots, PW-3h);
# leave-one-club-out stable at (170, 1.2) across all folds.
SPIN_COEFF_RPM_PER_MPH = 170.0
SPIN_LA_EXPONENT = 1.2

# Outside these bounds the kinematic model is extrapolating into
# regimes it was never calibrated on (top-spinned thins, pop-ups).
MIN_LAUNCH_ANGLE_DEG = 2.0
MAX_LAUNCH_ANGLE_DEG = 60.0

# Physical ceiling — beyond fresh-groove lob wedge territory.
MAX_SPIN_RPM = 13000.0


def calculated_spin_rpm(
    ball_speed_mph: float,
    launch_angle_deg: float,
) -> Optional[float]:
    """Kinematic spin estimate from ball speed and vertical launch angle.

    Use the true (cosine-corrected) ball speed when available — the
    model was calibrated against TrackMan ball speed.

    Returns None when inputs are missing or outside the calibrated
    range; callers should fall back to club-typical spin.
    """
    if ball_speed_mph is None or launch_angle_deg is None:
        return None
    if ball_speed_mph <= 0:
        return None
    if not MIN_LAUNCH_ANGLE_DEG <= launch_angle_deg <= MAX_LAUNCH_ANGLE_DEG:
        return None

    sin_la = math.sin(math.radians(launch_angle_deg))
    spin = SPIN_COEFF_RPM_PER_MPH * ball_speed_mph * sin_la**SPIN_LA_EXPONENT
    return float(min(spin, MAX_SPIN_RPM))
