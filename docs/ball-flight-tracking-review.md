# OpenFlight Ball-Flight Tracking — Review and Improvement Plan

A deep-dive into how OpenFlight currently measures golf shots, how that
compares to commercial launch monitors, and where the project can be pushed
to be as accurate as physically possible inside the chosen hardware envelope.

> Generated 2026-05-06 from a full read of the codebase plus targeted
> research into TrackMan 4, FlightScope X3 / Mevo+, Foresight GCQuad,
> Garmin R10/R50, and Bushnell Launch Pro.

---

## 1. How OpenFlight tracks ball flight today

OpenFlight is a hybrid radar system. The only sensor that "sees" the ball
post-impact is the OPS243-A; everything else is either an angle sensor or
a model.

### 1.1 Sensor stack

| Sensor | Role | Method |
|---|---|---|
| **OPS243-A** (24.125 GHz Doppler) | Ball speed, club speed, spin rate | Rolling-buffer I/Q capture → FFT → mode-based ball-speed extraction → seam-modulation envelope FFT for spin |
| **K-LD7 vertical** (24 GHz, 2-Rx) | Vertical launch angle, club angle of attack | RADC raw I/Q → 2048-pt FFT → narrow ball-speed band → magnitude²-weighted phase-interferometric centroid |
| **K-LD7 horizontal** (24 GHz, 2-Rx) | Horizontal launch / face angle, club path | Same pipeline as vertical |
| **SEN-14262** (sound) | Impact trigger | GATE → OPS243 HOST_INT → ~10 µs trigger latency |
| **Pi camera (optional)** | Backup launch angle | Hough/YOLO ball detection, post-impact frame triangulation |

### 1.2 Ball-speed pipeline (OPS243)

`src/openflight/rolling_buffer/processor.py`

1. SEN-14262 fires HOST_INT → radar dumps 4096 I/Q samples (~136 ms at 30 ksps).
2. Two FFT passes are run on the buffer:
   - **Standard pass** — 128-sample non-overlapping windows, 4096-pt zero-padded
     FFT (~56 Hz timeline, ~0.1 mph bin width). Used for ball-speed detection.
   - **Overlapping pass** — 32-sample stepping (~937 Hz timeline) used for
     spin-detection windowing and club-speed search.
3. Ball speed is the **mode** of the binned 1-mph histogram of outbound peaks
   across windows (`_find_consistent_ball_speed`), not the max — this rejects
   single-window noise spikes (line `processor.py:851`).
4. Club speed: the highest-magnitude outbound peak before the ball, in the
   physical smash-factor window 0.67–0.85 × ball speed.

Worth noting:
- Cosine error from the radar bore-sight is **not** corrected. With the
  radar 3–5 ft directly behind the tee this is small (a 5° offset costs
  0.4 % speed), but a tee positioned off-axis can lose more.
- The DC mask is 150 bins (~15 mph). Below that nothing is reported — fine
  for ball flight but means putts and very short chips will not register.

### 1.3 Spin pipeline (OPS243 envelope demodulation)

Same file, `detect_spin()`. This is the most signal-processing-heavy code in
the repo and also the least reliable in production.

Steps:
1. Bandpass the complex I/Q ±700 Hz around the ball Doppler frequency
   (4-th order Butterworth, zero-phase filtfilt).
2. Take `|filtered|` to get the amplitude envelope.
3. Trim filter transients on both ends (1 / bandwidth seconds each).
4. Reject if envelope std/mean < 0.005 (no real seam modulation).
5. Apply Hann window, FFT 8192-pt, search 33–200 Hz (≈ 2 000–12 000 RPM).
6. Two rail guards:
   - **Lower-rail**: zero the bottom 5 bins of the valid range to kill
     residual DC leakage.
   - **Upper-rail**: reject peaks at the very top of the band unless SNR ≥ 8.
7. Fallback: autocorrelation override when envelope-FFT SNR is marginal.
8. Confidence is mapped from SNR + seam cycles + modulation depth, with a
   weak-modulation cap at 0.5.

This is solid signal processing — the rail guards in particular came from
real failure-mode analysis on captured data — but the project's own README
quotes a **50–60 % spin-detection rate**, and `Shot.spin_quality` regularly
falls back to "low" or no detection at all on driver shots. The fundamental
constraint is that backspin modulates the ball's radar return only weakly
(a fraction of a percent envelope variation), and a radar 3–5 ft from the
ball only sees a few cycles of seam rotation before the ball flies out of
beam.

### 1.4 Angle pipeline (K-LD7 RADC interferometry)

`src/openflight/kld7/radc.py`. RADC frames carry six 256-sample uint16
channels (`F1A I/Q`, `F2A I/Q`, `F1B I/Q`). Currently only the F1A and F2A
channels are used.

For each shot:
1. Server snapshots the K-LD7 ring buffer when the OPS243 fires
   (`server.py:954`, ring buffer is 6 s @ ~34 Hz).
2. Use the OPS243-measured ball speed to compute the **aliased** velocity
   bin (K-LD7 max speed is 100 km/h ≈ 62 mph, so a 150 mph ball wraps
   multiple times into the spectrum).
3. Find frames whose energy in the aliased ball band exceeds 3× the
   median energy of the buffer ("impact frames").
4. Per-bin angle θ = arcsin(Δφ × λ / (2π × d)) at the F1A/F2A phase
   difference, with the K-LD7's ~0.64 λ antenna spacing.
5. Per-frame angle = magnitude²-weighted centroid of bin angles within
   ±16 bins of the spectral peak, gated to bins ≥ 50 % of the peak. This
   is the wideband-monopulse formulation from Zhang et al. (2016), which
   is genuinely the right call for a range-spread, Doppler-smeared target.
6. SNR²-weighted average across surviving frames; soft penalty (×0.1
   weight) for frames whose peak bin is > 25 bins from the OPS-expected
   bin.
7. Hard physical bounds: vertical ∈ [0°, 45°], horizontal ∈ ±15°.
8. A second sanity guard in `server.py:radar_launch_is_plausible` rejects
   K-LD7 angles that disagree with a club/speed launch model by more than
   18–22° (club-family-specific window plus low-confidence bonus).

The same machinery is reused with `club_speed_mph` to extract club angle
of attack (vertical) and club path (horizontal). Spin axis is then
computed kinematically as `face_angle − club_path`, i.e. via the D-plane,
not measured directly.

### 1.5 Carry distance

Two paths in `launch_monitor.py` and `rolling_buffer/monitor.py`:

- `estimate_carry_distance(ball_speed, club)` — interpolates a
  ball-speed → carry table derived from TrackMan PGA averages, monotonised
  so carry can never decrease as ball speed rises. **No physics; a lookup.**
- `adjust_carry_for_launch_angle(...)` — subtracts 2.0 yd per degree below
  the club's optimal launch and 1.5 yd per degree above, scaled by the
  measurement confidence and capped at 10 % of base carry.
- `estimate_carry_with_spin(...)` — applies multiplicative penalties for
  spin-rate deviation from optimal (asymmetric, capped at 18 % low / 12 %
  high) and a small smash-factor penalty for off-centre hits.

Crucially, **OpenFlight does not run a physics simulation**. There is no
Runge-Kutta integration of drag + Magnus + gravity, no air-density
adjustment, no spin-axis effect on lateral carry, no apex / hang-time /
landing-angle output. Carry is a tuned regression of TrackMan averages.

### 1.6 What you actually get out of a shot

From `launch_monitor.Shot` and `shot_to_dict`:

```
ball_speed_mph, club_speed_mph, smash_factor,
launch_angle_vertical, launch_angle_horizontal, launch_angle_confidence,
club_angle_deg (AoA), club_path_deg, spin_axis_deg,
spin_rpm, spin_confidence, spin_quality,
estimated_carry_yards, carry_range (±10% / ±5%),
peak_magnitude, angle_source
```

What's **missing** vs commercial monitors:
- Apex height, hang time, landing angle
- Side carry / total side
- Total distance (carry + roll)
- Spin loft, dynamic loft, face angle (we have a proxy via H-launch)
- Swing path / swing plane / lie angle / impact location
- Curve direction strength as a yards-of-curve number

---

## 2. How accurate is OpenFlight today?

There is no formal calibration study in the repo — the only "ground truth"
comparisons live in `session_logs/`. The honest answer has to come from
first-principles bounds plus the project's own observations.

### 2.1 Ball speed — the strongest measurement

- **Sensor floor**: OPS243 rated ±0.5 % per the datasheet at the bin
  resolution we use (~0.1 mph at 30 ksps × 4096-pt FFT).
- **In practice**: probably ±1 mph for most shots. Failure modes:
  - **Cosine error** off-axis (uncorrected).
  - **Aliasing** above 208 mph at 30 ksps (extreme — only relevant to elite
    drivers).
  - **Mode-based extraction** can bias toward 1-mph bin centres on shots
    with few outbound windows (e.g. very short windows after late triggers).

This is comparable to TrackMan/FlightScope's quoted ±0.5–1 mph. It's the
one number you can quote with confidence.

### 2.2 Club speed — accurate but lossy

- **Method**: highest-magnitude outbound peak before ball impact, in the
  smash-factor band. With a sound trigger at impact and `S#0`, only ~5–6 ms
  of pre-trigger history is captured, so the club is sometimes outside the
  buffer entirely. The speed-trigger fallback uses the trigger speed itself
  as the club speed.
- **Failure modes**:
  - Club is missed if smash factor is unusually high (PW thin hits) or
    low (toe strikes < 0.67) — the band cutoff rejects them.
  - Pre-trigger segments default to 16, giving ~68 ms of pre-trigger at
    30 ksps. Club typically reaches max speed ~10–20 ms before impact, so
    it's *usually* in the buffer.
  - Cosine error: the club face is rarely moving directly at the radar.

Realistic envelope: ±1–2 mph when detected; sometimes simply not
detected (`smash_factor` is `None`).

### 2.3 Launch angle (vertical) — heavily dependent on ball markings

- **K-LD7 RADC**: ±2–4° on **good** captures. Single-frame detections —
  which are the common case at 34 fps for a ball that transits the beam in
  ~50 ms — get confidence 0.40 + (SNR/10) × 0.50, capped at 0.90. The
  Cramér-Rao bound on a 2-element interferometer at SNR = 10 dB is
  ≈ 1° angular precision; in practice with seam modulation, multipath, and
  the OPS-bin penalty soft anchor, real-world variance is closer to
  ±2–4°.
- **Camera fallback**: lower confidence (Hough+ByteTrack on 60 fps frames
  has its own pixel-discretisation bias).
- **Estimated fallback**: club + ball-speed model, ±5° at 0.2–0.5
  confidence — basically a guardrail.
- **Net**: when the ball is marked with reflective tape or a painted
  stripe, K-LD7 vertical is usable for fitting decisions (±2°). With a
  bare ball, expect closer to ±5° and frequent fallback to the model.

### 2.4 Horizontal launch / club path / spin axis

- **K-LD7 horizontal**: same precision profile as vertical (±2–4° best
  case), but the orientation guard rejects only |angle| > 15°, which is
  fine for golf but means weakly-detected angles can leak through.
- **Spin axis**: derived as `H-launch − club_path`. Both inputs are
  ±2–4°, so spin axis accumulates **±4–6° at best**. Compare to TrackMan's
  ±0.5° on outdoor (directly measured from spin-induced phase modulation).

### 2.5 Spin rate — the weakest measurement

- The README quotes **50–60 % detection rate**. When detection fires:
  - "high" confidence (SNR ≥ 8, ≥ 5 cycles): probably ±300–500 RPM.
  - "medium" confidence: ±500–1000 RPM.
  - "low" confidence: borderline noise, falls back to model.
- Drivers are the worst case: only 2–3 seam cycles fit in the available
  ball-flight window before the ball leaves the beam. Wedges and short
  irons (more cycles, slower ball) fare best.
- The rail guards are aggressive — they correctly reject pile-ups at
  ~2700 RPM and ~12 000 RPM, but they also throw away marginal real shots.
  The autocorrelation fallback catches some of those but not all.
- **No spin-axis measurement from radar** — purely D-plane derived.

### 2.6 Carry distance

A regression trained on TrackMan **averages** with optional spin and
launch-angle corrections, capped at ±10–15 % deviation. For a player who
matches the regression prior (average launch + average spin for their ball
speed), carry is probably within ±5 % of truth. For a player far from
average (e.g. a strong-loft driver hitter at 220 ball speed but only 1500
RPM, or an iron player hitting 25° launch on a 7-iron), the regression
can be off by 10–15 %.

The lack of a physics simulator is the largest single source of
preventable carry error.

### 2.7 Honest summary table

| Metric | OpenFlight (today) | Commercial reference (TrackMan / Foresight) |
|---|---|---|
| Ball speed | ±1 mph | ±0.5 mph |
| Club speed | ±1–2 mph (when found) | ±0.5 mph |
| Launch angle (V) | ±2–4° (marked ball) / ±5° fallback | ±0.5° |
| Launch direction (H) | ±2–4° | ±0.3° |
| Club path | ±2–4° | ±0.5° |
| Spin axis | ±4–6° (derived) | ±0.5° (measured) |
| Spin rate | ±300–1000 RPM, 50–60 % detection | ±10 RPM, 100 % |
| Carry distance | ±5–15 % (regression) | ±2 yd |
| Apex / landing / hang time | Not provided | Provided |

This is the right framing for the project: **OpenFlight is a
$540 ball-speed monitor with credible angle and reasonable spin, not a
$25 000 tour monitor.** Most of the gap is fundamental hardware
(2-element vs 26-element phased arrays, no high-speed dimple cameras),
but a meaningful chunk is software that hasn't been built yet.

---

## 3. How commercial monitors do it (for reference)

Knowing what the gap actually is matters before deciding what to build.

### 3.1 TrackMan 4 — phased-array dual radar + camera

- Dual Doppler radars: short-range high-resolution (club + impact),
  long-range high-accuracy (ball flight).
- Each radar is a **multi-element phased array** (publicly reported
  16–26 elements depending on generation).
- Time-and-space-synchronised with an HD camera ("OERT" — Optically
  Enhanced Radar Tracking).
- Reports 40+ parameters including dynamic loft, spin loft, attack angle,
  swing direction, swing plane, low point, impact location.
- Spin axis is **directly measured** in outdoor mode via phase modulation
  of the Doppler signal.
- Ground-truth-grade outdoors; indoor accuracy degrades because the radar
  needs ~6 m of ball flight to lock spin.

Why phased arrays matter for this project: every angle-estimation
technique that beats simple phase-difference monopulse — MUSIC, ESPRIT,
beamspace MUSIC, Capon, sum-and-difference monopulse — needs ≥ 3 elements.
With the K-LD7's 2 elements per axis, OpenFlight is **at the floor of
array-radar DOA estimation**, with angle accuracy bounded by the
Cramér-Rao limit ~ 1 / (SNR · √N_snapshots). That's not a software bug;
it's information theory.

### 3.2 FlightScope X3 / Mevo+ — 3D Doppler + image fusion

- Patented "Fusion Tracking" combines 3D Doppler radar with synchronised
  cameras.
- Mevo+ is the closest commercial parallel to OpenFlight in price/role
  but uses a multi-element radar plus an HD camera, not a sound trigger.
- Spin is measured through dimple/seam modulation in the radar return
  combined with image processing (the patent literature describes
  cross-correlating dimple patterns between radar Doppler and image
  frames).
- Quoted accuracy is in the same league as TrackMan for ball metrics.

### 3.3 Foresight GCQuad — quadrascopic camera

- Four high-speed cameras at up to 10 000 fps each, ~200 frames in the
  first 30 cm of flight.
- Spherical Correlation™ — cross-correlates the dimple pattern between
  successive frames to recover spin rate **and spin axis** directly.
- Best-in-class on club face data because the cameras *see* the face
  geometry; radar systems infer it.
- Indoor-friendly because it doesn't need ball-flight tracking distance.

### 3.4 Garmin R10 / Bushnell Launch Pro / Skytrak+ / R50

- R10: single Doppler radar + accelerometer for club; widely reported to
  read distances 20–40 % short into a net indoors (the radar can't see
  enough flight to model carry well).
- Launch Pro / GC3-class: photometric stereo, similar method to GCQuad
  but with two cameras and slightly fewer parameters.
- R50: three high-speed cameras, no radar.

### 3.5 What this means for OpenFlight

The two technical paths are clear:
- **Stay radar-first** and fight for every dB of SNR and every °/% of
  precision the 2-channel K-LD7 will yield. Realistic ceiling: ±0.5 mph
  ball, ±2° angles, ±300 RPM spin on most shots, fallback for the rest.
- **Add a high-speed camera path** (single global-shutter camera at
  240–500 fps near impact) and shift spin-rate, launch-angle, and spin-axis
  measurement to the camera. This is what every commercial monitor in
  the consumer price band has done over the last 5 years.

Both are addressed in §5.

---

## 4. Software-only improvements (no new hardware)

Ordered by leverage / cost ratio. These are inside the existing OPS243 +
2× K-LD7 + sound-trigger envelope.

### 4.1 Replace carry-distance regression with a real ballistic simulator (HIGH)

Current state: lookup-table interpolation with confidence-scaled add-ons.

What to build: 4-th order Runge-Kutta integration of

```
F = -mg · ĵ  -  ½ ρ v² A C_d(v) · v̂  +  ½ ρ v² A C_l(v, ω) · (ŝ × v̂)
```

with:
- ρ from temperature + altitude (default sea-level if unknown).
- C_d and C_l as functions of Reynolds number and spin factor S = rω/v
  — published curves exist for Pro V1-class balls (Aoki et al.
  2009, Bearman & Harvey 1976 for the dimple-flow model).
- Spin axis from `spin_axis_deg` rotates the lift vector — this is what
  makes draws and fades curve correctly.
- Drag includes a "reverse Magnus" tweak above S ≈ 0.4 (golf-ball
  specific, see Sports Engineering 2020).

Outputs you immediately get for free:
- Apex height
- Hang time
- Landing angle (descent angle)
- Side carry / total side
- Curve magnitude in yards
- "Total distance" with a bounce-and-roll model (simple coefficient-of-
  restitution + rolling-resistance, fairway/rough/green presets).

**Why this matters most**: today the system has measured launch angle,
spin rate, and spin axis for many shots and immediately throws them at
a regression that doesn't use any of it physically. A simulator turns
those measurements into actual ball-flight predictions, and surfaces
the cases where measurements disagree (e.g. driver with 1 200 RPM should
balloon to ~180 yd carry — the regression is currently capped at 12 %
loss for excessive spin, which is wrong for outliers).

Effort: 1–2 weeks. Pure Python, no new hardware. Validation against
TrackMan-derived shots in `session_logs/` is straightforward.

### 4.2 Cosine-error correction for OPS243 ball speed (MEDIUM)

The OPS243 measures the radial component of velocity. A ball flying at
angle θ from the radar bore-sight reads `v · cos(θ)`. With a 4 ft
behind-tee position and 12° launch + 0° push, the ball is a few degrees
above the radar bore-sight at the moment of measurement, so cosine
correction is small (< 1 %). But:

- Range positioning errors of 6–10° in azimuth (radar pointed at the
  wrong spot) cost 0.5–1.5 %.
- After ball flight starts curving, the angle off-axis grows. The peak
  reading is taken near impact so this is small.

Implementation:
- When K-LD7 horizontal angle is available, divide ball speed by
  `cos(launch_h_deg + radar_yaw_deg)` from the OPS243 yaw setting.
- Same with vertical when available.
- Fall back to today's behaviour when angles aren't available.

Effort: an afternoon. Adds ~0.5 mph of typical accuracy on shots with
detected angles.

### 4.3 Joint Doppler + DOA maximum-likelihood angle estimator (MEDIUM-HIGH)

For each K-LD7 frame, instead of `FFT → pick peak bin → read phase`, do
a maximum-likelihood search over `(velocity, angle)` directly on the
2-channel raw I/Q. The cost function is the negative log-likelihood of
the 2-channel signal given a complex sinusoid at angular velocity ω with
phase difference φ. For a 256-sample frame this is a 2D grid search and
is provably optimal at low SNR / few-snapshot — exactly the OpenFlight
regime.

Reference: *Joint Doppler and DOA Estimation Using (Ultra-)Wideband
FMCW Signals*, Signal Processing 165 (2019).

Expected gain: 0.5–1° on ±2–4° baseline. Worth doing once §4.1 lands so
the simulator can absorb the improvement.

### 4.4 Use the F1B channel — delta-frequency interferometry (MEDIUM)

The K-LD7 RADC payload reserves channels for `f1a`, `f2a`, `f1b`. The
project currently uses only F1A and F2A. The F1B channel is at a slightly
different carrier and is what enables target ranging and angle
disambiguation on the K-LD7 hardware.

Combining the per-channel angle estimates (with 2π disambiguation) is
well-studied for synthetic-aperture radar and gives an independent
angle constraint that breaks the ±λ/(2d) ambiguity at high angles. For
OpenFlight this is most useful at the upper end of the launch-angle
window (≥ 35°) where today's interferometry can wrap.

Effort: 1 week of signal processing + a test rig. RFbeam's carrier
spacing needs to be confirmed from the datasheet.

### 4.5 Multi-frame phase tracking / Kalman launch model (LOW EFFORT, LOW GAIN today)

When the ball is visible for 2–3 frames, fit a constant-or-linear angle
model with SNR-weighted least squares, reject `>Nσ` outliers. This is
Kalman-filter territory but in the cluster-of-3-frames regime a closed-
form least-squares fit is enough.

Today the ball is usually visible for 1–2 frames per shot at 34 Hz
RADC, so this only fires occasionally. The same code is the substrate
for §5.1 (more frames), so build it now.

### 4.6 Spin detection improvements (HIGH leverage on a hard problem)

Spin is the project's worst metric. Several wins are available without
new hardware:

**4.6a — Phase-based spin detection.** Instead of FFT on the amplitude
envelope, look at the phase residual after removing the linear-in-time
Doppler trend. The seam modulates phase as well as amplitude, and at
24 GHz the phase shift per radian of seam rotation is much larger than
the amplitude change. Patented by FlightScope/TrackMan (US 10,775,492 et
al.) — the technique is public, the patents are on specific
implementations. This is a ~30 % improvement in detection rate based on
literature.

**4.6b — Coherent integration over the whole capture.** Currently the
spin window starts at `ball_timestamp_ms` and ends at the buffer end —
typically ~70 ms post-impact at 30 ksps. Re-armed pre-trigger captures
have ~85 ms of post-trigger data at S#16, and the trigger itself fires
~10 µs after impact. So the available ball-signal window is about 80 ms
at most — that's 4–8 seam cycles for a driver at 2 500 RPM. Increase
post-trigger segments (S#0 with sound trigger gives ~136 ms, ~6–14
cycles), at the cost of less pre-trigger context for club extraction.

**4.6c — Per-shot template adaptation.** The seam frequency is
1× spin rate, but the *spectrum* of seam modulation is broader than a
single tone (the seam isn't a perfect sinusoid). Cross-correlate the
envelope against a parametric seam template with a few free parameters
(rotation phase, decay constant) and let the optimizer find the best
match. This recovers signal in the marginal SNR range.

**4.6d — Reject mid-band rails.** The current rail guards cover the
bottom 5 bins (DC leakage) and top 2 bins (filter shoulder). Mid-band
"rails" still happen at ~3 000 and ~6 000 RPM where harmonics of body
motion or trigger-induced ringing show up. Add a check that the peak's
SNR is ≥ 1.5× the SNR of the closest neighbour 200 Hz away — kills
isolated mid-band noise spikes.

Combined gain: 50–60 % → 70–80 % detection rate, probably ±200–500 RPM
on the detected portion.

### 4.7 Tighten the carry-confidence story (LOW effort)

Today `carry_range` is a flat ±5–10 % whether or not the model agrees
with the measurements. Once §4.1 is built, propagate the per-input
confidences (ball speed σ, launch σ, spin σ) through the simulator with
a small Monte Carlo (1000 samples) to produce a real probabilistic
range. UI gets a believable error bar.

### 4.8 Calibration & ground-truth harness (CRITICAL infrastructure)

There is no way today to know how good any change is. Build:

- A calibration mode that records every shot with all OPS243 + K-LD7
  raw data plus a manual entry field for "the launch monitor I'm
  comparing against says X". Five sessions on a public TrackMan or
  Foresight is enough to build a real error model.
- A regression-test harness that replays archived `session_logs/*.jsonl`
  through the current code and compares predicted vs known outputs.
- A "diff-mode" between two algorithm versions that highlights shots
  whose answer changed > some threshold.

This is unglamorous but every other improvement in this list is much
more valuable when you can quantify it.

### 4.9 Lower-priority software wins

- **Aliasing-aware ball speed at 30 ksps** — reject ball-speed
  candidates whose magnitude is suspiciously similar at `f` and
  `f_sample - f`. Helps the rare 200+ mph driver shot.
- **Better "minimum reading" handling** — today `min_speed = 15 mph`.
  Putting and short chips fall below this; if the project ever wants
  putting, it needs a separate low-DC-mask path.
- **Reflective-marker detection auto-tune** — README notes that a
  reflective patch helps angles but kills spin. Detect which condition
  the ball is in (specular pulses vs sinusoidal) and switch the spin
  algorithm accordingly rather than failing.

---

## 5. Hardware-assisted improvements (small additions, large gains)

The 2-channel K-LD7 will not match a 26-element phased array. Past a
certain point the only way to materially improve angle and spin is to
add a different sensor. Listed cheapest first.

### 5.1 Faster K-LD7 frame rate — software-only-ish (FREE-ISH, HIGH)

The K-LD7 RADC stream rate is documented as 34 fps in the codebase but
the hardware will run faster at lower range/speed settings. A ball that
crosses the beam in ~50 ms gets 1–2 frames at 34 fps; at 60–80 fps it
gets 3–4. That's the difference between a single-frame fit (current
single-frame confidence floor 0.40) and a multi-frame least-squares fit
(potentially 0.85+). Worth a session in the lab characterising the
trade-off vs frame quality.

### 5.2 Add a single global-shutter camera at impact (BIGGEST UPGRADE)

A Raspberry Pi HQ camera with a global-shutter alternative (Arducam
OV9281, ~$50) at 240+ fps gives:

- **Spin rate** by tracking dimple/marking rotation between frames
  (OpenCV `cv2.phaseCorrelate` or a learned model). 5–6 frames in the
  first 30 cm of flight is enough for ±50 RPM.
- **Spin axis** by tracking marking orientation in 3D (requires marked
  ball or a known logo).
- **Launch angle (V + H)** by triangulating ball position across
  consecutive frames, with sub-degree precision.
- **Smash efficiency / impact location** if the camera can see the
  club face on a side view.

This is the **single change that closes the largest accuracy gap** for
the smallest dollar amount. The codebase already has a
`camera_tracker.py` and a Picamera2 path — it currently does Hough-
based ball detection but doesn't extract spin or do high-precision
launch angle. Adding spin extraction here would replace the radar
spin path entirely on shots where the camera fires.

Effort: 2–3 weeks for spin + launch angle, integrated as the primary
source with K-LD7/OPS243 as fallback. The hardest part is shutter
synchronisation with the sound trigger.

### 5.3 Second OPS243 from below the ball (MEDIUM)

A second OPS243 looking up from an angled mount (~30° elevation) gives
an independent ball-speed measurement on a different cosine-error axis.
Two radial speeds + the geometry recover the actual 3D velocity vector
(which is `v_x, v_y, v_z` — and from that, true speed plus vertical
launch angle without K-LD7).

This is also how TrackMan's dual-radar architecture works (one short-
range one long-range, but synchronised in a different geometry).

Cost: ~$250 + mount. Worth it primarily if K-LD7 angle is the bottleneck
and adding a camera is undesirable.

### 5.4 Replace K-LD7 with a higher-element-count sensor (HIGH cost, HIGH gain)

Texas Instruments AWR/IWR series 77 GHz mmWave radars (4 Tx × 4 Rx
virtual array — 16 elements) cost $100–300 for a starter board. They:

- Resolve 1° angles directly (vs ±2–4° from K-LD7).
- Run real MUSIC/ESPRIT — 1° to 0.5° in good SNR.
- Give range + Doppler + angle in one chirp.
- Support tracking *during* the swing, not just at impact.

This is the hardware path TrackMan and FlightScope took. Cost is a few
hundred dollars and a substantial software port (TI's mmWave SDK is C
on a Cortex-R; there's a Python toolchain via OpenRadar but it's
research-quality).

This would shift OpenFlight from "kitchen-table launch monitor" to
"genuine consumer-grade launch monitor" and is the ceiling of what's
buildable for under $1000.

### 5.5 Higher-speed sound trigger / shutter with fewer false positives

The SEN-14262 + HOST_INT path is ~10 µs and clean. The remaining
trigger noise is a nearby player's impact or a club drop. Adding:

- A **piezo contact mic** on the hitting mat (10 ms of advance warning
  about contact, in addition to airborne sound).
- A **photoelectric beam break** at the tee (highest reliability, no
  false positives, ~$5 of parts).

…makes the trigger robust enough to leave running unattended and lets
the camera path (5.2) start exposing 5–10 ms before impact, giving a
much better post-impact frame sequence.

---

## 6. Recommended roadmap

If the goal is "make this as accurate as possible" and budget matters
more than speed-to-market:

**Phase 1 — Software only (1 month).**
1. Build the calibration / replay harness (§4.8) **first**.
2. Real ballistic simulator (§4.1) — biggest single accuracy lift today.
3. Cosine correction (§4.2) — half-day, gets us to "honest" ball speed.
4. Spin improvements 4.6a–d (§4.6) — phase-domain detection + mid-band
   rails + adaptive template + S#0 long-window.

After Phase 1, OpenFlight should be: ±0.5 mph ball, ±2° angles when
detected, ±200–500 RPM spin at 75 % detection, real apex/hang/landing
numbers. That's competitive with the Garmin R10 at one-third the price.

**Phase 2 — Add a camera (3 weeks of work, $50 of hardware).**
1. Global-shutter camera + spin-from-dimple (§5.2).
2. Camera-based launch angle replaces K-LD7 vertical for marked balls.
3. Drop spin-rate dependence on radar entirely; keep radar as fallback.

After Phase 2: ±0.5 mph ball, ±0.5° launch angle (camera), ±100 RPM
spin. That's competitive with the Bushnell Launch Pro.

**Phase 3 — Optional, only if camera path stalls.**
1. Joint Doppler+DOA ML estimator on K-LD7 (§4.3).
2. F1B channel (§4.4).
3. Higher-element-count radar (§5.4).

Each phase has a falsification criterion built into the calibration
harness from Phase 1: if the actual measured improvement on the test
set is below threshold, don't ship the change.

---

## 7. Sources & References

### Commercial launch monitors
- TrackMan 4 — [Tech specs](https://www.trackman.com/golf/launch-monitors/tech-specs), [product page](https://www.trackman.com/golf/launch-monitors/trackman-4)
- FlightScope X3C / Mevo+ — [Tech overview](https://flightscope.com/pages/technology), [Mevo+](https://flightscope.com/pages/mevo-plus-launch-monitor), [X3C product page](https://flightscope.com/products/flightscope-x3c)
- Foresight GCQuad — [Foresight overview](https://www.foresightsports.com/pages/gcquad), [tech inside](https://www.foresightsports.asia/en/inside-the-technology-powering-foresight-launch-monitors/)
- Garmin R10 / R50 — [Garmin comparison](https://www.garmin.com/en-US/blog/outdoor/3-key-differences-between-garmin-approach-r50-and-r10/)
- Bushnell Launch Pro — [Bushnell tech](https://www.bushnellgolf.com/launch-pro/what-we-measure)
- 2026 monitor roundup — [BirdieReport](https://www.birdiereport.com/blog/best-golf-launch-monitors-2026/), [GolfMonthly](https://www.golfmonthly.com/best-golf-deals/best-golf-launch-monitors-213610)

### Signal processing & physics
- Spin via Doppler patents — [US20140191896A1](https://patents.google.com/patent/US20140191896) (ball spin rate measurement), [US Patent 10,775,492](https://patents.justia.com/patent/10775492) (golf ball spin axis)
- Doppler radar baseball/golf evaluation — [WSU thesis (Martin)](https://baseball.physics.illinois.edu/trackman/JasonMartinThesisWSU.pdf)
- Photometric vs Doppler comparison — [Uneekor blog](https://uneekor.com/blogs/blog/photometric-vs.-doppler:-which-launch-monitor-technology-delivers-the-most-accurate-golf-data)
- Golf ball flight dynamics — [Union College](https://www.math.union.edu/~wangj/courses/previous/math238w13/Golf%20Ball%20Flight%20Dynamics2.pdf), [Stanford CFD](http://aero-comlab.stanford.edu/Papers/golf_ball_sports_engineering_2019.pdf), [reverse Magnus](https://link.springer.com/article/10.1007/s12283-020-0318-1)
- Optimal launch & spin — [Trackman PGA averages via Golf.com](https://golf.com/gear/swing-speed-optimal-trackman-numbers-to-hit-your-drives-farther/), [MyGolfSpy](https://mygolfspy.com/news-opinion/instruction/optimal-launch-and-spin-chart-for-drivers-are-you-in-the-right-range/)
- D-plane / spin-axis modelling — [Rapsodo guide](https://rapsodo.com/blogs/golf/understanding-club-path-and-attack-angle-for-your-golf-launch-monitor), [Bushnell parameters](https://www.bushnellgolf.com/launch-pro/what-we-measure)

### Internal references (in this repo)
- `src/openflight/rolling_buffer/processor.py` — OPS243 ball / spin pipeline
- `src/openflight/kld7/radc.py` — RADC interferometric angle pipeline
- `src/openflight/launch_monitor.py` — `Shot` data model, carry regression
- `src/openflight/server.py` — K-LD7 ↔ OPS243 correlation, spin axis from D-plane
- `docs/kld7-ball-detection-theory.md` — prior work on angle extraction
- `docs/rolling_buffer_spin_detection.md` — prior work on spin detection
