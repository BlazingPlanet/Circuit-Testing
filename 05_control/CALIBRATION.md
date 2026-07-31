# TVC Gimbal Servo Calibration

**Project:** `05_control`
**Date measured:** 2026-07-31
**Hardware:** BPS.space-style 3D printed TVC mount, 2× SG90 9g servos, STM32 Nucleo F446RE
**Power:** 4×AA battery pack (~6V), common ground to Nucleo

---

## Channel map

Five naming systems are in play. This table is authoritative.

| Timer channel | STM32 pin | Arduino label | Mount label | IMU axis |
|---|---|---|---|---|
| TIM3_CH1 | PB4 | D5 | "Y" | TBD |
| TIM3_CH2 | PB5 | D4 | "X" | TBD |

**Note:** PB4 = D5 and PB5 = D4. The Arduino header numbering does *not* match the
STM32 port numbering. This was confirmed by observation, not assumed.

**IMU axis correspondence is not yet determined.** It depends on the flight computer's
mounting orientation in the airframe, which is not yet fixed. Constants are named after
the *mount* axes (MX, MY) deliberately until that is settled.

---

## Timer configuration

TIM3, APB1 at 84 MHz.

| Parameter | Value | Result |
|---|---|---|
| Prescaler | 83 | 1 MHz tick = 1 µs per count |
| Counter period (ARR) | 19999 | 20 ms frame = 50 Hz |

Compare register value = pulse width in microseconds.

---

## Measurement method

A rigid rod was taped to the motor tube, parallel to the thrust axis, extending
**6.75 inches** from the gimbal's axis of rotation. The gimbal was laid flat and the
rod tip's displacement measured against a fixed ruler at each commanded pulse width.

Angle conversion:

```
theta_deg = 57.3 * d / L  =  8.49 * d      (L = 6.75 in, d in inches)
```

### Measurement caveat — first attempt discarded

An initial round of measurements used a 5.75" nail file as the lever. It **fouled on the
mount** at large deflections, which produced two false findings on CH1:

- An apparent 2× slope asymmetry (2.49 vs 1.25 deg per 100 µs)
- An apparent progressive "bind" near 1150 µs, with the servo audibly straining

Both vanished once the rod was remounted clear of obstructions. **Always sweep the full
range by hand, power off, and confirm the lever clears everything before measuring.**

---

## CH1 — mount "Y" axis (PB4 / D5)

**Trim (gimbal neutral): 1575 µs**

| Command | Δ from trim | d (in) | Angle | deg / 100 µs |
|---|---|---|---|---|
| 1675 | +100 | 0.281 | 2.39° | 2.39 |
| 1775 | +200 | 0.547 | 4.64° | 2.32 |
| 1475 | −100 | 0.250 | 2.12° | 2.12 |
| 1375 | −200 | 0.500 | 4.24° | 2.12 |
| **1950** | **+375** | 0.938 | **7.96°** | *(mechanical stop)* |
| **1150** | **−425** | 1.063 | **9.02°** | *(mechanical stop)* |

**Slope: 0.0239 deg/µs positive, 0.0212 deg/µs negative.** Average ≈ **0.0225 deg/µs**
(44.4 µs per gimbal degree).

Linear to within measurement resolution across ±200 µs. The ~12% direction asymmetry is
real but modest — consistent with pushrod geometry.

At the positive stop, measured deflection is ~11% below linear extrapolation. The negative
stop matches the linear model almost exactly.

---

## CH2 — mount "X" axis (PB5 / D4)

**Trim (gimbal neutral): 1825 µs**

| Command | Δ from trim | d (in) | Angle | deg / 100 µs |
|---|---|---|---|---|
| 1925 | +100 | 0.188 | 1.59° | 1.59 |
| 2025 | +200 | 0.438 | 3.72° | 1.86 |
| 1725 | −100 | 0.250 | 2.12° | 2.12 |
| 1625 | −200 | 0.484 | 4.11° | 2.06 |
| **2350** | **+525** | 0.875 | **7.42°** | *(mechanical stop)* |
| **1425** | **−400** | 1.031 | **8.75°** | *(mechanical stop)* |

**Slope: ~0.0186 deg/µs positive, ~0.0206 deg/µs negative.** Average ≈ **0.0206 deg/µs**
(48.5 µs per gimbal degree).

The negative side is cleanly linear at 2.12 deg/100 µs across three readings. The positive
side is shallower in the first 100 µs above trim (1.59) then steepens (2.12) — the stepwise
and direct measurements agree, so this appears to be genuine geometry rather than noise.
It may indicate trim sits slightly off the linkage's symmetric point.

At the positive stop, measured deflection is well below linear extrapolation — significant
falloff at large positive deflection.

**Trim was re-verified with the corrected 6.75" lever** and remained at 1825. Mechanical
stops were also unchanged from the first attempt. CH2's original measurements did not
appear to be corrupted by the lever fouling, but these values supersede them (better
resolution).

---

## Derived constants

```c
// ---- Servo calibration (measured 2026-07-31, 6.75" lever arm) ----
#define SERVO_MY_TRIM   1575    // µs at gimbal neutral
#define SERVO_MY_USPD   44.4f   // µs per gimbal degree
#define SERVO_MY_MIN    1200    // µs hard bound, inside mechanical stop at 1150
#define SERVO_MY_MAX    1900    // µs hard bound, inside mechanical stop at 1950

#define SERVO_MX_TRIM   1825
#define SERVO_MX_USPD   48.5f
#define SERVO_MX_MIN    1475    // inside stop at 1425
#define SERVO_MX_MAX    2300    // inside stop at 2350

#define MAX_DEFLECT     6.0f    // gimbal degrees, applies to both axes
```

**The two guards are independent by design.** `MAX_DEFLECT` prevents the *controller* from
commanding excessive deflection. The MIN/MAX pulse bounds prevent a *software* error — bad
trim, wrong slope, arithmetic bug — from driving a servo into a mechanical stop. Either
alone would be insufficient.

---

## Deflection limit rationale

`MAX_DEFLECT = 6.0°` was chosen for margin, not derived from vehicle dynamics.

Margin to the mechanical stops at 6°:

| Channel | Direction | Command (µs) | Stop (µs) | Margin |
|---|---|---|---|---|
| CH1 | + | 1842 | 1950 | 108 µs |
| CH1 | − | 1308 | 1150 | 158 µs |
| CH2 | + | 2116 | 2350 | 234 µs |
| CH2 | − | 1534 | 1425 | **109 µs** |

All four clear. The tightest margin is roughly 2° of gimbal.

**Why not larger:** both channels show slope falloff beyond roughly ±300 µs, so a single
linear constant misrepresents the plant near full deflection. Going to ±7° would have left
only ~26 µs of margin on CH2's negative side — too thin against trim drift or measurement
error.

**Why not smaller:** ±5° was the original estimate borrowed from BPS.space and was never
verified against this hardware. The measured stops are near ±8°, so 6° captures useful
additional authority while staying inside the verified-linear region.

---

## Open items

- [ ] IMU axis ↔ mount axis correspondence (depends on flight computer mounting orientation)
- [ ] **Actuation sign check** — does positive `cmd` produce a deflection that *corrects*
      the error, or worsen it? Must be verified by hand before flight. This is the single
      most dangerous remaining unknown in the control loop.
- [ ] Servo behavior under thrust load (bench measurements are unloaded)
- [ ] Slew rate limiting — SG90 transit time under load not yet characterized
- [ ] Flight battery (LiPo + BEC); the 4×AA pack sags under two-servo load and is
      bench-only
- [ ] Control gains `Kp_att` / `Kd_att` — currently placeholders (30, 3), pending plant
      model from OpenRocket or a pivot test rig

---

## Notes for future re-calibration

- Re-measure if the gimbal is disassembled, a servo horn is reseated, or a pushrod is
  adjusted.
- Sweep the full range by hand with the lever attached, power off, before commanding
  anything.
- Use the longest practical lever. At 6.75", one 1/32" ruler tick ≈ 0.27°. At 12" it would
  be ≈ 0.15°.
- Approach each measurement point from both directions to check backlash. On this build,
  backlash was below measurement resolution on both channels.
- Never leave a servo commanded against a mechanical stop. SG90 stall current is ~700 mA
  and the servo heats quickly.
