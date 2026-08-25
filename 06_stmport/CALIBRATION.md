# TVC Gimbal Servo Calibration

**Project:** `06_stmport`
**Date measured:** 2026-07-31 (servo geometry), ported to F411 2026-08-24
**Hardware:** BPS.space-style 3D printed TVC mount, 2× SG90 9g servos, STM32F411CEU6 Black Pill
**Power:** 4×AA battery pack (~6V), common ground to flight computer — **bench only**

> **Port note.** Servo geometry was measured on a Nucleo F446RE. Those measurements carry
> over unchanged: the mount, servos, and linkages are the same hardware, and TIM3 is
> configured for the same 1 µs tick on both boards. A given pulse width means the same
> physical deflection. Only the pin assignments and clock tree differ, and those are
> updated below.

---

## Channel map

This table is authoritative.

| Function | STM32 pin | Peripheral | Mount label | Control signal |
|---|---|---|---|---|
| TVC servo, "Y" axis | PB4 | TIM3_CH1 | "Y" | `cmd_y` |
| TVC servo, "X" axis | PB5 | TIM3_CH2 | "X" | `cmd_z` |
| Ejection servo | PB0 | TIM3_CH3 | — | latch knock |
| IMU chip select | PB12 | GPIO out | — | — |
| Barometer chip select | PB6 | GPIO out | — | — |
| Flash chip select | PB10 | GPIO out | — | — |
| SPI clock | PA5 | SPI1_SCK | — | — |
| SPI MISO | PA6 | SPI1_MISO | — | — |
| SPI MOSI | PA7 | SPI1_MOSI | — | — |
| Serial TX | PA2 | USART2_TX | — | to USB-TTL RX |
| Serial RX | PA3 | USART2_RX | — | to USB-TTL TX |
| HSE crystal | PH0 / PH1 | 25 MHz | — | — |

**Do not use Arduino header labels on this board.** The F446RE Nucleo carried an Arduino
header where D5 = PB4, D4 = PB5, and A3 = PB0. The Black Pill has no such header, and its
silkscreen reads STM32 port names directly. Carrying the old labels over is actively
dangerous here: "A3" on a Black Pill is PA3, which is the UART receive line to the USB-TTL
adapter, not the ejection servo.

**Changed from F446RE:** IMU chip select moved PC7 → PB12. The F411 in UFQFPN48 has no
port C pins broken out. All other assignments are identical.

## Board mounting configuration

Mount the flight computer such that the **+Y arrow on the IMU points in-plane with the X
axis servo**, oriented from the servo toward the rocket motor. This ensures the Y axis
servo controls `err_y` via `cmd_y`.

---

## Clock tree

| Parameter | Value |
|---|---|
| HSE crystal | 25 MHz |
| PLLM / PLLN / PLLP | 12 / 96 / 2 |
| SYSCLK / HCLK | 100 MHz |
| APB1 peripheral | 50 MHz |
| APB1 timer clock | 100 MHz |
| APB2 | 100 MHz |
| Flash latency | 3 wait states |

**DWT cycle-to-microsecond divisor: 100** (was 84 on the F446RE). Any code converting
`DWT->CYCCNT` deltas to time must use this value.

## Timer configuration

**TIM3 — servo PWM.** APB1 timer clock at 100 MHz.

| Parameter | Value | Result |
|---|---|---|
| Prescaler | 99 | 1 MHz tick = 1 µs per count |
| Counter period (ARR) | 19999 | 20 ms frame = 50 Hz |

Compare register value = pulse width in microseconds. Prescaler was 83 on the F446RE at
84 MHz; the change to 99 preserves the 1 µs tick, which is what makes every calibration
value below portable.

**TIM2 — flight loop tick.** PSC 9999 / ARR 49 → 200 Hz.

**SPI1.** Prescaler 64 → 1.5625 Mbit/s, Mode 3 (CPOL high, CPHA 2nd edge).

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

## CH1 — mount "Y" axis (PB4, TIM3_CH1)

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

## CH2 — mount "X" axis (PB5, TIM3_CH2)

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

## Ejection servo (PB0, TIM3_CH3)

Separate mechanism from the gimbal: the servo horn knocks a spring-loaded latch to release
the recovery system. Values tuned against the physical latch, not derived.

```c
#define EJECT_ARMED_US       2000   // latch engaged, horn clear
#define EJECT_KNOCK_US        600   // tuned against the latch
#define EJECT_DWELL_PASSES    100   // 500 ms at 200 Hz
#define EJECT_BACKUP_PASSES  1500   // 7.5 s after BOOST -- set from OpenRocket
```

**The armed position must be loaded into the compare register before the channel is
enabled** (see `main.c`), so the first pulse out of PB0 holds the latch. A default compare
value would otherwise appear as a deploy event on the pad.

`EJECT_BACKUP_PASSES` is an OpenRocket estimate and should be re-derived once the vehicle
is assembled at flight mass.

---

## Derived constants

```c
// ---- Servo calibration (measured 2026-07-31, 6.75" lever arm) ----
// Channel map:
//   TIM3_CH1 = PB4 = mount "Y" axis servo
//   TIM3_CH2 = PB5 = mount "X" axis servo
#define SERVO_MY_TRIM   1575    // µs at gimbal neutral
#define SERVO_MY_USPD   44.4f   // µs per gimbal degree
#define SERVO_MY_MIN    1200    // µs hard bound, inside mechanical stop at 1150
#define SERVO_MY_MAX    1900    // µs hard bound, inside mechanical stop at 1950

#define SERVO_MX_TRIM   1825
#define SERVO_MX_USPD   48.5f
#define SERVO_MX_MIN    1475    // inside stop at 1425
#define SERVO_MX_MAX    2300    // inside stop at 2350

#define MAX_DEFLECT     6.0f    // gimbal degrees, applies to both axes

#define SERVO_MY_SIGN   (+1.0f) // UNVERIFIED -- see open items
#define SERVO_MX_SIGN   (+1.0f) // UNVERIFIED -- see open items
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

## Loop timing budget

Measured on the F411 at 100 MHz, DWT cycle counter sampled across the full loop body
including the flash page program.

| Condition | Worst case |
|---|---|
| Full loop with flash write | **3232 µs** |
| Budget (200 Hz tick) | 5000 µs |
| Margin | ~1770 µs |

The dominant costs are the W25Q128 page program (every third pass, `Flash_WaitBusy` spins
until the WIP bit clears) and the periodic state `printf` with float formatting. If margin
ever needs recovering, gating the state print on `flight_state <= FLIGHT_PAD` is the
cheapest available saving.

---

## Open items

- [ ] **Actuation sign check** — does positive `cmd` produce a deflection that *corrects*
      the error, or worsen it? `SERVO_MY_SIGN` and `SERVO_MX_SIGN` are both `+1.0f` and
      **unverified**. Must be confirmed by hand in flight configuration before flight.
      This is the single most dangerous remaining unknown in the control loop.
- [ ] Servo behavior under thrust load (bench measurements are unloaded)
- [ ] Slew rate limiting — SG90 transit time under load not yet characterized
- [ ] Flight battery (LiPo + BEC); the 4×AA pack sags under two-servo load and is
      bench-only
- [ ] Control gains `Kp_att` / `Kd_att` — currently placeholders (30, 3), pending plant
      model from OpenRocket or a pivot test rig
- [ ] `EJECT_BACKUP_PASSES` re-derived at as-built flight mass

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
- If TIM3's prescaler is ever changed, every pulse width in this document becomes invalid.
  The 1 µs tick is load-bearing.
