# Amaltheia Flight Controller Development Context

Last updated: 2026-07-02

## Project Shape

Amaltheia is an STM32-based quadcopter flight-controller firmware project.
Core firmware lives mainly in `Core/Src/main.c`; flight-specific modules live
under `FlightControl/`; board and sensor drivers live under
`RunnionProjectDrivers/`.

Current active work is on the flight-mode and mission-executive behavior,
especially takeoff, mission navigation, landing, and safety behavior.

## Safety Posture

Treat this as safety-critical embedded control code. Motor/ESC behavior,
arming, emergency stop, takeoff, landing, mission progression, estimator state,
and PID reset/windup changes need extra care.

Interrupt policy:

- Do not use global `__disable_irq()` / `__enable_irq()` in normal runtime,
  initialization, telemetry, or sensor data paths.
- Use specific `HAL_NVIC_DisableIRQ()` / `HAL_NVIC_EnableIRQ()` calls only for
  the interrupt source that owns the shared state being copied or claimed.
- Global IRQ disable is acceptable only for final unrecoverable fault entry
  after control is intentionally abandoned.

Preferred test order:

1. Build firmware.
2. Inspect state-machine and target outputs through telemetry.
3. Test grounded with motor battery disconnected.
4. Only then consider powered motor tests.

## Current State-Machine Notes

Mode IDs are defined in `FlightControl/state.h`:

- `MODE_MANUAL_LEVEL = 1`
- `MODE_MISSION = 2`
- `MODE_THRUST_STAND = 3`
- `MODE_ENGINEER = 4`

`EnterMode(MODE_MISSION)` in `Core/Src/main.c` currently:

- requires GPS lock before accepting mission mode
- calls `Navigation_Init(&g_mission, mission_waypoints, total_wp_count, &g_state)`
- resets `takeoff_state = INIT`
- sets `g_state.offGround = false`
- resets active flight PIDs from current state
- sets `g_drone_status.drone_mode = MODE_MISSION`

Mission arming/takeoff GPS gate:

- `mode 2` is rejected unless `gps_ready && gps_fix.valid`.
- `arm` is rejected while in mission mode if GPS lock has been lost.
- If mission mode is somehow armed on-ground and GPS lock is lost before takeoff,
  the FC disarms and returns to Mode Select instead of spooling/launching.

Current decision: takeoff is owned by the procedural executive state machine in
the main control loop. Mission navigation must not write `g_target` until
`g_state.offGround == true`.

Takeoff sequence:

- `81`: `INIT`
- `82`: `SPOOLUP`
- `83`: `TAKEOFF`
- `84`: `TRANSISTION` spelling in code
- `8`: stabilize/main flight-control handoff

Takeoff threshold is currently:

- `flight_takeoff_height_threshold = 0.5f` meters
- TAKEOFF target is threshold plus `0.2f`, so about `0.7 m`
- transition occurs when `g_state.z > 0.5f`
- transition sets `g_state.offGround = true`

Takeoff thrust control:

- Normal altitude control uses `altitude_base_thrust_raw = 63.0f` plus the
  cascaded Z PI correction.
- During `TAKEOFF`, the executive ramps `altitude_base_thrust_raw` from `70.0f`
  to `85.0f` raw mixer percent over 2 seconds, while the Z controller still
  trims thrust and attitude control remains active.
- Mixer thrust linearization means raw mixer percent is not the same as final
  motor output percent.

Grounded mission test expectation:

1. Send `mode 2`.
2. Send `arm`.
3. Observe `flight_mode` walk through `81 -> 82 -> 83`.
4. With motor battery disconnected, physically lift the frame in Z or spoof
   LiDAR altitude above `0.5 m`.
5. Observe `84`, then `8`, with mission navigation driving `g_target`.

Because motor battery is disconnected, the drone cannot actually climb, so it
will remain in TAKEOFF unless LiDAR/state altitude rises past the threshold.

Soft landing command:

- `land` and `rth` currently request the same first-increment behavior:
  latch current local X/Y/yaw, descend under normal vertical control, disarm
  near the ground, then return to Mode Select.
- This is not full GPS return-to-home yet. Full RTH still needs home capture,
  route planning/control back to home, and GPS-loss behavior.

Telemetry/PBIT handshake:

- During initialization, the FC sends `Arbiter` in the SPI TX frame until the
  HUZZAH responds with `peace`.
- HUZZAH should always scan incoming SPI frames for `Arbiter` because it may
  remain powered while the FC resets or reinitializes.
- FC PBIT treats telemetry alive as the explicit `peace` latch, not merely
  observed SPI clocks.
- GPS module presence is part of PBIT, but GPS fix is not required for generic
  Mode Select entry.

## Mission Navigation Notes

Mission waypoint logic is in `FlightControl/navigation.c`.

`Navigation_GetTarget()`:

- snaps immediately for zero-duration/invalid leg duration
- interpolates normal move legs
- has special handling for landing when `WP_ACTION_LAND` and
  `landing_start_t > 0.0f`
- advances on waypoint tolerance or overshoot

`WP_ACTION_TAKEOFF` exists in `navigation.h`, but takeoff is not meaningfully
implemented in `navigation.c` yet. Mission takeoff is currently handled by the
procedural main-loop takeoff state machine. Avoid adding mission-side takeoff
target writes unless the executive takeoff path is removed or deliberately
refactored.

## Current Priority Risks

See `TODO.md` for the living task list. Highest-risk themes currently include:

- consolidate takeoff ownership between main-loop executive and navigation
- make emergency stop immediate and bypass navigation/control outputs
- add anti-windup behavior from mixer saturation flags
- add LSM303 DMA/I2C recovery timeout
- complete magnetometer calibration path

## Session Recovery

Codex local session files have been found under:

`C:\Users\colin\.codex\sessions\YYYY\MM\DD`

Use those only as recovery history. This file is the durable repo-local context
that future sessions should read first.
