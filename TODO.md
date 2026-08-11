# Amaltheia Firmware TODO

## P0 - Must Fix Before Flight Test

- Consolidate takeoff ownership before mission flight.
  - Current decision: takeoff belongs to the procedural executive state machine in `Core/Src/main.c`.
  - Mission navigation is gated until `g_state.offGround == true`, so TAKEOFF owns `g_target.z`, `g_target.ff_vz`, yaw hold, and PID reset state before handoff.
  - Remaining cleanup: remove or explicitly reserve `WP_ACTION_TAKEOFF` / `Navigation_TakeOff()` until mission-side takeoff is intentionally implemented.
  - Files: `Core/Src/main.c`, `FlightControl/navigation.c`

- Make emergency stop immediate.
  - Status: tabled while soft land / RTH path is being brought up.
  - Current `x`/`X` command still routes through landing/state-machine behavior.
  - Desired behavior: zero motor commands, disarm, bypass navigation/control outputs immediately.
  - File: `Core/Src/main.c`

- Add soft land / return-to-mode-select command.
  - Current first increment: `land` and `rth` command aliases request a soft landing in place, then disarm and return to Mode Select.
  - Remaining RTH work: capture home position, route back horizontally using GPS/local frame, then run the soft landing terminal phase.
  - Files: `Core/Src/main.c`, `FlightControl/navigation.c`

- Add anti-windup behavior for motor saturation.
  - Mixer returns saturation flags; flight logic should use them.
  - Freeze or back-drive integrators when an axis is commanding into saturation.
  - Files: `FlightControl/flight_logic.c`, `FlightControl/mixer.c`

- Add LSM303 DMA/I2C recovery timeout.
  - Track last transfer tick per DMA state.
  - If no completion within about 50 ms, reset the state machine to IDLE.
  - File: `RunnionProjectDrivers/src/lsm303.c`

- Finish magnetometer calibration path.
  - Hard-iron offset compensation.
  - Soft-iron scale/matrix compensation.
  - Apply before AHRS yaw fusion.
  - Files: `FlightControl/AHRS.c`, `RunnionProjectDrivers/src/lsm303.c`

## P1 - Flight Behavior Refactors

- Refactor and validate the landing sequence.
  - Define explicit landing states instead of mixing landing behavior into emergency/navigation branches.
  - Verify descent rate scheduling, flare/ground detection, disarm conditions, and failsafe behavior.
  - Files: `Core/Src/main.c`, `FlightControl/navigation.c`

- Extract and simplify waypoint time-of-arrival behavior.
  - Separate waypoint sequencing from timing/interpolation math.
  - Make zero-duration actions explicit instead of relying on `toa <= wp_start_time` snap behavior.
  - Ensure takeoff, hover, and landing actions can run without being bypassed by ToA shortcuts.
  - File: `FlightControl/navigation.c`

- Implement return-to-home now that GPS is available.
  - Capture home position after valid GPS lock and arming.
  - Add RTH command/state and mission action.
  - Define altitude, route, yaw, landing, and GPS-loss behavior.
  - Files: `Core/Src/main.c`, `FlightControl/navigation.c`, `RunnionProjectDrivers/src/gt_u7.c`

- Implement `WP_ACTION_TAKEOFF`.
  - Current case body in `HandleArrival()` is empty.
  - Mission currently uses `WP_ACTION_MOVE` as a workaround.
  - File: `FlightControl/navigation.c`

## P2 - Timing And Control Quality

- Move main control-loop timing to a hardware timer interrupt.
  - Replace `HAL_GetTick()`-gated loop timing with deterministic timer scheduling.
  - Keep ISR work minimal: set a control-loop flag or run a tightly bounded control step.
  - Use high-resolution `dt` for AHRS and PID math.
  - Files: `Core/Src/main.c`, timer config

- Make hover thrust tunable.
  - `base_thrust = 63.0f + thrust_adj` should become a parameter.
  - File: `FlightControl/flight_logic.c`

- Add horizontal velocity feedforward.
  - `target->ff_vx` and `target->ff_vy` are computed but not used in roll/pitch loops.
  - Feed them into outer-loop rate/angle commands.
  - Files: `FlightControl/flight_logic.c`, `FlightControl/navigation.c`

- Tune mixer thrust linearization per platform.
  - Current expo value is hardcoded.
  - Make configurable per motor/prop/battery setup.
  - File: `FlightControl/mixer.c`

## P3 - Sensor And Estimation Cleanup

- Add magnetic declination correction.
  - AHRS currently uses magnetic north directly as true north.
  - Add compile-time or config-based declination offset.
  - File: `FlightControl/AHRS.c`

- Fix duplicate `static bool mag_trust` declaration.
  - File-scope and function-scope declarations currently shadow each other.
  - File: `FlightControl/AHRS.c`

- Replace blocking I3GD20 SPI calls.
  - Current driver uses `HAL_MAX_DELAY`.
  - Use bounded timeouts so a hung SPI bus cannot stall the flight loop indefinitely.
  - File: `RunnionProjectDrivers/src/i3gd20.c`

- Remove unused Kalman filter code.
  - Superseded by Madgwick AHRS.
  - Files: `RunnionProjectDrivers/src/kalman.c`, `RunnionProjectDrivers/inc/kalman.h`

## Done / Recently Addressed

- Gyro full-scale range updated to 2000 dps.
  - `CTRL_REG4=0xA0`, `dps_per_lsb=0.07`

- SPI5 telemetry command parsing moved away from global interrupt masking.
  - Uses DMA ping-pong buffering and main-loop processing.

- VCP altitude now uses raw LiDAR distance converted to meters.
