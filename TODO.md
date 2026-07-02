# Amaltheia Firmware TODO

## High — fix before flight testing

- Add magnetometer calibration pipeline in firmware:
  - hard-iron offset compensation (`mx/my/mz` bias subtraction)
  - soft-iron compensation (3x3 matrix or per-axis scale)
  - apply before AHRS yaw fusion path

- LSM303 DMA state machine deadlock: add timeout to I2C transfers
  - track `last_transfer_tick` per state
  - if no completion within ~50ms, reset state machine to IDLE
  - file: `RunnionProjectDrivers/src/lsm303.c`

- PID integral windup during motor saturation
  - mixer already returns per-motor saturation flags (8-bit, lo/hi)
  - when saturation detected, freeze or back-drive integrators on saturating axis
  - files: `FlightControl/flight_logic.c`, `FlightControl/mixer.c`

- Emergency stop (`x`/`X` command) is not immediate
  - currently sets a landing waypoint and continues through state machine
  - should zero motor commands and disarm immediately, bypassing navigation
  - file: `Core/Src/main.c`

## Medium — stability and accuracy

- Horizontal feedforward velocity unused in roll/pitch loops
  - `target->ff_vx` and `target->ff_vy` are computed by navigation but never applied
  - add feedforward into roll/pitch outer loop rate commands
  - files: `FlightControl/flight_logic.c`, `FlightControl/navigation.c`

- Hard-coded hover thrust (63%) in flight logic
  - `base_thrust = 63.0f + thrust_adj` should be a tunable parameter
  - file: `FlightControl/flight_logic.c`

- ~~Gyro full-scale range fixed at 250 dps~~ — updated to 2000 dps (CTRL_REG4=0xA0, dps_per_lsb=0.07)

- `WP_ACTION_TAKEOFF` waypoint action is unimplemented
  - case body in `HandleArrival()` is empty; mission currently uses `WP_ACTION_MOVE` as workaround
  - file: `FlightControl/navigation.c`

## Low — cleanup

- Remove unused Kalman filter code (superseded by Madgwick AHRS)
  - files: `RunnionProjectDrivers/src/kalman.c`, `RunnionProjectDrivers/inc/kalman.h`

- Fix duplicate `static bool mag_trust` declaration in AHRS.c
  - declared at file scope and again inside a function; second shadows first
  - file: `FlightControl/AHRS.c`

- Add magnetic declination correction to AHRS
  - magnetic north used directly as true north; add compile-time declination offset
  - file: `FlightControl/AHRS.c`

- Mixer thrust-linearization expo is hardcoded (0.55)
  - make configurable per platform/motor combination
  - file: `FlightControl/mixer.c`

- I3GD20 SPI calls use `HAL_MAX_DELAY`
  - a hung SPI bus will stall the main loop indefinitely
  - replace with a bounded timeout (e.g. 10ms)
  - file: `RunnionProjectDrivers/src/i3gd20.c`
