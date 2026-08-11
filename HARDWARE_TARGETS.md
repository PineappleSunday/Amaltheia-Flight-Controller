# Amaltheia Hardware Targets

This repository is moving toward shared flight-control code with separate
hardware-target integration layers.

## Current Target: DiscoveryBoard

The current firmware integration is treated as the DiscoveryBoard hardware
baseline.

Known current sensor/IO mapping:

- LSM303AGR accel/mag: I2C1, PB6 SCL, PB9 SDA
- I3GD20/I3G4250D gyro: SPI1, PA5 SCK, PA6 MISO, PA7 MOSI, PE3 CS
- BME280 barometer: I2C3, PA8 SCL, PB8 SDA
- GT-U7 GPS: USART2, PA2 TX, PA3 RX
- LiDAR: USART1, PA15 TX, PB7 RX
- ESP telemetry link: SPI5 slave, PB0 SCK, PB1 NSS, PE13 MISO, PE14 MOSI
- Battery/voltage sense: ADC1_IN1, PA1
- External oscillator: 12 MHz HSE, configured to preserve 96 MHz SYSCLK and
  48 MHz USB

## New Target: Arbiter Industries Hardware

Branch:

```text
hardware/arbiter-industries
```

Architecture details to capture before implementation:

- MCU part number/package
- External oscillator frequency
- IMU/gyro/accel/mag part numbers
- Barometer/altimeter sensor
- GPS module and UART baud
- LiDAR/range sensor model and interface
- Telemetry companion/radio interface
- Motor output protocol and timer/pin mapping
- Battery voltage/current sensing pins and scale
- USB connector wiring and VBUS sensing choice
- Status LEDs and fault indication pins
- Any new safety-critical features or hardware interlocks

Implementation intent:

- Keep AHRS, PID, mixer, navigation, telemetry formats, calibration math, and
  state-machine behavior shared unless hardware forces a change.
- Put pin maps, clock setup, bus selection, and sensor bring-up behind a
  hardware-target boundary.
- Preserve safety gates, arming checks, disarm paths, and motor clamps while
  moving board-specific code.
