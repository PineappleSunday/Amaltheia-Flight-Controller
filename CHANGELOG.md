# Changelog

## 2026-08-08

- Added telemetry ownership module for SPI/VCP packet staging, BIT reporting, raw engineer-mode sensor capture, and compact diagnostics.
- Added engineer-mode VCP support with dedicated packet parsing/display paths for raw sensor data and BIT/PBIT detail.
- Added IBIT/PBIT health reporting for startup, mode-select, and runtime status, including sensor presence/readiness and telemetry handshake status.
- Added altitude estimator module that learns BME280 altitude bias from GPS/BME comparison during boot and mode select, then reports corrected BME altitude for redundancy.
- Added explicit sensor-frame contract module and kept the current PCB-to-body transform identity for aligned gyro, accel, and mag axes.
- Moved accelerometer and gyro calibration data into dedicated calibration storage, including LSM303AGR accel bias/scale and gyro bias fields.
- Updated LSM303AGR accel decoding for high-resolution 12-bit samples and exposed relevant configuration/register values for diagnostics.
- Tightened GPS readiness so sensor init/PBIT can distinguish UART startup from actual GPS traffic.
- Updated AHRS comments and interfaces around direct body-frame X/Y/Z sensor input and gyro-bias naming.
- Updated VCP live dashboard for engineer/BIT packets, PBIT labels, CSV mode handling, and reduced Matplotlib 3D redraw load.
