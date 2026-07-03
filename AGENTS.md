# Amaltheia Flight Controller Instructions

Before making code changes in this repo, read:

- `CODING_CONTEXT.md`
- `TODO.md`

This is embedded flight-control firmware. Treat motor outputs, arming,
takeoff, landing, failsafe, estimator, and PID changes as safety-critical.
Prefer small, reviewable changes and verify with build or focused inspection
before declaring work complete.

Do not remove safety gates, arming checks, disarm paths, or output clamps
without explicit user approval.

