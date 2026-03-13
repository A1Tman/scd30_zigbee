# Changelog

## v1.0.15 - 2026-03-13

- Persisted SCD30 control settings in NVS so temperature offset, ASC, and compensation settings survive reboot.
- Applied Zigbee-driven SCD30 configuration changes safely from the sensor task instead of directly from Zigbee callbacks.
- Documented the tested Home Assistant workflow using `zha_toolkit.attr_write` for the manufacturer-specific control cluster.
- Reduced benign Zigbee log noise by treating successful NLME status indications as handled debug events.

## v1.0.14 - 2026-03-13

- Added a saved-channel fast rejoin path so known devices reconnect to Zigbee much faster after reboot.
- Persisted the last successful Zigbee channel in NVS and reuse it before falling back to broader steering.
- Avoided unconditional network steering on non-factory-new startup signals.

## v1.0.13 - 2026-03-13

- Disabled maintenance-only Zigbee controls by default to reduce exposed attack surface.
- Updated the Home Assistant and ZHA quirk documentation to match the default control cluster.
- Stopped automatic Zigbee network storage wipes during normal boot and reconnect handling.
- Added stale steering recovery so commissioning restarts automatically if Zigbee join gets stuck.

## v1.0.11 - 2026-03-13

- Fixed the I2C startup and probe flow for the SCD30.
- Reduced serial log noise during normal operation.
- Replaced the old blocking SCD30 warm-up delay with non-blocking startup stabilization.
- Updated the project for ESP-IDF v5.5.3 and 16 MB flash targets such as the ESP32-C6-WROOM-1-N16.
