# Changelog

## v1.0.13 - 2026-03-13

- Disabled maintenance-only Zigbee controls by default to reduce exposed attack surface.
- Updated the Home Assistant and ZHA quirk documentation to match the default control cluster.

## v1.0.11 - 2026-03-13

- Fixed the I2C startup and probe flow for the SCD30.
- Reduced serial log noise during normal operation.
- Replaced the old blocking SCD30 warm-up delay with non-blocking startup stabilization.
- Updated the project for ESP-IDF v5.5.3 and 16 MB flash targets such as the ESP32-C6-WROOM-1-N16.
