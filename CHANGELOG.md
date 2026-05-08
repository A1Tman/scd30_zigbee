# Changelog

## v1.0.19 - 2026-05-08

- Fixed `clear_zigbee_network_state` to use `nvs_erase_key` instead of writing a blob over the existing `u8` connection flag, which corrupted the key's NVS type and produced misleading logs on the next boot.
- Corrected the startup log path so a missing NVS connection flag no longer reports "Previous connection detected".
- Removed an unreachable `STEERING_MAX_ATTEMPTS` cap in `bdb_start_top_level_commissioning_cb` (the existing `>=8` reset always fires first) and dropped the dead `STEERING_MAX_ATTEMPTS`, `ESP_ZB_NETWORK_INIT_TIMEOUT`, `ED_SCAN_DURATION`, and `MAX_CHILDREN` defines.
- Deleted the no-op `signal_struct->esp_err_status = ESP_FAIL` line in the `CAN_SLEEP` handler whose comment claimed to prevent sleep — sleep is governed by whether `esp_zb_sleep_now()` is called, not by mutating the signal struct.
- Removed `main/zcl_utility.h`; its single declaration had no implementation and nothing referenced its types or macros.
- Consolidated user-input bounds into named constants (`SCD30_FRC_TARGET_PPM_MIN/MAX`, `SCD30_TEMP_OFFSET_MIN_C/MAX_C`) and used them at every Zigbee/API entry point. The driver-internal `scd30_force_recalibration` keeps its 300 ppm diagnostic floor with an inline comment explaining why.

## v1.0.18 - 2026-04-24

- Protected the shared SCD30 runtime config with a spinlock to eliminate a cross-task data race between the Zigbee callback path and the measurement task.
- Added bounds checking in `scd30_send_command` to prevent a latent stack buffer overflow if a future caller passes too many data words.
- Yielded from the BOOT button ISR after waking the event task, restoring the FreeRTOS contract for `xEventGroupSetBitsFromISR` users.
- Switched the persisted Zigbee connection flag in NVS from a 1-byte blob to `nvs_get_u8`/`nvs_set_u8` for type consistency with the channel key (one-time auto-migration on first boot after upgrade).
- Removed a redundant CRC verification loop in `scd30_read_measurement` that re-checked bytes already validated by `scd30_read_data`.

## v1.0.17 - 2026-04-03

- Simplified the BOOT button behavior to two hold actions only: `3s` for Zigbee rejoin and `12s` for outdoor `400 ppm` forced recalibration.
- Removed the old quick-press debug, diagnostics, and factory-reset maintenance menu from the firmware button handler.
- Updated the documentation to match the new maintenance-button behavior and outdoor recalibration workflow.

## v1.0.16 - 2026-04-03

- Mirrored accepted control writes back into the local Zigbee attribute table so cluster `0xFC00` reads stay in sync with the applied SCD30 configuration.
- Updated the Home Assistant quirk example to define readable `0xFC00` attributes without manufacturer-specific flags.

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
