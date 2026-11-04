# AUDIT & CHANGE BACKLOG — modbus_master_mqtt

> Living document: findings plus the concrete changes to execute.
> **Status:** audit complete, no code changes applied yet.
> **Date:** 2026-11-03
> **Scope:** `firmware/modbus_master_mqtt` only.
> **Baseline:** `firmware.bin` = 952,240 bytes (default partition, fits).
>
> This file describes *what to change*. `AGENTS.md` describes *what is*.
> When an item is implemented, tick it and add the commit/PR reference. Move the
> rationale to the "Done" section instead of deleting it.

---

## 1. How to use this file

- Pick items top-down by priority; do not mix unrelated fixes in one change.
- Every item has **evidence** (file:line), a **proposed change**, and
  **acceptance criteria**.
- Re-verify the evidence before starting: line numbers drift.
- After a change, update `AGENTS.md` if an invariant, task, or config changed.
- Keep the "Correct as-is" section in mind: several things that look odd are
  intentional for Sparkplug compliance.

### Priority legend

| Tag | Meaning |
|---|---|
| **P0** | Can stop field operation or cause silent data loss |
| **P1** | Reliability / maintainability risk; fix in the next iteration |
| **P2** | Quality, reproducibility, documentation, tooling |

---

## 2. Change backlog

### P0 — Correctness / field reliability

- [ ] **P0-01 — Reduce Modbus timeout and bound the poll cycle.**
  - Evidence: `src/ModbusAPI.cpp:67` (`syncRequest` blocks), `include/ModbusConfig.h:24`
    (2000 ms), `include/DevicesConfig.h:129` (2000 ms), `src/main.cpp:100-130`.
  - Problem: 4 blocks x 2 s = up to ~8 s per offline device; more devices/blocks
    push the cycle past the 30 s interval and fill `queueMqtt`.
  - Change: lower per-device timeout to 300–500 ms (measure on real bus), or move
    to eModbus async callbacks + result queue. Document the worst-case cycle.
  - Accept: worst-case scan for all offline devices is measured and < poll
    interval; no `queueMqtt` overflows in a 10-minute offline test.

- [ ] **P0-02 — Make NTP a documented, configurable dependency.**
  - Evidence: `src/SparkplugB.cpp:261-268`, `src/SparkplugB.cpp:362-364`,
    `src/main.cpp:49-54` (NTP servers), `src/SparkplugB.cpp:342` (`configTime`).
  - Problem: no NBIRTH (and therefore no data) until `time() > 1700000000`; an
    isolated LAN with no internet leaves the node silent.
  - Change: allow configuring a local NTP server in `netconfig.h` or document
    the requirement and the failure mode. Optionally publish a diagnostic after
    a timeout without time.
  - Accept: README/AGENTS note the requirement; test case on LAN without internet
    is described (expected behavior, not a hang).

- [ ] **P0-03 — Add a task watchdog and recovery policy.**
  - Evidence: no `esp_task_wdt` / `configASSERT` anywhere in `src/` or `include/`.
  - Problem: a stuck task (NVS, MQTT, Modbus) leaves the node mute forever.
  - Change: initialize TWDT, subscribe `mainPollingTask` and `tareaRed`, feed in
    their loops; define panic/reboot behavior. Log stack high-water marks in
    debug builds.
  - Accept: unplugging the broker/bus does not trip the watchdog; a simulated
    hang reboots within the configured timeout.

- [ ] **P0-04 — Stop writing NVS on every connect attempt.**
  - Evidence: `src/SparkplugB.cpp:374-383` (`prepareDeath` writes `bdSeq`),
    `src/main.cpp:339`.
  - Problem: with 30 s backoff this is ~2.9 k flash writes/day; unnecessary wear.
  - Change: increment `bdSeq` in RAM on each CONNECT and persist only when
    NBIRTH is published successfully, or rate-limit persistence.
  - Accept: only one NVS write per successful session; `bdSeq` still increments
    per CONNECT and matches NDEATH/NBIRTH.

- [ ] **P0-05 — Replace cross-core `volatile` flags with atomics.**
  - Evidence: `src/main.cpp:37-45` (flags), `src/main.cpp:189-232` (callbacks).
  - Problem: `volatile` gives no atomicity or ordering guarantee between cores.
  - Change: use `std::atomic<bool>`/`std::atomic<uint8_t>` for `g_mqttReady`,
    `g_mqttConnecting`, `g_needBirth`, `g_birthDone`, `g_ncmdSubscribed`,
    `g_wifiReady`, `g_wifiReason`.
  - Accept: same behavior; no `volatile` left for cross-core state.

- [ ] **P0-06 — Guarantee `cycleEnd` is not dropped.**
  - Evidence: `src/main.cpp:161-163` (drop on full queue), `src/main.cpp:383`
    (`g_lastCycleOk` updated only on `cycleEnd`), `src/SparkplugB.cpp:476-478`.
  - Problem: a dropped final frame suppresses NDATA and leaves `g_lastCycleOk`
    stale.
  - Change: explicit queue policy (e.g., keep the latest `cycleEnd` frame,
    overwrite oldest, or a dedicated single-slot for node status).
  - Accept: forced queue-full test still publishes exactly one NDATA per cycle.

### P1 — Reliability / maintainability

- [ ] **P1-01 — Fix serial monitor baud mismatch.**
  - Evidence: `src/main.cpp:417` (`Serial.begin(115200)`), `platformio.ini:5`
    (`monitor_speed = 921600`).
  - Change: align both (prefer `115200` unless boot logs need faster).
  - Accept: monitor shows readable text at the configured speed.

- [ ] **P1-02 — Remove or implement the dead poll-notification path.**
  - Evidence: `src/main.cpp:33` (`g_pollTaskHandle`), `src/main.cpp:167`
    (`ulTaskNotifyTake`), `src/main.cpp:451` (handle stored); no
    `xTaskNotifyGive` in the project.
  - Change: either delete the handle/notify and use `vTaskDelay`, or expose
    `requestPollNow()` that calls `xTaskNotifyGive`.
  - Accept: no dead code; behavior unchanged if removed.

- [ ] **P1-03 — Make birth retry per-device, not all-or-nothing.**
  - Evidence: `src/SparkplugB.cpp:439-445` (returns false if any DBIRTH fails),
    `src/main.cpp:272-280` (retries the whole birth).
  - Problem: a single failed DBIRTH forces a new NBIRTH with the same `bdSeq`,
    which is not a clean retry.
  - Change: track per-device birth state; retry only missing DBIRTHs; only
    re-send NBIRTH after a real reconnect.
  - Accept: inject one publish failure and observe only that DBIRTH retried.

- [ ] **P1-04 — Verify RS485 direction control (DE/RE) and GPIO12.**
  - Evidence: `src/ModbusAPI.cpp:38-46` (`de_re_pin = -1`), `src/main.cpp:426`,
    `include/ModbusConfig.h:22-23` (TX on GPIO12).
  - Problem: if the transceiver is not auto-direction, no field communication;
    GPIO12 (MTDI strapping) can affect boot.
  - Change: confirm the schematic (`hardware/modbus_master_MQTT/`), pass the real
    DE/RE pin if needed, and document the pinout.
  - Accept: cold-boot test + 24 h bus test with the documented pins.

- [ ] **P1-05 — Determine QoS policy for birth messages.**
  - Evidence: `src/SparkplugB.cpp:279` (DBIRTH), `:433` (NBIRTH), `:300`
    (NDATA), `:471` (DDATA); LWT already QoS 1 (`src/main.cpp:340`).
  - Change: decide QoS 1 for NBIRTH/DBIRTH (recommended for reliability) and
    document the choice; keep DATA at QoS 0 if bandwidth matters.
  - Accept: decoder shows expected QoS for each message type.

- [ ] **P1-06 — Security posture for MQTT.**
  - Evidence: `src/MqttTransport.cpp:139-149` (plain TCP when `tls == nullptr`),
    `include/netconfig.h:12-15` (anonymous, no TLS).
  - Change: document that TLS is required outside a trusted LAN; add a
    `netconfig.example.h` with placeholders. Do not commit real credentials.
  - Accept: `netconfig.h` stays gitignored; example file has no secrets.

- [ ] **P1-07 — Initialize node health as unknown.**
  - Evidence: `src/main.cpp:60` (`g_lastCycleOk = true`), used at
    `src/main.cpp:270`.
  - Problem: the first NBIRTH may report `Modbus_ScanOK = true` before any scan.
  - Change: start `false` or add an explicit "unknown" state.
  - Accept: NBIRTH before the first cycle reports a non-successful scan.

- [ ] **P1-08 — Check task creation results.**
  - Evidence: `src/main.cpp:451-452`.
  - Change: assert/log if `xTaskCreatePinnedToCore` fails; define fallback.
  - Accept: failure path is logged and does not boot half-configured.

- [ ] **P1-09 — Pin toolchain and libraries for reproducibility.**
  - Evidence: `platformio.ini:2` (`platform = espressif32`, no version),
    `platformio.ini:7` (eModbus from `master`).
  - Change: pin the platform version and an eModbus tag/commit; add
    `build_flags` for `-Wall -Wextra` and `LOG_LEVEL`; add `monitor_filters`.
  - Accept: two clean builds on different days produce the same dependency set.

- [ ] **P1-10 — Remove unused `SPI.begin()`.**
  - Evidence: `src/main.cpp:424`; the LoRa radio is not used.
  - Change: delete it (or justify if it is needed for the board).
  - Accept: build + boot + Modbus unaffected.

- [ ] **P1-11 — De-duplicate WiFi reconnection strategy.**
  - Evidence: `src/main.cpp:441` (`setAutoReconnect(true)`) and
    `src/main.cpp:316-319` (manual `begin()` every 10 s).
  - Change: pick one strategy; if manual, disable auto-reconnect and keep the
    backoff.
  - Accept: reconnect behavior is predictable and logged once.

- [ ] **P1-12 — Verify `exception_code` mapping.**
  - Evidence: `src/ModbusAPI.cpp:31`
    (`exception_code = static_cast<uint8_t>(e)`).
  - Change: check the eModbus `Error` enum values against the Modbus exception
    codes you expect (e.g., 0x01 vs 0x81); map explicitly if needed.
  - Accept: a forced illegal-address response logs the expected exception code.

### P2 — Quality, tooling, documentation

- [ ] **P2-01 — Add host-side tests for decode helpers.**
  - Evidence: `include/ModbusConfig.h:32-50`; no tests in the repo.
  - Change: PlatformIO `[env:native]` + Unity tests for `mb_u16`, `mb_i32`,
    `mb_byte` (bounds, endianness, sign).
  - Accept: `pio test -e native` runs in CI or locally.

- [ ] **P2-02 — Add a Sparkplug conformance check to the workflow.**
  - Evidence: `tools/sparkplug/sparkplug_decode.py` already validates seq,
    timestamps, alias/name rules.
  - Change: document the exact command in `AGENTS.md`; optionally add a
    make/ps1 target that starts the decoder against the broker.
  - Accept: a birth/data cycle produces no `[WARN]` output.

- [ ] **P2-03 — Fix documentation drift.**
  - Evidence: `TUTORIAL_MODBUS.md:3-9` (still describes `kBlocks`, JSON,
    `tareaMqtt`), `include/SensorRegistry.h:10-11` and `:47-49` (LoRa payload /
    Activate Byte comments).
  - Change: update the tutorial to the current `DevicesConfig` + Sparkplug flow;
    clarify that `SensorRegistry` IDs are only logical IDs in this firmware.
  - Accept: docs match code names; no stale JSON references.

- [ ] **P2-04 — Guard node-metric builders against overflow.**
  - Evidence: `src/SparkplugB.cpp:158-218` (`addBdSeqMetric`,
    `addRebirthMetric`, `addNodeHealthMetrics` do not check
    `metrics_count < SPB_MAX_METRICS`).
  - Change: add bounds checks or `static_assert` on the worst case.
  - Accept: no behavioral change; overflow impossible by construction.

- [ ] **P2-05 — Consolidate duplicated metric setters.**
  - Evidence: `src/SparkplugB.cpp:103-129` (Int32/UInt32 and Int64/UInt64 are
    identical).
  - Change: keep distinct names but document that the protobuf field is unsigned
    and the datatype carries the sign, or merge into one helper.
  - Accept: same wire output.

- [ ] **P2-06 — Remove unused/undocumented knobs.**
  - Evidence: `src/main.cpp:32` (`g_pollIntervalMs` is never written).
  - Change: delete or expose a documented API to change the interval.
  - Accept: no unused global.

- [ ] **P2-07 — Document/measure stack usage.**
  - Evidence: stacks 8192/12288 (`src/main.cpp:451-452`).
  - Change: log `uxTaskGetStackHighWaterMark` in a debug build; record values
    here.
  - Accept: measured headroom documented; adjust stacks if needed.

- [ ] **P2-08 — Serialize logging across cores.**
  - Evidence: `include/Log.h:17-24` writes to `Serial` from both cores.
  - Change: optional mutex around `LOG_PRINT` or a small log queue.
  - Accept: no interleaved lines under load.

- [ ] **P2-09 — Add `netconfig.example.h`.**
  - Evidence: `include/netconfig.h` is gitignored (`.gitignore:25` at repo root).
  - Change: commit an example with placeholders and document the setup step.
  - Accept: a fresh clone knows what to create; no secrets committed.

- [ ] **P2-10 — Document NDEATH timestamp edge case.**
  - Evidence: `src/SparkplugB.cpp:385-390` (`prepareDeath` may build NDEATH with
    `has_timestamp = false` if NTP is not ready).
  - Change: decide whether to delay CONNECT until `timeReady()` or document the
    non-compliant window.
  - Accept: documented behavior for the first connect before NTP.

---

## 3. Correct as-is — do not "fix" without a spec reason

- Alias uniqueness across the edge node; device aliases start at 10 and node
  aliases at 1. `src/SparkplugB.cpp:344-356`.
- DATA messages send alias + timestamp + value only (no name/datatype).
  `src/SparkplugB.cpp:469`.
- BIRTH messages send name + alias + datatype + timestamp + value.
  `src/SparkplugB.cpp:243-248`.
- NDEATH has no `seq`; DDEATH has no metrics. `src/SparkplugB.cpp:390`,
  `:499`.
- `seq` is shared by node and devices and resets to 0 on NBIRTH.
- Clean session enabled (`disable_clean_session = 0`), NCMD re-subscribed before
  birth. `src/MqttTransport.cpp:131`, `src/main.cpp:250-258`.
- LWT is QoS 1. `src/main.cpp:340`.
- `SPB_MAX_METRICS = 24` matches `sparkplug_b.options`.
- Static asserts on device count and alias base.

---

## 4. Verification plan (after changes)

1. `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run` — build clean,
   fix new warnings.
2. Monitor at the corrected baud rate.
3. Broker + `python tools/sparkplug/sparkplug_decode.py --host <broker>` —
   expect NBIRTH, DBIRTH, DDATA, NDATA with no `[WARN]`.
4. Force failures: unplug RS485 (DDEATH after 3 cycles), stop broker
   (reconnect/backoff), send NCMD rebirth.
5. Offline-bus test for 10 minutes: no queue overflow, no watchdog reset.

---

## 5. Decisions pending

- [ ] Timeout value for Modbus (300 / 500 / 1000 ms) based on real bus timing.
- [ ] Async eModbus vs. shorter synchronous timeout.
- [ ] QoS 1 for NBIRTH/DBIRTH or keep QoS 0.
- [ ] Local NTP server address, or require internet.
- [ ] Whether OTA is in scope (needs a dual-app partition table).

---

## 6. Done

_(empty — add implemented items here with date and commit reference)_
