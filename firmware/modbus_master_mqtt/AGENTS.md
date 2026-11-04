# AGENTS.md — modbus_master_mqtt

> Context file for AI coding agents (opencode, Cursor, Claude Code, etc.).
> Read this before editing. Keep it short; prefer references to code over copies.
>
> **Last verified:** 2026-11-04
> **Scope:** this folder only (`firmware/modbus_master_mqtt`). Other firmware
> variants in the repo are out of scope.

---

## 1. What this project is

ESP32 firmware that reads a three-phase power meter over **Modbus RTU** and
publishes measurements as **Sparkplug B** messages over **MQTT** (protobuf via
**nanopb**).

```
Modbus RTU (RS485, Serial2)  ->  MeasureFrame  ->  queue  ->  Sparkplug B  ->  MQTT broker
```

- Board: `ttgo-lora32-v21` (ESP32, 4 MB flash). The LoRa radio is **not used**.
- Framework: Arduino on ESP-IDF (`framework = arduino`).
- Modbus library: eModbus (`ModbusClientRTU`), pinned to tag `v1.7.4.stable`.
- Protobuf: nanopb 0.4.92. Generated files are committed; do not edit them.
- Toolchain: `platform = espressif32@7.1.3` (pinned).
- Current build size: ~942 KB (`firmware.bin` = 942,189 bytes), fits the default partition.

Source of truth for configuration is the `include/*Config.h` headers, not this
file. If a value here disagrees with code, the code wins — and this file is stale.

---

## 2. Read / ignore

**Read first (small, high-signal):**

1. `platformio.ini` — target, libraries, monitor speed.
2. `include/DevicesConfig.h` — devices, Modbus blocks/signals, Sparkplug metrics.
3. `include/ModbusConfig.h` — RS485 bus params, decode helpers, poll interval.
4. `include/SparkplugConfig.h` — datatypes, aliases, limits.
5. `src/main.cpp` — tasks, state machine, queue, publish flow.
6. `src/SparkplugB.cpp` — topics, bdSeq, birth/death/data encode/decode.

**Ignore unless the task requires it:**

- `.pio/`, `.venv/`, `__pycache__/`, `*.pyc`
- `tools/sparkplug/python/sparkplug_b_pb2.py` (generated)
- `docs/` at repo root (large Doxygen dump)
- `archive/` at repo root (legacy versions)
- `PRESENTACION_SPARKPLUG.md` was removed; do not look for it
- `src/sparkplug_b.pb.c` and `include/sparkplug_b.pb.h` (nanopb-generated)

---

## 3. Build / flash / monitor

`pio` is **not on PATH** on this machine. Use the full path:

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"

& $pio run                 # build
& $pio run -t upload       # flash
& $pio device monitor      # serial monitor
& $pio run -t clean        # clean
```

`Serial.begin(115200)` and `monitor_speed = 115200` are aligned. The build uses
`-Wall -Wextra` on project sources (`build_src_flags`) and
`monitor_filters = esp32_exception_decoder, time`.

There is no test suite and no CI. See `AUDIT.md` for proposed verification work.

---

## 4. Architecture and task ownership

### Tasks

| Task | Core | Priority | Stack | Responsibility |
|---|---:|---:|---:|---|
| `mainPollingTask` (`"MainPoll"`) | 0 | 3 | 8192 | Modbus polling, decoding, `MeasureFrame` production |
| `tareaRed` (`"NetTask"`) | 1 | 5 | 12288 | WiFi/MQTT lifecycle, Sparkplug birth/data, queue consumer |
| Arduino `loopTask` | 1 | 1 | default | Only `vTaskDelay(1000)`; does nothing |
| esp-mqtt event task | IDF | IDF | 8192 | Runs MQTT callbacks (`onConnect`, `onMessage`, ...) |

### Resource ownership (do not violate)

| Resource | Owner | Notes |
|---|---|---|
| Modbus bus / `ModbusClientRTU` | `mainPollingTask` only | `syncRequest` is blocking; no mutex because only this task reads |
| MQTT client (`publish`, `subscribe`, `connect`, `setWill`) | `tareaRed` only | Callbacks run in the esp-mqtt task |
| Sparkplug encode state (`g_payload`, `g_txBuf`, `g_seq`) | `tareaRed` only | Never encode from another task |
| NCMD decode (`g_cmdPayload`) | esp-mqtt task | Only `handleCommand` |
| `bdSeq` + NVS | `tareaRed` (`prepareDeath` + persist after NBIRTH) | One NVS write per successful session |
| WiFi/MQTT state flags | event/callback tasks write, `tareaRed` reads | `std::atomic` |
| `queueMqtt` | producer `MainPoll`, consumer `NetTask` | 16 frames (`2 * kMaxDevices`); on overflow, normal frames are dropped and `cycleEnd` evicts the oldest |
| Actuators | none | Telemetry only; no control loop |

### Data flow

1. `setup()` initializes Serial, Modbus UART, queue, Sparkplug topics/NTP, WiFi,
   MQTT transport, then creates both tasks.
2. `mainPollingTask` iterates devices -> blocks -> `modbus_api_read_registers()`.
   Each `ModbusSignal::decode()` writes into a per-device `SensorAcc`.
3. It builds one `MeasureFrame` per device, marks the last one `cycleEnd`, and
   sends it to `queueMqtt` without blocking. If full, normal frames are dropped;
   a `cycleEnd` frame evicts the oldest frame.
4. `tareaRed` drives WiFi/MQTT reconnection with backoff, subscribes NCMD, waits
   for NTP, publishes NBIRTH + DBIRTH, then consumes frames.
5. Per frame: DDEATH after `SPB_DEVICE_DEATH_CYCLES`, DBIRTH on recovery, DDATA
   when online, NDATA on `cycleEnd`.
6. NCMD `Node Control/Rebirth` is decoded in the MQTT task, debounced, and sets
   an atomic flag; `tareaRed` republishes birth.

```
mainPollingTask (core 0)                       tareaRed (core 1)
  for each device                                WiFi / MQTT state machine
    for each block: syncRequest()                subscribe NCMD
    decode signals -> SensorAcc                  wait NTP -> NBIRTH + DBIRTH
    build MeasureFrame --xQueueSend--> [queue] --xQueueReceive--> DDEATH/DBIRTH
                                                                 DDATA + NDATA
```

---

## 5. Modbus model

Configuration lives in `include/DevicesConfig.h`:

- `ModbusBlock` = one contiguous register read (function code + start + count).
- `ModbusSignal` = one decoded value (sensor type + channel + block index +
  decode function pointer).
- `SpbMetricDef` = one Sparkplug metric (sensor type + channel + datatype +
  alias offset + name).
- `DeviceConfig` = one slave: id, slave address, alias base, timeout, arrays.
- `kDevices[]` is the device table. Adding a meter = adding one entry.

Bus parameters (`include/ModbusConfig.h`):

| Parameter | Value |
|---|---|
| UART | `Serial2`, RX `13`, TX `12` |
| Baud / mode | 9600, `SERIAL_8N1` |
| Default timeout | 500 ms (`MODBUS_DEFAULT_TIMEOUT_MS`) |
| Block gap | 10 ms (`MODBUS_BLOCK_GAP_MS`) |
| Poll interval | 30000 ms (`POLL_INTERVAL_MS`) |
| DE/RE pin | `-1` (not configured) |

Worst-case scan: one 4-block device = 4\*500 + 3\*10 = **~2.03 s**;
8 devices = **~16.3 s**, still below `POLL_INTERVAL_MS`.

Decode helpers: `mb_u16`, `mb_i32`, `mb_byte` (`include/ModbusConfig.h:32-50`).
The meter sends **pure little-endian** (low byte / low word first). Bounds
guards return 0 instead of reading out of range.

Current device: `medidor_trifasico_1`, slave `1`, alias base `10`, 4 blocks,
17 Sparkplug metrics.

> **Hardware warning:** GPIO12 is an ESP32 strapping pin (MTDI). It is used as
> the UART TX pin. Verify the board boots reliably and that the RS485
> transceiver does/doesn't need an explicit DE/RE pin. Tracked in `AUDIT.md`.

---

## 6. Sparkplug B invariants

Topics: `spBv1.0/{group}/{message}/{edgeNode}[/{device}]`.

| Message | QoS | Retain | Notes |
|---|---:|---|---|
| NBIRTH | 0 | no | `seq = 0`, must include `bdSeq` and `Node Control/Rebirth=false` |
| DBIRTH | 0 | no | name + alias + datatype + timestamp + value |
| NDATA | 0 | no | alias + timestamp + value only (no name/datatype) |
| DDATA | 0 | no | alias + timestamp + value only |
| NDEATH | 1 (LWT) | no | `bdSeq` only, **no** `seq` |
| DDEATH | 0 | no | no metrics, includes `seq` |

Rules enforced by the code:

- Aliases are **unique across the whole edge node**: node aliases `1..6`,
  device aliases start at `aliasBase` (`10..26` today).
- `bdSeq` increments on every new MQTT CONNECT in RAM; it is persisted in NVS
  namespace `spb`, key `bdseq`, only after a successful NBIRTH.
- `seq` is shared by the whole edge node and resets to 0 on NBIRTH.
- Timestamps are **UTC epoch ms**; nothing is published until NTP is ready
  (`time(nullptr) > 1700000000`).
- Node health (`Modbus_ScanOK`) starts as `false` until the first polling cycle
  completes; `g_lastCycleOk` is written by `mainPollingTask` and read by `tareaRed`.
- `SPB_MAX_METRICS = 24` **must** match `tools/sparkplug/sparkplug_b.options`.
- `SPB_MAX_PAYLOAD_BYTES = 1536`; MQTT buffers are 2048.

Identity is in `include/netconfig.h` (local, gitignored):
group `EMASA`, edge node `nemo_medidor1`, LAN broker, MQTT anonymous,
**no TLS**. Never commit real credentials or the local broker address.

---

## 7. How to extend

**Add a block:** append to the device's blocks array and bump `blockCount`.

**Add a signal:** append to the signals array; set `blockIndex`, `sensorType`,
`channel`, and a decode lambda. Reuse `mb_u16`/`mb_i32`/`mb_byte`.

**Add a published metric:** append to the metrics array with a unique `offset`;
`alias = device.aliasBase + offset`. Reserve contiguous alias ranges per device.

**Add a device:** append to `kDevices[]` with a unique `deviceId`, `slaveId`, and
an `aliasBase` above the previous device's range. Keep
`static_assert(kDeviceCount <= kMaxDevices)`.

**Change poll rate:** `POLL_INTERVAL_MS` in `include/ModbusConfig.h`.

After changing any of the above, update this file if the invariants changed.

---

## 8. Conventions

- C++11/Arduino style, 4-space indentation, `camelCase` functions,
  `PascalCase` types, `g_` prefix for globals, `k` prefix for constants.
- Logging: use `LOG_E/W/I/D` from `include/Log.h` (`LOG_LEVEL` 0..4). Do not add
  raw `Serial.printf` in new code.
- User-facing documentation and code comments are mostly **Spanish**; this
  context file and `AUDIT.md` are **English** for agent efficiency.
- Configuration is declarative in headers; avoid hardcoding pins/addresses in
  `.cpp`.
- Do not add dynamic allocation (`new`/`malloc`) in tasks without a documented
  bound. Prefer static buffers.
- Keep ISR-free design: no work belongs in interrupts here.
- Do not edit generated files: `src/sparkplug_b.pb.c`, `include/sparkplug_b.pb.h`,
  `tools/sparkplug/python/sparkplug_b_pb2.py`.

---

## 9. Known limitations / gotchas

See `AUDIT.md` for evidence, priority, and acceptance criteria. Highlights:

- NBIRTH requires public NTP; an isolated LAN leaves the node silent.
- Modbus reads are synchronous with a 500 ms timeout -> up to ~2 s per offline
  device per cycle (8 devices ~16.3 s, still below the 30 s poll interval).
- No task watchdog and no stack high-water instrumentation.
- DE/RE pin is not configured; GPIO12 strapping must be verified.
- Birth retry is all-or-nothing (a failed DBIRTH retries the whole NBIRTH).
- Documentation drift: `TUTORIAL_MODBUS.md` and `SensorRegistry.h` still mention
  the old LoRa/JSON design.

---

## 10. Troubleshooting

| Symptom | Likely cause |
|---|---|
| No Sparkplug messages at all | NTP not synced, or MQTT not connected, or birth not done |
| Garbage on serial monitor | Wrong `monitor_speed` (must be 115200) |
| Device never appears | Wrong slave id / wiring / DE-RE / timeout too short |
| DDEATH every ~90 s | Device offline for 3 consecutive cycles |
| Rebirth ignored | NCMD not subscribed yet, wrong alias/name, or debounce window |
| Broker rejects connection | Wrong host/port, client id collision, anonymous auth disabled |
| Node reboots when bus is noisy | GPIO12 strapping, brownout, or missing watchdog/recovery |

---

## 11. Verification before finishing a change

- Build with the full `pio.exe` path; fix warnings you introduce.
- If you touch Sparkplug payloads, run the decoder/validator:
  `python tools/sparkplug/sparkplug_decode.py --host <broker>`.
- Check that a new metric has a unique alias and a matching `SpbMetricDef`.
- Check that `SPB_MAX_METRICS` still matches `sparkplug_b.options`.
- Update `AUDIT.md` checkboxes and this file if the change invalidates a section.

---

## 12. How to keep this file useful

`AGENTS.md` is a **static** context file. Agents do not regenerate it
automatically; it only stays correct if it is updated with the code.

- Update it in the **same change** that modifies tasks/cores/priorities, queue
  sizes, config structs, Sparkplug invariants, build commands, or workflows.
- Bump **Last verified** after re-reading the key files.
- Keep it short and point to `file:line` instead of pasting code.
- Put open work in `AUDIT.md`, not here. This file describes *what is*;
  `AUDIT.md` describes *what to change*.
- If a fact conflicts with code, trust code and fix the file immediately.
