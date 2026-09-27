# HOW TO BUILD A BATTERY-POWERED, LOW-POWER ZIGBEE DEVICE

### ⚠️ **FOLLOW THIS PATH EXACTLY OR FAIL.** ⚠️

**This is not a style guide.** Every rule below was paid for with a failed flash, and most of them
produce *silent* failures — a device that boots, joins, and reports nothing, or that panics
somewhere unhelpful. The order in which you do things matters more than anything in the code.

Written 2026-09-26 after a full day lost to doing it in the wrong order on a device whose
predecessor did it in the right order and worked perfectly. **Confirmed working end-to-end
2026-09-27.**

---

## THE RECIPE THAT ACTUALLY PRODUCED A WORKING SENSOR

Read this before the rest. Every line is load-bearing, and the sequence matters.

```
1. Define the FINAL cluster set BEFORE the first flash.       (never add a cluster later)
2. Write the external converter FIRST: bind() AND configureReporting().
3. Erase All Flash ENABLED on the first upload  -> factory-new -> the library steers by itself.
4. Hold a permit-join window open.
5. Pair ONCE, cleanly, with a firmware that cannot crash.
6. KEEP THE WINDOW OPEN while it settles -- this is what lets the coordinator's
   configure() complete. It fails every other way with "Delivery failed".
7. RUN IT ON A REAL BATTERY, not USB alone. See the next section. This is not optional.
8. THEN flash the sleeping production build, erase DISABLED, window held again.
```

## ⚠️ USB-ONLY POWER CANNOT SUSTAIN TRANSMISSION — test on a real battery

**The single hardest bug in this project, and it looks exactly like a software fault.**

A radio's transmit current peaks hard. Powered from USB alone the rail can sag during those peaks
and the packets are simply lost — **while the serial output stays perfect**, because serial draws
almost nothing. The result is a device whose every log line says it is healthy and whose every
report never arrives.

| symptom | why it misleads |
|---|---|
| serial perfect, joins fine | low-current operations, no sag |
| **every report vanishes** | **the one high-current event** |
| firmware fixes change nothing | there was no software fault |
| works when the battery is attached | a 1500 mAh LiPo is a large reservoir |

**Rule: any test involving reporting must run on the real cell, or with the cell connected.**
"USB-only, reporting fine on the bench" is not a thing on this hardware. Two separate multi-hour
debugging sessions were spent on this.

**EVIDENCE, and what is NOT yet isolated:** every USB-only run failed to deliver reports, and both
runs with the cell connected delivered them — including 36 reports over 290 s on battery alone with
no USB attached. **But the successful runs also followed a clean re-pair, so the confound is not
excluded.** Treat this as strong evidence with a named test outstanding, not as proof.
**The test that settles it:** the same firmware, on USB with the cell attached but discharging —
if reports flow there, and fail with the cell removed, the power explanation is confirmed.

Also: a mains-powered bench supply is not automatically a substitute — it may have less bulk
capacitance than the cell it replaces.

---

## TWO REAL GAPS IN THE ARDUINO ZIGBEE LIBRARY (worth reporting upstream)

Both found by reading the library's source, both with evidence, neither a user error.

### 1. `setXxxReporting()` never sets the report destination

`ZigbeeAnalog::setAnalogInputReporting()` and its siblings fill almost everything but leave the
destination unset:

```c
reporting_info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
reporting_info.manuf_code     = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
return setClusterReporting(&reporting_info);      // dst.endpoint / dst.short_addr never set
```

The resulting record has destination **`0x0000/0`** — endpoint 0 — and reports go nowhere, with
nothing logged on either side. Setting `dst.endpoint = 1` (the coordinator's ZCL endpoint) makes
them flow immediately.

**Suggested fix:** default `dst.endpoint` to 1 and `dst.short_addr` to 0x0000 for a report
addressed to the coordinator.

### 2. Network steering only happens when the device is factory-new

```c
if (esp_zb_bdb_is_factory_new()) {
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}
```

A device that still holds credentials tries a secured rejoin and — **if the coordinator refuses
it** — never falls back to searching. It sits `NOT JOINED` indefinitely (`short=0xFFFE`,
`pan=0xFFFF`) and a permit-join window alone does nothing, because the device isn't looking.

**Suggested fix / documentation:** expose steering for a device whose rejoin failed, so a
battery device that loses its network can recover without a full re-commission.

### 3. `esp_zb_zcl_update_reporting_info()` returns a misleading error

It returns `ESP_ERR_NO_MEM (257)` when there is **no existing record to update** — not an
out-of-memory condition at all. It cost hours; the device had ~290 KB free.

---

## THE ORDER OF IMPLEMENTATION IS THE ROOT CAUSE

---

## 0. THE ONE THING THAT MATTERS MOST

**A Zigbee attribute REPORT requires a REPORTING CONFIGURATION RECORD on the device. That record
is created by the COORDINATOR, and the coordinator can only create it during a FRESH JOIN.**

Therefore:

```
   firmware with the FINAL cluster set
        -> external converter written (bind + configureReporting)
             -> pair ONCE, cleanly, with a firmware that CANNOT CRASH
                  -> the coordinator configures it inside the join window
                       -> the records exist
                            -> the device reports. Forever. Across deep sleep.
```

**Any deviation from that order fails.** Specifically, and this is the mistake that cost the day:

> **NEVER ADD A CLUSTER TO A DEVICE THAT IS ALREADY PAIRED.**
> It will never be configured — there is no second join window — and reporting it will panic.
> There is no supported path that recovers this. Re-pair from scratch instead.

---

## 1. DECIDE THE CLUSTER SET FIRST — BEFORE YOU WRITE ANY FIRMWARE

Write down every endpoint and every cluster the finished device will ever have. Include the
battery cluster if it has a battery. **This list is final.** It is not "phase 1" and "add later".

A typical battery sensor:

| EP | Cluster | Purpose |
|----|---------|---------|
| 1 | `genBasic` | identity (manufacturer, model) |
| 1 | `genIdentify` | optional, harmless |
| 1 | `genBinaryInput` | the actual signal |
| 1 | **`genPowerCfg`** | **the battery — include it from the very first flash** |
| 2 | `genBinaryInput` | a fault flag (a dead sensor is not "no motion") |

**Rule: if the first flash lacks a cluster, the device can never use it.**

---

## 2. FIRMWARE — THE CALL ORDER THAT WORKS

```c
// ── registration, in this exact order ──
sensor.setManufacturerAndModel("Vendor", "Model");   // identity matters: z2m matches the converter on it
sensor.addBinaryInput();                             // creates _cluster_list
sensor.setBinaryInputApplication(...);
sensor.setBinaryInputDescription("name");
sensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, pct);  // ← AFTER addBinaryInput(), NEVER before
```

**`setPowerSource()` MUST come after `addBinaryInput()`.** It reads `_cluster_list`, which
`addBinaryInput()` creates. Called first, the cluster is built against a null list and the stack
aborts ~73 ms into setup, *before the join line prints*:

```
Zigbee stack assertion failed common/zb_address.c:816
```

⚠️ The ESP-IDF example `Zigbee_Temp_Hum_Sensor_Sleepy` calls `setPowerSource()` *first* — that is
`ZigbeeTempSensor`, a **different class**, and the order does not generalise to `ZigbeeBinary`.
**Copy the order from a device of the same class that already works.**

### WAIT FOR THE ROUTE BEFORE YOU REPORT — the silence bug

**`Zigbee.begin()` returning does NOT mean you have a working route.** A rejoining end device's
parent/coordinator route takes a moment to establish, and **a report sent before it exists is
dropped silently** — the serial looks perfect, the LED says the device is fine, and nothing ever
arrives at the coordinator.

```c
// WRONG — report at ~900 ms, then wait
pushState(...);
delay(900);                         // this is a FLUSH, not a settle

// RIGHT — settle 3000 ms FIRST, then report
delay(3000);                        // "give the parent/coordinator a beat to establish"
joined = actuallyJoined();          // ask the stack, do not assume
if (joined) { pushState(...); delay(400); }   // small flush after
```

**3000 ms, and it goes BEFORE the report.** This is copied from a device that works — the original
sedan sensor, whose constant is `ZB_REPORT_SETTLE_MS 3000  // wait after begin() before first
report (route settle)`. Ours had `900`, and applied it *after* the report. Both wrong.

**Symptom that points here:** the device's own serial shows a clean boot — radio started, sensor
read, `awake_ms` measured — and the coordinator receives nothing at all.

### NEVER HARDCODE A DIAGNOSTIC'S RESULT

```c
bool joined = true;                 // was real, replaced during debugging to stop a hang
...
if (joined) { report... }           // now reports whether or not it is on the network
blink(joined ? 2 : 3);              // and the LED lies: always blinks "joined"
```

**A diagnostic that cannot fail is worse than no diagnostic.** Ask the stack instead:

```c
extern "C" uint16_t esp_zb_get_short_address(void);
static bool actuallyJoined(void) {
  uint16_t sa = esp_zb_get_short_address();
  return (sa != 0xFFFF && sa != 0xFFFE);   // invalid short address = not on the network
}
```

### Never gate reports on `Zigbee.connected()`


```c
// WRONG - on core 3.3.11 this skips every report from a rejoining device
if (Zigbee.connected()) { report... }

// RIGHT - started() is all reportX() needs
if (Zigbee.started()) { report... }
```

The library's `_connected` flag is **not** set for an end device that re-joins (SECURED_REJOIN)
even though the device is genuinely on the network. Gating on it makes a healthy device look dead.

### Only the PERCENTAGE is reportable

```c
// BatteryVoltage (0x0020) is NOT a reportable attribute. ZigbeeEP.h says so.
// NEVER call setBatteryVoltage(). Percentage only.
presentSensor.setBatteryPercentage(pct);       // 0x0021
presentSensor.reportBatteryPercentage();
```

---

## 3. WRITE THE EXTERNAL CONVERTER **BEFORE** YOU PAIR

The converter is not decoration. Its `configure()` is **the only thing that creates the reporting
records.** Without it the device will join, report binaries fine, and panic the moment it touches
the battery.

```js
// zigbee2mqtt/data/external_converters/<device>.js
const definition = {
    zigbeeModel: ['Model'],            // MUST match setManufacturerAndModel() in the firmware
    model: 'Model',
    vendor: 'Vendor',
    fromZigbee: [fzLocal.yourCluster, fz.battery],
    toZigbee: [],
    exposes: [ e.binary(...).withEndpoint('1'), e.battery() ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const ep1 = device.getEndpoint(1);
        await ep1.bind('genBinaryInput', coordinatorEndpoint);
        await ep1.configureReporting('genBinaryInput', [
            { attribute: 'presentValue', minimumReportInterval: 0,
              maximumReportInterval: 65000, reportableChange: 1 } ]);

        // ⚠️ BOTH ARE REQUIRED. A binding alone is NOT enough. 
        await ep1.bind('genPowerCfg', coordinatorEndpoint);
        await ep1.configureReporting('genPowerCfg', [
            { attribute: 'batteryPercentageRemaining', minimumReportInterval: 0,
              maximumReportInterval: 65000, reportableChange: 1 } ]);
    },
};
module.exports = definition;
```

Register it in `configuration.yaml`:
```yaml
external_converters:
  - external_converters/<device>.js
```

**`bind()` and `configureReporting()` are two different requirements. Doing only the first is the
single most common way to lose an afternoon.**

---

## 4. PAIR ONCE, CLEANLY, WITH A FIRMWARE THAT CANNOT CRASH

**The join window is the only chance the coordinator gets.** A sleepy end device polls actively
only while joining; afterwards the coordinator's commands wait for a poll and time out:

```
Failed to configure 'device', attempt 1
  ZCL command .../1 genBinaryInput.configReport([...]) failed
  (Delivery failed for '41010'.)
```

**Four consecutive retries fail the same way. Retrying does not help — the window is gone.**

So:

1. Flash the firmware **complete and stable** — no experimental calls, no half-built features.
2. Open a permit-join window.
3. Pair. Let the coordinator interview **and configure** it.
4. **Do not reset, do not power-cycle, do not reflash** until configure has finished.

**If you miss the window** (device crash-looped, you unplugged it, the window expired):
you cannot fix it by retrying. You must give it a new window:

```c
if (!Zigbee.begin(ZIGBEE_END_DEVICE, true))   // erase_nvs = true: forget the network
```

...using a build that cannot crash, then pair again. **Remove the device from the coordinator
first** so it is interviewed as new (`zigbee2mqtt/bridge/request/device/remove` with
`force: true` if the old one is unreachable). **Reboot into a normal build afterwards** — erasing
NVS on every boot means rejoining on every boot.

---

## 5. VERIFY THE RECORD EXISTS — DO NOT ASSUME

Ask the device. This is cheap and it is the difference between knowing and hoping:

```c
extern "C" esp_zb_zcl_reporting_info_t *esp_zb_zcl_find_reporting_info(
        esp_zb_zcl_attr_location_info_t attr_info);
extern "C" bool esp_zb_lock_acquire(uint32_t timeout);
extern "C" void esp_zb_lock_release(void);

static bool batteryReportingPresent(void) {
  esp_zb_zcl_attr_location_info_t loc;
  memset(&loc, 0, sizeof(loc));
  loc.endpoint_id = 1;
  loc.cluster_id  = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
  loc.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  loc.manuf_code  = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  loc.attr_id     = 0x0021;                    // BatteryPercentageRemaining

  if (!esp_zb_lock_acquire(portMAX_DELAY)) return false;   // ⚠️ THE LOCK IS MANDATORY
  esp_zb_zcl_reporting_info_t *ri = esp_zb_zcl_find_reporting_info(loc);
  esp_zb_lock_release();
  return ri != nullptr;
}
```

⚠️ **The Zigbee stack runs in its OWN TASK.** Every SDK call from sketch code must hold the stack
lock — the library wraps all of its own calls this way. Calling without it races the stack task
and crashes the device. I built a "read-only" probe without the lock and it took the device down.

---

## 6. GUARD EVERY REPORT — THE PLATFORM PATTERN

**Ask before you touch.** Worst case the battery is not reported and the log says why, instead of
the device dying:

```c
if (batteryReportingPresent()) {
    presentSensor.setBatteryPercentage(pct);
    presentSensor.reportBatteryPercentage();
}
```

**Use this on every battery device.** A missing record must never be fatal.

Once the record exists the stack **reports on its own** — a device making no report calls at all
will publish its battery, because that is what a reporting configuration does. The manual call is
only for reporting *promptly* on change.

---

## 7. WHAT HAPPENS IF YOU GET IT WRONG

| Mitigation attempted | What actually happened |
|---|---|
| Added genPowerCfg to a paired device | never configured → panic |
| Created the binding by hand | not enough; the record is separate |
| Reported it anyway | `assert esp_zigbee_zcl_command.c:263` → reboot loop |
| Removed `setBatteryVoltage()` | still panicked |
| Reordered the report burst | still panicked |
| Downgraded the core | same core all along |
| Made the device create its own record | `ESP_ERR_NO_MEM` — SDK bug, see below |
| Retried the coordinator's configure | `Delivery failed` — window gone |

**Two different crash signatures, one cause.** Do not chase them as two bugs:

```
Zigbee stack assertion failed .../esp_zigbee_zcl_command.c:263
Guru Meditation Error: Core 0 panic'ed (Load access fault)
```

Decode them instead:

```bash
arduino-cli compile --build-path /tmp/b --fqbn "esp32:esp32:XIAO_ESP32C6:..." .
~/.arduino15/packages/esp32/tools/esp-rv32/2601/bin/riscv32-esp-elf-addr2line \
    -f -C -i -e /tmp/b/<sketch>.ino.elf 0x420307be
# -> zb_zcl_get_next_reporting_info     <- names the faulting function, one command
```

---

## 8. KNOWN DEPENDENCY — NOT YOUR BUG

**The device cannot create its own reporting configuration.** `esp_zb_zcl_update_reporting_info()`
returns `ESP_ERR_NO_MEM (257)` — a misleading error from a function with plenty of RAM.

Upstream, both open, no maintainer resolution:
- `espressif/esp-zigbee-sdk` **#728** (TZ-2063) — same cluster, same attribute, same two symptoms
- `espressif/esp-zigbee-sdk` **#341** (TZ-858) — *"using the coordinator to configure reporting …
  reporting is working"*

**The coordinator's Configure Reporting works. The device-side call does not. Do not spend time on
it.** Measured on esp-zigbee-lib 1.6.8 / esp32 core 3.3.11.

---

## 9. NON-NEGOTIABLES

1. **The cluster set is final before the first flash.** No exceptions.
2. **The converter exists before pairing, with `bind()` AND `configureReporting()`.**
3. **`setPowerSource()` after `addBinaryInput()`.**
4. **Settle the route (~3000 ms) BEFORE reporting — `Zigbee.begin()` returning is not a route.**
5. **Never hardcode a diagnostic's result.** Ask the stack (`esp_zb_get_short_address()`), or the LED and the guard both lie.
6. **Never gate reports on `Zigbee.connected()`.**
7. **Only the percentage is reportable — never `setBatteryVoltage()`.**
8. **Every SDK call from sketch code holds the stack lock.**
9. **Guard every report on the record existing.**
10. **The join window is one-shot. Pair with a firmware that cannot crash.**
11. **A blink with no serial is a HOST problem** — a manual-BOOT flash re-enumerates USB and the IDE holds a dead port. Close and reopen the IDE before touching the board.
12. **Read the working predecessor before designing.** In this project the answer was in
   `sedan_sensor_v2.ino` and in `external_converters/ami_sedan_sensor.js` the whole time.
   **Grep the project for what already works before you reason about the platform.**
13. **Replacing a device: clone its IEEE.** `esp_base_mac_addr_set()` before any radio starts,
    base MAC + `ff:fe` inserted → same Zigbee IEEE → same coordinator entry, same entity IDs,
    same references, **no Home Assistant changes at all.** Never power two boards on one address.
