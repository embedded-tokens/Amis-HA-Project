# `ZigbeeEP::setXxxReporting()` never sets the report destination — reports are silently dropped

**Component:** Arduino ESP32 core, `libraries/Zigbee`
**Versions:** esp32 core **3.3.11**, esp-zigbee-lib **1.6.8** (zboss 1.6.4)
**Hardware:** Seeed XIAO ESP32-C6, end device, battery powered
**Zigbee mode:** ED · Partition: zigbee · CDCOnBoot=cdc

---

## Summary

`ZigbeeAnalog::setAnalogInputReporting()` — and every sibling `setXxxReporting()` method — builds a
reporting-configuration record that fills in everything **except the destination**. The resulting
record has `dst.short_addr = 0x0000` and `dst.endpoint = 0`. Reports built from that record go
nowhere, and **nothing is logged on the device or the coordinator**. It looks exactly like the
device never reported.

Setting `dst.endpoint = 1` (the coordinator's ZCL endpoint) makes reports flow immediately, with no
other change.

## The code

`libraries/Zigbee/src/ep/ZigbeeAnalog.cpp`, in `setAnalogInputReporting()`:

```c
reporting_info.u.send_info.min_interval     = min_interval;
reporting_info.u.send_info.max_interval     = max_interval;
reporting_info.u.send_info.def_min_interval = min_interval;
reporting_info.u.send_info.def_max_interval = max_interval;
reporting_info.u.send_info.delta.s32        = delta;
reporting_info.dst.profile_id               = ESP_ZB_AF_HA_PROFILE_ID;
reporting_info.manuf_code                   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

return setClusterReporting(&reporting_info);      // <-- dst.endpoint / dst.short_addr never set
```

`dst.endpoint` and `dst.short_addr` are left at their `memset`-zero values.

## Evidence

Reading the record back from the device with `esp_zb_zcl_find_reporting_info()`:

```
default (built by the library path):  min=5  max=0     delta=0  dst=0x0000/0  flags=0x51
after setting dst.endpoint = 1:       min=0  max=65000 delta=1  dst=0x0000/1  flags=0x15
```

With `dst=0x0000/0`: **zero reports delivered** over 100+ s, while the device's own serial showed a
healthy boot, a successful join (`short=0x13CA pan=0x1A62 ch=11`), existing bindings, and
`reportBinaryInput()` / `reportBatteryPercentage()` being called every cycle.

With `dst=0x0000/1`: **reports arrive at ~1 per 60 s immediately**, on the same firmware, same
device, same network.

## Suggested fix

Default the destination to the coordinator when a reporting config is created:

```c
reporting_info.dst.short_addr = 0x0000;                    /* coordinator */
reporting_info.dst.endpoint   = 1;                         /* coordinator ZCL endpoint */
reporting_info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
```

At minimum, please document that callers must set `dst` themselves — the current behaviour is
undiscoverable, and the failure is silent.

---

## Two related issues found in the same investigation

### 1. Network steering only runs when the device is factory-new

`libraries/Zigbee/src/ZigbeeCore.cpp`, in the `DEVICE_FIRST_START` / `DEVICE_REBOOT` handler:

```c
if (esp_zb_bdb_is_factory_new()) {
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}
```

A device that still holds credentials attempts a secured rejoin, and **if the coordinator refuses
it, it never falls back to searching**. It stays `NOT JOINED` indefinitely:

```
network short address = 0xFFFE   pan=0xFFFF  ch=0
```

A permit-join window does nothing, because the device is not looking. For a battery-powered device
that may lose its network (coordinator replaced, credentials invalidated), the only recovery was a
full `erase_nvs` re-commission. **Suggest exposing a way to start steering when a rejoin has
failed**, so a deployed device can recover without being physically re-commissioned.

### 2. `esp_zb_zcl_update_reporting_info()` reports `ESP_ERR_NO_MEM` when no record exists

Calling it when there is no existing record for the attribute returns `ESP_ERR_NO_MEM (257)`. It is
not an out-of-memory condition — the device had ~290 KB free. The name sends you looking for a
memory leak. `ESP_ERR_NOT_FOUND` would describe it accurately.

---

## Environment notes

- Reproduced consistently across ~15 builds over two days.
- `Zigbee.begin()` succeeds, the device joins, the interview completes in zigbee2mqtt, and bindings
  are present throughout — the failure is invisible from the coordinator side.
- A second, unrelated confound that cost significant time and may be worth documenting: **testing
  report delivery while powered from USB alone**. The radio's transmit peaks appear to be
  unsupported without the battery attached, producing identical symptoms (perfect serial, no
  reports). Not a library issue, but a note for anyone else debugging "it reports nothing" on this
  hardware.
