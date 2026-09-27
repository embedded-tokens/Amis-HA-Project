# garage_sedan -- a working battery-powered Zigbee sensor

A complete, working example: **VL53L0X distance sensor -> Zigbee end device -> deep sleep ->
battery reporting**, built on a Seeed XIAO ESP32-C6.

This is the real firmware from a deployed sensor. It is not a demo sketch -- it carries the fixes
for several silent failure modes that are documented in
[the bible](../../docs/01-how-to-build-a-battery-powered-zigbee-device.md) at `docs/01-...`.

## Files

| file | what it is |
|---|---|
| `garage_sedan.ino` | the firmware |
| `ami_sedan_sensor.js` | the zigbee2mqtt **external converter** -- required, not optional |

## Hardware

- Seeed XIAO ESP32-C6
- VL53L0X time-of-flight sensor (I2C)
- 1500 mAh LiPo with a divider on GPIO0 / enable on GPIO21
- zigbee2mqtt + Home Assistant

## Setup -- IN THIS ORDER

**The order is not a suggestion. Done out of order, the device will join, report nothing, and log
no reason why.**

```
1. Install the external converter
     copy ami_sedan_sensor.js to zigbee2mqtt/data/external_converters/
     register it in configuration.yaml under external_converters:
     restart zigbee2mqtt

2. Flash the board -- FIRST TIME ONLY:
     Tools -> Erase All Flash Before Sketch Upload = ENABLED
     (a factory-new device starts network steering by itself)

3. Open a permit-join window in zigbee2mqtt

4. POWER IT FROM THE BATTERY, not USB alone.
     USB alone cannot sustain the transmit current; reports vanish while the
     serial looks perfect. This is the single most misleading failure here.

5. Pair once, cleanly. Let the coordinator interview AND configure it.
     KEEP THE JOIN WINDOW OPEN while it settles -- that is what lets configure()
     complete. It fails every other way with "Delivery failed".

6. For production, re-flash with erase DISABLED so the network state survives.
     Reboot -- it should rejoin on its own in a few hundred ms.
```

## What it reports

```
EP1  genBinaryInput   car present   ON / OFF
EP1  genPowerCfg      battery       percentage (standard cluster, native HA entity)
EP2  genBinaryInput   lidar fault   ON = sensor init failed or a read timed out
```

**It reports on every wake, not only on change.** A device that reports only on change is
indistinguishable from a dead one -- "healthy and quiet" and "broken" look identical.

## Tuning

| define | default | meaning |
|---|---|---|
| `SLEEP_SECS` | 60 | deep-sleep interval |
| `CAR_DISTANCE_MM` | 1000 | at or below this = car present |
| `MIN_VALID_MM` | 100 | below this = crosstalk, discard the reading |
| `REPORT_SETTLE_MS` | 3000 | wait for the route before reporting |

**A presence test is a BAND, not a threshold.** Note `MIN_VALID_MM`: the VL53L0X reports a
persistent short phantom from the case aperture, and discarding sub-100 mm readings is what makes
the sensor usable inside an enclosure.
