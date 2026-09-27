# Ami's HA Project

Practical notes from building **Zigbee sensors and Home Assistant hardware for a real home** —
a residence in Foley, Alabama, watched over by an ambient intelligence called Ami.

**Not theory.** Everything here was paid for with a failed flash, and most of it produces *silent*
failures if you get it wrong. If a page here saves you one evening, it has done its job.

---

## Contents

| path | what it is |
|---|---|
| [`docs/01-how-to-build-a-battery-powered-zigbee-device.md`](docs/01-how-to-build-a-battery-powered-zigbee-device.md) | **The rules.** Start here. The ordered path, the failure modes, and the things that cost the most time. |
| [`examples/garage-sedan/`](examples/garage-sedan/) | A **complete working battery-powered Zigbee sensor** — VL53L0X distance, deep sleep, battery reporting, cloned identity. End to end, no framework. |
| [`bugs/`](bugs/) | Issues found in the Arduino ESP32 Zigbee stack, with reproductions and evidence. |

---

## What's in here that's hard to find elsewhere

**The order of implementation is the root cause.** A Zigbee attribute report needs a *reporting
configuration record*, that record is created by the **coordinator**, and the coordinator can only
create it **during a fresh join**. Add a cluster to an already-paired device and it can never be
configured. There is no recovery. This single fact explains a remarkable amount of otherwise
inexplicable pain.

**A battery-powered radio cannot be debugged on USB alone.** The transmit current peaks, the rail
sags, and the packets vanish — **while the serial output stays perfect**, because serial draws
almost nothing. The device reports its own health beautifully and delivers nothing. Test on the
real cell.

**Two real gaps in the Arduino Zigbee library**, documented in [`bugs/`](bugs/) — a reporting
destination that is never set (reports silently discarded, nothing logged on either side), and a
network-steering path that only runs for factory-new devices, so a deployed device that loses its
network can never find it again.

---

## The hardware this came from

Seeed **XIAO ESP32-C6** · Arduino ESP32 core 3.3.11 · esp-zigbee-lib 1.6.8 · zigbee2mqtt ·
Home Assistant · a 1500 mAh LiPo.

The sensor monitors whether a car is parked on a driveway spot, reports over Zigbee, deep-sleeps
between samples, and reports its own battery — the last of which turned out to be the hard part.

---

## License

MIT — see [LICENSE](LICENSE).

## A note on tone

These documents are written the way we wish someone had written them for us: plainly, with the
failures included. Where a claim is a hypothesis rather than a measurement, **it says so.**
That distinction is the most useful thing in here.
