# 02 — Measuring a LiPo on an ESP32-C6 (and why the chip doesn't do it for you)

*Why isn't battery measurement built into the silicon, or at least onto the carrier board?
It looks like an obvious oversight, especially from a vendor whose strategy is built on
low power. It isn't an oversight — **it's a power trade you have to make yourself**, and
here is the arithmetic.*

---

## The problem

The XIAO ESP32-C6 **does not break out a VBAT sense pin.** Neither does the C3. There is no
internal path from the battery to the ADC.

Worse for our purposes: **a 1S LiPo at 4.2 V exceeds the ADC's comfortable input range**, so
you cannot wire the cell straight to an ADC pin even if one were exposed.

So you need a divider. And the moment you add one, you meet the real problem:

## Why it isn't in silicon: a divider is a permanent load

**An always-on divider draws current every second, whether or not you ever measure anything.**

| divider | current at 4.2 V | versus the C6's ~15 µA deep sleep |
|---|---|---|
| 200 kΩ + 200 kΩ | **~10 µA** | **≈ 40 % extra sleep current** |
| 100 kΩ + 100 kΩ | ~21 µA | ≈ 140 % extra |

**Putting that in silicon would make every part pay it, forever, including the boards that
never measure a battery.** For a chip whose headline feature is microamp sleep, that is a bad
trade to force on everyone.

Three further reasons a fixed internal divider is the wrong shape of solution:

1. **A generic ADC pin has to serve any analog source.** A hard-wired divider on that pin
   would load every other thing you wanted to measure through it.
2. **The right ratio depends on the cell.** 1S LiPo (4.2 V), 2S (8.4 V), LiFePO4 (3.6 V),
   and a 12 V rail all need different divisors. There is no one-size ratio.
3. **Discharge curves are chemistry-specific**, and no silicon can know what you plugged in.

**What Espressif *did* put in silicon is the genuinely hard part:** eFuse ADC calibration and
`analogReadMilliVolts()`, which returns **calibrated** millivolts and compensates for the ADC's
non-linearity. The resistor divider is the easy part, and it's the part that costs current —
so they left it to you.

**The design rule this produces: measure the battery through a divider you can switch off.**

---

## The circuit, as built

```
        BAT+  (3.0 – 4.2 V)
          |
        [ 200k ]  R1
          |
          +------------------> D0 / GPIO0   (ADC tap)  = Vbat / 2
          |
        [ 200k ]  R2
          |
        [ 2N7002 ]  low-side N-MOSFET      <- gate
          |                                    |
         GND                              D3 / GPIO21
                                               |
                                          [ 100k ]  pull-down to GND
```

**Parts:** 2 × 200 kΩ, 1 × 100 kΩ, 1 × 2N7002 (or BSS138 — any small logic-level N-MOSFET).

**Why each piece:**

- **200 kΩ + 200 kΩ** divides the cell in half — 4.2 V becomes 2.1 V, inside the ADC's range.
  Read it back as `analogReadMilliVolts(D0) * 2`.
- **The MOSFET is what makes it viable.** The divider's low side goes to ground *through the
  FET*, so it only conducts when the gate is driven HIGH. **Sampling takes milliseconds out of
  every 60 seconds — the 10 µA becomes 10 µA × (a few ms per minute), which is negligible.**
- **The 100 kΩ gate pull-down is not optional.** If the gate pin floats during deep sleep, the
  FET can drift on and the divider quietly drains the cell all night. The pull-down makes the
  *safe* state the passive default, so nothing depends on the pin holding a level.

**The general rule here, which outlives this circuit:** put any signal that must hold a level
through deep sleep on a pin whose *passive* default is the safe state. Then a floating pin
during sleep is harmless instead of expensive.

---

## The code

```c
#define PIN_BAT_ADC  GPIO_NUM_0     /* D0 -- divider tap (LP pad, ADC capable) */
#define PIN_BAT_EN   GPIO_NUM_21    /* D3 -- MOSFET gate */

static uint32_t readCellMilliVolts() {
  pinMode(PIN_BAT_EN, OUTPUT);
  digitalWrite(PIN_BAT_EN, HIGH);          /* divider ON -- only while sampling */
  delay(20);                               /* let the tap settle */

  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(PIN_BAT_ADC);

  digitalWrite(PIN_BAT_EN, LOW);           /* divider OFF */
  return (sum / 8) * 2;                    /* undo the 2:1 divider */
}
```

**Three details that matter:**

- **`analogReadMilliVolts()`**, not `analogRead()`. The raw version needs your own calibration
  curve; this one uses the eFuse calibration the chip ships with.
- **Average several samples.** One ADC read on this part carries several millivolts of noise.
- **Gate it off immediately.** Forgetting the `digitalWrite(LOW)` is the same as never adding
  the MOSFET at all.

## Always validate the reading

**A damaged or unconnected divider produces a plausible-looking wrong number** — and a wrong
number here does not look broken, it looks like a flat battery. Classify before you trust:

```c
static const char *dividerVerdict(float v) {
  if (v < 0.5f)  return "OPEN or SHORTED - divider not connected (assembly?)";
  if (v < 2.9f)  return "BELOW LiPo floor";
  if (v > 4.35f) return "ABOVE LiPo ceiling";
  return "OK - plausible LiPo cell";
}
```

**Why this matters in practice:** this circuit is on the far side of a glued-shut enclosure.
If the divider were damaged in assembly, the symptom would be a battery that reads low
forever — and the natural conclusion would be "the cell is dying", which would send you
changing a perfectly good battery. **A verdict line turns a silent misread into an obvious
one.**

**One caution from real use:** a cell measured while the board is on USB is reading the
**charger** voltage, not the cell. 4.15 V on the bench told us nothing; the resting cell was
3.98 V. **Never compare a reading taken on USB against one taken on battery and call the
difference "drain".**
