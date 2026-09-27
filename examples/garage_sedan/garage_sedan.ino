/* ============================================================================
 * garage_sedan -- battery-powered Zigbee car-presence sensor
 * ============================================================================
 *
 * WHAT IT DOES
 *   Wakes every 60 s, takes one VL53L0X distance sample, reports "car present"
 *   and its own battery over Zigbee, and deep-sleeps again.
 *
 * READ THE RULES FIRST
 *   docs/01-how-to-build-a-battery-powered-zigbee-device.md
 *   Every choice below is load-bearing, and most produce SILENT failures if
 *   changed. The five that matter most:
 *
 *   1. THE CLUSTER SET IS FINAL BEFORE THE FIRST FLASH.
 *      Never add a cluster to an already-paired device -- the coordinator can
 *      only configure a device during its fresh-join window, so a cluster added
 *      later can never get a reporting record. There is no recovery.
 *
 *   2. setPowerSource() comes AFTER addBinaryInput().
 *      It reads _cluster_list, which addBinaryInput() creates. Called first, the
 *      stack aborts ~73 ms into setup, before the join line even prints.
 *
 *   3. SETTLE ~3000 ms AFTER Zigbee.begin() AND BEFORE REPORTING.
 *      begin() returning does not mean a route exists. Reports sent too early
 *      are dropped with nothing logged anywhere.
 *
 *   4. THE LIBRARY NEVER SETS THE REPORTING DESTINATION.
 *      It leaves dst.endpoint = 0, so reports go nowhere. This firmware corrects
 *      its own record at every wake -- see fixBatteryReportingRecord().
 *
 *   5. TEST ON THE REAL CELL, NOT USB ALONE.
 *      The transmit current peaks; on USB the rail sags and packets vanish while
 *      the serial stays perfect. The failure looks exactly like a software bug.
 *
 * HARDWARE
 *   Seeed XIAO ESP32-C6 (end device) + VL53L0X ToF on I2C + 1500 mAh LiPo.
 *
 * PINS
 *   GPIO22 / GPIO23   I2C SDA / SCL
 *   GPIO2             XSHUT -- lidar power gate, held LOW through sleep (~5 uA)
 *   GPIO15            onboard LED (ACTIVE LOW)
 *   GPIO0             battery divider tap (ADC)
 *   GPIO21            divider enable -- low-side MOSFET gate
 *
 * ZIGBEE ENDPOINTS
 *   EP1  genBinaryInput "car present" + genPowerCfg (battery, standard cluster)
 *   EP2  genBinaryInput "lidar fault" -- a dead sensor must not read as "no car"
 *
 * REPORTING CADENCE
 *   Every wake reports, whether or not anything changed, so that "healthy and
 *   quiet" and "dead" are never the same observation.
 * ============================================================================ */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "Zigbee.h"
#include "esp_mac.h"          /* esp_base_mac_addr_set / esp_read_mac */

/* ══ DEVICE IDENTITY — A DROP-IN REPLACEMENT FOR THE ORIGINAL SENSOR ══════════
 * zigbee2mqtt keys every device on its IEEE (long) address, and Home Assistant
 * builds entity_ids from it:
 *      sensor.0x58e6c5fffe1aabb4_battery
 *      binary_sensor.0x58e6c5fffe1aabb4_sedan_present_1
 *
 * So a board that presents the SAME IEEE is not "a new device" to z2m or HA at
 * all: it inherits the device entry, the friendly name, the entity_ids, the
 * history, and every Ami reference. NOTHING in HA needs changing.
 *
 * The address is the original garage_sedan's, and it is not arbitrary:
 *      Zigbee IEEE  0x58e6c5fffe1aabb4
 *      base MAC     58:e6:c5:1a:ab:b4      (ff:fe inserted mid-way, ESP32 rule)
*  Setting the BASE MAC before any radio starts makes the stack derive exactly
 * that IEEE. Verified present in the installed SDK: esp_mac.h:61.
 *
 * CAVEAT: never power two boards with the same address at once. The original is
 * dismantled; the board being reused elsewhere keeps its own MAC untouched.
 * ═══════════════════════════════════════════════════════════════════════════ */
static const uint8_t AMI_BASE_MAC[6] = {0x58, 0xe6, 0xc5, 0x1a, 0xab, 0xb4};
#define AMI_IEEE_STRING "0x58e6c5fffe1aabb4"

/* Set to 1 only AFTER z2m's device definition lists a `battery` expose, i.e.
 * after a CLEAN first interview that saw the Power Config cluster from birth. */
/* ── HOW THE BATTERY GOES OVER ZIGBEE ─────────────────────────────────────
 * 0 = do not touch the attribute, do not report  (bench build: PROVEN STABLE)
 * 1 = update the attribute only                  (this build)
 * 2 = update the attribute AND send a device-initiated report
 *                                                  (bench build/12: PANICS)
 *
 * bench build (0) ran clean and z2m showed battery:null -- the entity exists, it
 * simply never received a value. bench build (2) panics with a Load access fault
 * ~340 ms after the local battery print, i.e. inside the battery report call.
 * Mode 1 splits the two calls apart: if it stays up, the fault is in
 * reportBatteryPercentage(); if it panics too, it is in setBatteryPercentage().
 *
 * AND it may be all we actually need: z2m reads POWER CONFIG attributes from a
 * sleepy device via its parent. If mode 1 fills in the battery value, the
 * device-initiated report is unnecessary -- no crash, and Ami still gets a
 * battery entity. That is the question this build answers.
 * ───────────────────────────────────────────────────────────────────────── */
#define BATTERY_PUSH_MODE 2
#include "ep/ZigbeeBinary.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

/* ── build identity — always know which firmware is on the board ────────── */
#define FW_BUILD "this build"

/* ── sleep ─────────────────────────────────────────────────────────────── */
#define SLEEP_SECS      60
#define uS_TO_S_FACTOR  1000000ULL

/* ── the bench twin sets this to 0; the production build sleeps ────────── */
#ifndef DEEP_SLEEP
#define DEEP_SLEEP 1
#endif

/* ── measurement — the BAND rule carried forward from Phase 0/1 ──────────
 * The part reports "no usable target" three different ways and a one-sided
 * rule turns two of them into a false PRESENT:
 *     8190      out of range                (library sentinel)
 *     0         the read failed
 *     25-58 mm  crosstalk phantom from the case aperture (measured on real hardware)
 * Anything below MIN_VALID_MM is ABSENT. Fail-safe direction: unmeasurable
 * must never assert presence.                                        */
#define CAR_DISTANCE_MM 1000
#define MIN_VALID_MM     100
#define OOR_SENTINEL    8190

/* ── every wait is bounded — never hang on a battery ───────────────────── */
#define JOIN_TIMEOUT_MS  8000   /* gives Zigbee.begin() time, then we sleep anyway */
/* ROUTE SETTLE -- 3000 ms, and it goes BEFORE the report, not after.
 *
 * THE BUG THIS FIXES -- a rejoin does not give you a route immediately:
 *     ORIGINAL:  #define ZB_REPORT_SETTLE_MS 3000  // wait after begin() before first report
 *                delay(ZB_REPORT_SETTLE_MS);        // ...then report
 *     OURS WAS:  report first, then delay(900)      // to let it flush before sleeping
 *
 * A rejoin does not give you a working route the instant Zigbee.begin() returns.
 * The original waits 3 s for the parent/coordinator route to establish and only
 * THEN reports; its comment says so outright. Ours reported at ~900 ms into a
 * ~900 ms wake -- before the route existed -- so every report was dropped on the
 * floor while the serial looked perfect. That is the whole of "the device runs
 * but z2m never hears it".
 *
 * The post-report flush is kept small and separate: the settle is about the
 * route being READY, not about the report draining. */
#define REPORT_SETTLE_MS        3000  /* wait for the rejoin route BEFORE reporting */
#define REPORT_FLUSH_MS          400  /* brief drain so the burst leaves before sleep */

/* ── NO UNNECESSARY CLUSTERS ──────────────────────────────────────────────
 * This device carries EXACTLY two endpoints. That is deliberate: every cluster
 * must be present before the first pairing, and each one has to be bound and
 * configured by the coordinator or it is dead weight that can abort the stack.
 *
 * NOTE FOR THE CURIOUS: an earlier version of this project concluded that the
 * analog-input report path was "broken". It is not. The analog endpoints failed
 * for exactly the same reason the battery did -- no reporting configuration
 * record, because they had been added to an ALREADY-PAIRED device. Same bug,
 * different attribute. Do not repeat that conclusion. */

/* ── pins (identical to the proven Phase 1 build) ──────────────────────── */
#define PIN_SDA    GPIO_NUM_22
#define PIN_SCL    GPIO_NUM_23
#define PIN_XSHUT  GPIO_NUM_2   /* D2, LP pad — read-back unreliable, see header */
#define PIN_LED    15           /* onboard user LED, ACTIVE LOW                  */

/* ── battery measurement ──────────────────────────────────────────────────
 * Turned on to answer two questions at once:
 *   1. does the divider actually survived the mechanical assembly? (it is
 *      buried in the case and cannot be inspected)
 *   2. what does the cell do over hours — i.e. the drain gate as a measurement
 *      rather than arithmetic.
 *
 * THE READOUT PROBLEM, AND HOW THIS SOLVES IT: on battery there is no USB, so
 * there is no serial to read a voltage from -- and probing the cell in the case
 * would mean ungluing a package whose mechanical build is already proven.
 * So every reading is stored in RTC memory and REPRINTED at the next boot:
 *
 *      last_cell_mV=...      <- what the cell was at the PREVIOUS wake
 *
 * Run on battery for hours, then plug USB in and read the first line. RTC
 * memory survives a reset (not a power loss), so the USB plug does not lose it.
 * No new Zigbee cluster is involved -- which matters, because an unconfigured
 * cluster is what ABORTS this chip (see the REVISION notes).
 *
 * LIMITATION, stated up front: LiPo terminal voltage is nearly FLAT between
 * about 4.0 V and 3.7 V, which is most of the discharge. Voltage-over-hours is
 * therefore a COARSE drain proxy -- good for "is it draining plausibly" and for
 * a low-cell warning, NOT a substitute for a current measurement. The precise
 * drain figure still comes from the wake/rejoin timing (awake_ms).
 */
#define ENABLE_BATTERY_MEASUREMENT 1
#define PIN_BAT_ADC  GPIO_NUM_0     /* D0 — divider tap     */
#define PIN_BAT_EN   GPIO_NUM_21    /* D3 — low-side MOSFET gate */
#define VBAT_RATIO   2.0f           /* 200k + 200k               */

/* ── endpoints ─────────────────────────────────────────────────────────── */
#define EP_PRESENT 1
#define EP_FAULT   2

VL53L0X sensor;

/* ── WHY THIS SUBCLASS EXISTS ─────────────────────────────────────────────
 * ZigbeeEP::setClusterReporting() is PROTECTED (ZigbeeEP.h:243 sits under
 * "protected:"), so a sketch cannot call it on a ZigbeeBinary. A subclass can.
 *
 * We need it because a Zigbee attribute REPORT requires a REPORTING
 * CONFIGURATION RECORD on the device. Without one, any call that touches the
 * attribute walks the reporting table, finds NULL and panics:
 *     Guru Meditation / Load access fault
 *     MEPC -> zb_zcl_get_next_reporting_info
 *     A1   -> 0x21  (BATTERY_PERCENTAGE_REMAINING)
 * Both setBatteryPercentage() AND reportBatteryPercentage() die this way.
 *
 * Until now that record was only ever created by the COORDINATOR, in
 * zigbee2mqtt's configure() (see external_converters/ami_sedan_sensor.js, which
 * binds genPowerCfg and calls configureReporting). That makes the device depend
 * on the coordinator having configured it -- and a device whose cluster set
 * changed after pairing never gets configured at all.
 *
 * Setting it HERE makes the device self-sufficient: the record exists from
 * every boot, with no dependence on the coordinator. ────────────────────── */
/* BatteryPercentageRemaining = 0x0021, Power Configuration cluster (0x0001),
 * ZCL spec. Written as a literal because zcl/esp_zigbee_zcl_power_config.h is
 * not on the sketch include path (same reason esp_zigbee_nwk.h would not
 * resolve), and this attribute ID is fixed by the spec -- it can never change.
 * The library uses the same value internally. */
#define AMI_ATTR_BATTERY_PERCENTAGE_REMAINING 0x0021

class AmiBinary : public ZigbeeBinary {
 public:
  AmiBinary(uint8_t endpoint) : ZigbeeBinary(endpoint) {}
  bool configureReporting(esp_zb_zcl_reporting_info_t *ri) { return setClusterReporting(ri); }
};

AmiBinary presentSensor(EP_PRESENT);
AmiBinary faultSensor(EP_FAULT);

/* Set an attribute's reporting config, populated exactly the way the library's
 * own worked example does it (ZigbeeAnalog.cpp:264 setAnalogInputReporting):
 * memset to zero, dst.profile_id = HA profile, manuf_code = non-manufacturer. */
/* SDK entry points, declared here because zcl/esp_zigbee_zcl_command.h is not on
 * the sketch include path (same trap as esp_zigbee_nwk.h). Signatures copied from
 * the installed header. */
extern "C" esp_zb_zcl_reporting_info_t *esp_zb_zcl_find_reporting_info(esp_zb_zcl_attr_location_info_t attr_info);
/* The Zigbee stack runs in its OWN TASK, so every SDK call from sketch code must
 * hold the stack lock -- this is how the library wraps all of its own calls.
 * Without it the lookup races the stack task. */
extern "C" bool esp_zb_lock_acquire(uint32_t timeout);
extern "C" void esp_zb_lock_release(void);
/* The honest join signal. 0xFFFF / 0xFFFE = no network address = NOT joined.
 * Read from the stack rather than a cached flag -- the library's connected()
 * is not set for a rejoining end device on core 3.3.11, which is what made a
 * healthy device look absent in the first place. */
extern "C" uint16_t esp_zb_get_short_address(void);
/* Updates an EXISTING reporting-configuration record. Returns ESP_ERR_NO_MEM if
 * there is no record to update -- which is why our earlier attempt failed: we
 * called it when no record existed. At boot the stack builds a default one, so
 * there is always something to correct. */
extern "C" esp_err_t esp_zb_zcl_update_reporting_info(esp_zb_zcl_reporting_info_t *report_info);

/* Call the SDK DIRECTLY rather than through the library's setClusterReporting().
 *
 * ZigbeeEP::setClusterReporting() reports failure with log_e/log_w, which are
 * COMPILED OUT at the default CORE_DEBUG_LEVEL -- so it returns false and says
 * nothing. Going direct lets us print the esp_err_t, which names the failure.
 * ESP_ERR_NO_MEM here is a MISNOMER: it means "no record to update", not "out of
 * memory". */
/* ── READ-ONLY PROBE: does the reporting record exist? ────────────────────
 * The device CANNOT create this record itself. Measured, twice:
 *   esp_zb_zcl_update_reporting_info() -> ESP_ERR_NO_MEM (257)
 * Upstream confirms it is a library bug, and that the COORDINATOR's Configure
 * Reporting works where the device-side call does not:
 *   espressif/esp-zigbee-sdk issues #728 (TZ-2063) and #341 (TZ-858) — same
 *   cluster, same attribute, same two symptoms we have (ZCL assertion at
 *   esp_zigbee_zcl_command.c:263, and Load access fault in the report path with
 *   0x21 in the registers). #728 has NO maintainer resolution.
 *
 * So the record must be created by zigbee2mqtt's configure() -- which is what
 * external_converters/ami_sedan_sensor.js does: bind('genPowerCfg') AND
 * configureReporting('genPowerCfg', batteryPercentageRemaining). That is how the
 * ORIGINAL sensor got its record, and it persists in zb_storage across deep
 * sleep -- which is why the original's battery still reports after every reboot.
 *
 * This probe only LOOKS. It never touches the attribute, so this build cannot
 * fault -- and it turns "does the record exist?" into something visible on the
 * wire every loop iteration. Run z2m's configure and the line flips to PRESENT.
 * ─────────────────────────────────────────────────────────────────────────── */
/* ── THE GUARD ────────────────────────────────────────────────────────────
 * Returns true if the reporting record exists for the battery attribute, and
 * prints the detail. Holds the Zigbee stack lock, because the stack runs in its
 * own task and this is called from setup() and from loop().
 *
 * WHY THIS EXISTS: both setBatteryPercentage() and reportBatteryPercentage()
 * walk the reporting table. With NO record for the attribute, that walk
 * dereferences NULL and panics:
 *     Load access fault in zb_zcl_get_next_reporting_info
 *     / zb_zcl_send_report_attr_command, with 0x21 in the registers
 * The record is created by the COORDINATOR -- z2m's configure(), driven by
 * external_converters/ami_sedan_sensor.js:
 *     bind('genPowerCfg') + configureReporting('genPowerCfg', [batteryPercentageRemaining])
 * The device CANNOT create it: esp_zb_zcl_update_reporting_info() returns
 * ESP_ERR_NO_MEM (esp-zigbee-sdk #728 / #341, both open).
 *
 * So we ASK FIRST. Worst case the battery is simply not reported and the log
 * says why -- instead of taking the whole device down.
 * PLATFORM PATTERN: use this on every battery device we build.
 * ─────────────────────────────────────────────────────────────────────────── */
static bool batteryReportingPresent(void) {
  esp_zb_zcl_attr_location_info_t loc;
  memset(&loc, 0, sizeof(loc));
  loc.endpoint_id  = EP_PRESENT;
  loc.cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
  loc.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  loc.manuf_code   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  loc.attr_id      = AMI_ATTR_BATTERY_PERCENTAGE_REMAINING;

  if (!esp_zb_lock_acquire(portMAX_DELAY)) {
    Serial.println("battery reporting record: probe could not take the stack lock");
    return false;
  }
  esp_zb_zcl_reporting_info_t *ri = esp_zb_zcl_find_reporting_info(loc);

  /* copy out while locked; never print inside the lock */
  bool found = (ri != nullptr);
  uint16_t mn = 0, mx = 0, dsh = 0; uint8_t du8 = 0, dep = 0, fl = 0;
  if (found) {
    mn = ri->u.send_info.min_interval;  mx = ri->u.send_info.max_interval;
    du8 = ri->u.send_info.delta.u8;
    dsh = ri->dst.short_addr;           dep = ri->dst.endpoint;
    fl = ri->flags;
  }
  esp_zb_lock_release();

  if (!found) {
    Serial.println("battery reporting record: NOT PRESENT  <- skipping the battery report (it would fault)");
    return false;
  }
  Serial.printf("battery reporting record: PRESENT  min=%u max=%u delta=%u "
                "dst=0x%04x/%u flags=0x%x  <- the report is safe\n",
                mn, mx, du8, dsh, dep, fl);
  return true;
}

/* ── survives deep sleep, NOT power loss ───────────────────────────────────
 * This is deliberate: wakeCount is also a reset detector. If it ever comes
 * back as 1 after we have been running a while, the power was interrupted —
 * a brownout or a flat cell — and we learn that from the network.        */
/* Initial battery percentage handed to setPowerSource() at registration.
 * Set from a real measurement in setup() before startRadioAndEndpoints(), the
 * way the working original does it. A global so the registration helper does
 * not have to reach forward to the battery reader. */
static uint8_t gInitialPct = 100;

RTC_DATA_ATTR uint32_t wakeCount = 0;


/* The PREVIOUS wake's full awake duration. RTC memory so it survives sleep, and
 * published one wake late on purpose: awake_ms is not known until the report and
 * its settle time are done, which is exactly the part the drain model gets wrong. */
RTC_DATA_ATTR uint32_t lastAwakeMs = 0;

/* The cell voltage measured on the PREVIOUS wake, in mV. RTC memory: survives
 * deep sleep AND a reset, so the value from a battery run is still there to be
 * printed the moment USB is plugged back in. 0 = nothing recorded yet. */
RTC_DATA_ATTR uint16_t lastCellMv = 0;

static uint32_t awakeStartMs = 0;

/* ── LED: a glance must distinguish alive from dead ────────────────────── */
static inline void ledOn()  { digitalWrite(PIN_LED, LOW);  }   /* active LOW */
static inline void ledOff() { digitalWrite(PIN_LED, HIGH); }

static void blink(uint8_t times, uint16_t onMs = 90, uint16_t gapMs = 140) {
  for (uint8_t i = 0; i < times; i++) {
    ledOn();  delay(onMs);
    ledOff(); if (i + 1 < times) delay(gapMs);
  }
}

/* ─────────────────────────────────────────────────────────────────────────
 * Sensor power + init. XSHUT is the only reliable control we have; the pin
 * read-back is not trustworthy on this LP pad, so init() is the arbiter.
 * ───────────────────────────────────────────────────────────────────────── */
static bool initSensor(void) {
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);      /* ensure a clean power-on reset */
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);     /* release from shutdown */
  delay(10);                         /* datasheet: <=1.2 ms to boot, 10 is safe */

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  if (!sensor.init()) return false;
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(50000);
  return true;
}

/* ─────────────────────────────────────────────────────────────────────────
 * Radio first — so a sensor failure still has a transport to report on.
 * ───────────────────────────────────────────────────────────────────────── */
static void startRadioAndEndpoints(void) {
  /* ── THE BATTERY, DONE THE STANDARD WAY ────────────────────────────────
   * Power Configuration cluster (0x0001) on the PRIMARY endpoint. This is not
   * a nicety: it is the only form zigbee2mqtt / Home Assistant -- and therefore
   * Ami -- understand natively as a BATTERY ENTITY. A custom number on a custom
   * endpoint would be invisible to every consumer. This is PLATFORM
   * INFRASTRUCTURE for every battery device we build, not a per-device extra.
   *
   * ORDER MATTERS: setPowerSource() MUST come BEFORE addBinaryInput(). The
   * library creates the power-config cluster during addBinaryInput(), and the
   * vendor's own working example calls them in this order.
   *
   * BatteryVoltage (0x0020, 100 mV units) is settable but NOT reportable, so
   * the LIVE signal is BatteryPercentageRemaining (0x0021) via
   * reportBatteryPercentage(). The voltage is still set each wake so it is
   * readable whenever the coordinator reads the attribute.              */
  presentSensor.setManufacturerAndModel("Ami", "SedanSensorV2");
  presentSensor.addBinaryInput();
  presentSensor.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_OTHER);
  presentSensor.setBinaryInputDescription("sedan_present");

  /* ── ORDER IS LOAD-BEARING ─────────────────────────────────────────────────
   * addBinaryInput() FIRST, setPowerSource() SECOND.
   *
   * setPowerSource() begins with
   *     esp_zb_cluster_list_get_cluster(_cluster_list, BASIC, SERVER)
   * and _cluster_list DOES NOT EXIST until addBinaryInput() has run. Called
   * first, the power-config cluster is built against a null list and the stack
   * aborts ~73 ms into setup -- before the join line even prints:
   *     Zigbee stack assertion failed common/zb_address.c:816
   *
   * The older ESP-IDF example (Zigbee_Temp_Hum_Sensor_Sleepy) calls
   * setPowerSource() BEFORE adding its sensor cluster. That is ZigbeeTempSensor,
   * a DIFFERENT class, and the order does NOT generalise to ZigbeeBinary.
   *
   * Taken from the WORKING garage sensor, a device of the same class that already works lines 183-188,
   * which calls addBinaryInput() then setPowerSource(). Copy the working order.
   * ───────────────────────────────────────────────────────────────────────── */
  presentSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, gInitialPct);

  faultSensor.setManufacturerAndModel("Ami", "SedanSensorV2");
  faultSensor.addBinaryInput();
  faultSensor.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_OTHER);
  faultSensor.setBinaryInputDescription("sensor_fault");

  Zigbee.addEndpoint(&presentSensor);
  Zigbee.addEndpoint(&faultSensor);
}

/* Bounded wait — the whole point is that we SLEEP anyway if this fails. */
static bool waitForJoin(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (Zigbee.connected()) return true;
    delay(50);
  }
  return Zigbee.connected();
}

static uint16_t readDistance(bool sensorOk, bool *timeoutOut) {
  if (!sensorOk) { *timeoutOut = false; return OOR_SENTINEL; }
  uint16_t d = sensor.readRangeSingleMillimeters();
  *timeoutOut = sensor.timeoutOccurred();
  return d;
}

#if ENABLE_BATTERY_MEASUREMENT
/* Classify the reading so a divider that was damaged or never connected in the
 * assembly is OBVIOUS, instead of being quietly read as "the battery is flat".
 * A floating ADC pin tends to read near zero or wander. */
static const char *dividerVerdict(float v) {
  if (v < 0.5f)   return "OPEN or SHORTED - divider not connected (assembly?)";
  if (v < 2.50f)  return "IMPOSSIBLE for a LiPo - check the tap/ratio";
  if (v <= 4.35f) return "OK - plausible LiPo cell";
  return "HIGH - charger rail, or wrong ratio";
}

/* Map cell voltage to a state-of-charge percentage.
 * 1S LiPo, piecewise linear. Stated plainly because it matters: voltage is a
 * POOR proxy for state of charge -- it is nearly flat between 4.0 and 3.7 V,
 * which is most of the capacity. This is good enough to say "charge me soon"
 * and NOT good enough to say "you have 47% left". Ami should threshold on it,
 * not trust it to the percent. */
static uint8_t lipoPercent(float v) {
  static const struct { float v; uint8_t p; } T[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.93f, 70}, {3.87f, 60},
    {3.82f,  50}, {3.78f, 40}, {3.74f, 30}, {3.70f, 20}, {3.62f, 10},
    {3.50f,   5}, {3.30f,  0},
  };
  if (v >= T[0].v) return 100;
  for (size_t i = 1; i < sizeof(T) / sizeof(T[0]); i++) {
    if (v >= T[i].v) {
      float f = (v - T[i].v) / (T[i - 1].v - T[i].v);
      return (uint8_t)(T[i].p + f * (T[i - 1].p - T[i].p) + 0.5f);
    }
  }
  return 0;
}

static float readBatteryVolts(void) {
  if (PIN_BAT_EN >= 0) {
    pinMode(PIN_BAT_EN, OUTPUT);
    digitalWrite(PIN_BAT_EN, HIGH);       /* gate the divider ON only while sampling */
    delay(5);
  }
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(PIN_BAT_ADC);
  if (PIN_BAT_EN >= 0) digitalWrite(PIN_BAT_EN, LOW);
  return (float)sum / 8.0f / 1000.0f * VBAT_RATIO;
}

/* Measure, print, remember, and return the percentage. Local only -- it does
 * NOT touch Zigbee here. */
static uint8_t measureBatteryPct(void) {
  float v = readBatteryVolts();
  lastCellMv = (uint16_t)(v * 1000.0f + 0.5f);
  uint8_t pct = lipoPercent(v);
  Serial.printf("battery: cell_mV=%u (%.3f V)  %u%%  v100=%u  divider: %s\n",
                lastCellMv, v, pct, (unsigned)(v * 10.0f + 0.5f), dividerVerdict(v));
  return pct;
}
#endif

/* ── pushState(): the report burst, shaped EXACTLY like the working original ──
 * a device of the same class that already works's pushState(), lines 164-171:
 *
 *     zbPresence.setBinaryInput(gPresent);
 *     zbPresence.reportBinaryInput();
 *     zbPresence.setBatteryPercentage(gBatteryPct);     <-- battery INSIDE the burst
 *     zbPresence.reportBatteryPercentage();
 *     zbFault.setBinaryInput(gFault);
 *     zbFault.reportBinaryInput();
 *
 * The battery is reported IMMEDIATELY AFTER the EP1 binary report, in the same
 * burst, on the same endpoint -- NOT later in isolation. That ordering is the
 * one structural difference that survived every earlier fix, and it is the
 * difference between a build that works and one that aborts at
 * esp_zigbee_zcl_command.c:263.
 * ────────────────────────────────────────────────────────────────────────── */
/* Ask the stack whether we actually have a network address. `joined` used to be
 * hardcoded true (bench build) to stop setup hanging -- which silently defeated both
 * the report guard and the LED, so the device reported into the void and blinked
 * "joined" while doing it. */
static bool actuallyJoined(void) {
  uint16_t sa = esp_zb_get_short_address();
  bool ok = (sa != 0xFFFF && sa != 0xFFFE);
  Serial.printf("network short address = 0x%04X  ->  %s\n", sa,
                ok ? "JOINED (reports will route)" : "NOT JOINED (reports go nowhere)");
  return ok;
}

/* ── THE DEVICE FIXES ITS OWN REPORTING RECORD ────────────────────────────────
 * At every boot the stack builds a DEFAULT reporting
 * record for the battery attribute with the destination UNSET:
 *
 *     min=5  max=0  delta=0  dst=0x0000/0  flags=0x51     <- reports go nowhere
 *
 * The library's own setXxxReporting() has the same gap: it sets dst.profile_id
 * and manuf_code but NEVER dst.endpoint or dst.short_addr.
 *
 * The coordinator CAN fix it (z2m's configureReporting produced
 * min=0 max=65000 delta=1 dst=0x0000/1, and reports flowed immediately) -- but
 * THIS DEVICE SLEEPS, so it cannot be reached to configure:
 *
 *     device/configure -> "Delivery failed for '6769'"
 *
 * and the record is rebuilt from scratch on every wake anyway. So the device has
 * to correct its own record. It can: esp_zb_zcl_update_reporting_info() only
 * needs a record to exist, and one always does.
 *
 * dst endpoint 1 = the coordinator's ZCL endpoint, which is where zigbee2mqtt
 * listens. dst 0 is why a day of reports vanished with nothing logged anywhere.
 * ──────────────────────────────────────────────────────────────────────────── */
static bool fixBatteryReportingRecord(void) {
  esp_zb_zcl_attr_location_info_t loc;
  memset(&loc, 0, sizeof(loc));
  loc.endpoint_id  = EP_PRESENT;
  loc.cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
  loc.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  loc.manuf_code   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  loc.attr_id      = AMI_ATTR_BATTERY_PERCENTAGE_REMAINING;

  if (!esp_zb_lock_acquire(portMAX_DELAY)) {
    Serial.println("  record fix: could not take the stack lock");
    return false;
  }
  esp_zb_zcl_reporting_info_t *ri = esp_zb_zcl_find_reporting_info(loc);
  bool ok = false;
  uint16_t before = 0xFFFF; uint8_t beforeEp = 0xFF; esp_err_t err = ESP_FAIL;
  if (ri != nullptr) {
    before   = ri->dst.short_addr;
    beforeEp = ri->dst.endpoint;
    ri->dst.short_addr = 0x0000;                   /* the coordinator */
    ri->dst.endpoint   = 1;                        /* where z2m listens */
    ri->dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    ri->u.send_info.min_interval     = 0;
    ri->u.send_info.max_interval     = 65000;
    ri->u.send_info.def_min_interval = 0;
    ri->u.send_info.def_max_interval = 65000;
    ri->u.send_info.delta.s32        = 1;
    err = esp_zb_zcl_update_reporting_info(ri);
    ok  = (err == ESP_OK);
  }
  esp_zb_lock_release();

  if (ri == nullptr) {
    Serial.println("  record fix: no record to fix (unexpected)");
    return false;
  }
  Serial.printf("  record fix: dst 0x%04X/%u -> 0x0000/1  err=0x%x (%s)  %s\n",
                before, beforeEp, (unsigned)err,
                esp_err_to_name(err), ok ? "OK" : "FAILED");
  return ok;
}

static void pushState(bool present, bool fault, uint8_t pct) {
  presentSensor.setBinaryInput(present);
  presentSensor.reportBinaryInput();
#if ENABLE_BATTERY_MEASUREMENT
  /* NEVER touch the battery attribute without a reporting record -- see the
   * guard above. A missing record is not worth crashing the device for. */
  if (batteryReportingPresent()) {
    presentSensor.setBatteryPercentage(pct);
    presentSensor.reportBatteryPercentage();
  }
#endif
  faultSensor.setBinaryInput(fault);
  faultSensor.reportBinaryInput();
}

/* The gate / bootstrap-switch / reportBattery machinery that lived here is GONE.
 * pushState() above reports the battery INSIDE the binary burst, exactly like the
 * working original (a device of the same class that already works pushState()), so there is nothing to gate
 * and nothing to bootstrap. measureBatteryPct() does the local measurement. */

static void sleepNow(void) {
  /* Hold XSHUT LOW so the lidar cannot float and idle at ~19 mA all sleep. */
  digitalWrite(PIN_XSHUT, LOW);
  gpio_hold_en(PIN_XSHUT);
#if ENABLE_BATTERY_MEASUREMENT
  digitalWrite(PIN_BAT_EN, LOW);
  gpio_hold_en(PIN_BAT_EN);
#endif
  /* NOTE (ESP32-C6, measured 2026-09-25): gpio_hold_en(pin) ALONE holds the pin
   * through deep sleep on this chip. The blanket gpio_deep_sleep_hold_en() is the
   * original-ESP32 idiom and is COMPILED OUT here, because the C6 sets
   * SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP — per-pin hold already applies in
   * deep sleep. Calling it will not compile ("not declared in this scope").
   * This is also why the 2026-09-14 latch worked with gpio_hold_en alone. */
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_SECS * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

/* ─────────────────────────────────────────────────────────────────────────
 * setup() runs on EVERY wake. loop() is never reached in the sleeping build.
 * ───────────────────────────────────────────────────────────────────────── */
void setup() {
  /* BEFORE anything touches the radio: adopt the original's identity. */
  esp_err_t macErr = esp_base_mac_addr_set(AMI_BASE_MAC);
  awakeStartMs = millis();

  /* 1. Prove we woke, BEFORE any radio work. One blink = "I woke".
   *    If you see one blink and nothing more, the radio never started. */
  pinMode(PIN_LED, OUTPUT);
  ledOff();
  blink(1);

  /* 2. Release any hold left from the previous sleep. Without this the pin
   *    stays latched and every sensor read fails — the 2026-09-14 bug. */
  /* Per-pin hold only — see the chip note in sleepNow(). The blanket
   * gpio_deep_sleep_hold_dis() does not exist on the C6. Releasing this hold is
   * MANDATORY: a leftover latch keeps XSHUT low no matter what loop() drives,
   * and every sensor read then fails (the 2026-09-14 bug). */
  gpio_hold_dis(PIN_XSHUT);
#if ENABLE_BATTERY_MEASUREMENT
  gpio_hold_dis(PIN_BAT_EN);
#endif
  pinMode(PIN_XSHUT, OUTPUT);

  /* 3. wakeCount: +1 per wake. Coming back as 1 after a long run means the
   *    POWER was lost (RTC memory does not survive power loss). */
  wakeCount++;

  esp_reset_reason_t resetReason = esp_reset_reason();

  Serial.begin(115200);
  delay(80);
  Serial.printf("\n[%s] wake #%lu  reset=%d\n", FW_BUILD,
                (unsigned long)wakeCount, (int)resetReason);

  /* 4. RADIO FIRST (silent-failure fix), endpoints registered before begin. */
#if ENABLE_BATTERY_MEASUREMENT
  /* Measure BEFORE registration: setPowerSource() wants a real initial value. */
  gInitialPct = lipoPercent(readBatteryVolts());
  Serial.printf("initial battery for registration: %u%%\n", gInitialPct);
#endif

  startRadioAndEndpoints();

  /* ── SELF-HEALING JOIN ────────────────────────────────────────────────────
 * Try a normal rejoin first (fast, cheap, the everyday case).
 *
 * If that fails -- the coordinator no longer honours our stored credentials --
 * the device must be able to recover by itself. It cannot just retry: the
 * library only starts NETWORK STEERING when the device is factory-new, so a
 * device holding stale credentials tries a rejoin forever and never searches.
 * Forgetting the network makes it factory-new, and then the library steers.
 *
 * A battery device that can lose power has to be able to find its network again. */
  /* ── SELF-HEALING JOIN ────────────────────────────────────────────────────
   * WHY THIS IS NEEDED, and why the original never had it:
   *
   * The Arduino Zigbee library triggers NETWORK STEERING only when the device is
   * factory-new:
   *
   *     if (esp_zb_bdb_is_factory_new()) {
   *         esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
   *     }
   *
   * A device that still holds credentials -- ours does -- tries a secured rejoin
   * and, if the coordinator refuses it, NEVER falls back to searching. It sits
   * NOT JOINED forever. That is exactly what we measured: a permit-join window
   * did nothing, and only erase_nvs (= factory-new = steal) ever worked.
   *
   * The original sensor never needed this because its credentials are valid and
   * it never loses its network. A device that can lose power must be able to
   * recover by itself, so: try the normal rejoin first (fast, cheap, the common
   * case), and only if that fails, forget everything and steer. */
  bool started = Zigbee.begin();
  if (!started) {
    Serial.println("Zigbee.begin() failed");
  }

  delay(REPORT_SETTLE_MS);
  if (started && !actuallyJoined()) {
    Serial.println("no network on stored credentials -> factory-reset + steer");
    if (Zigbee.begin(ZIGBEE_END_DEVICE, true)) {   /* factory-new => library steers */
      delay(REPORT_SETTLE_MS + 2000);              /* steering scans; give it longer */
    }
  }
  /* DO NOT GATE ON Zigbee.connected().
   * On core 3.3.11 the library's _connected flag is only set on certain BDB
   * signal paths and is NOT set for an end device that re-joins (SECURED_REJOIN)
   * -- even though the device IS on the network and the coordinator logs the
   * join. Gating on it made a healthy device look absent and skipped EVERY
   * report, which is most of why this debugging looked like radio failure.
   * reportBinaryInput() requires only Zigbee.started(), which begin() guarantees.
   * Same finding as a device of the same class that already works's own note. */
  /* Read the address BACK rather than trusting that the call took effect --
   * serial is the only way to know what z2m will actually see. */
  {
    uint8_t base[6] = {0};
    uint8_t ieee[8] = {0};
    esp_base_mac_addr_get(base);
    esp_read_mac(ieee, ESP_MAC_IEEE802154);   /* the address the Zigbee stack uses */
    Serial.printf("identity: target=%s (set rc=%d)\n", AMI_IEEE_STRING, (int)macErr);
    Serial.printf("          base MAC      = %02x:%02x:%02x:%02x:%02x:%02x\n",
                  base[0], base[1], base[2], base[3], base[4], base[5]);
    Serial.printf("          802.15.4 IEEE = 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                  ieee[7], ieee[6], ieee[5], ieee[4], ieee[3], ieee[2], ieee[1], ieee[0]);
    Serial.printf("          MATCH: %s\n",
                  /* esp_read_mac returns the address in interface order: ieee[0] is
                   * the FIRST octet (0x58), so it reads 58:e6:c5:ff:fe:1a:ab:b4. An
                   * earlier version compared ieee[7] first and printed a false
                   * "MATCH: NO" while the address had in fact taken correctly. */
                  (ieee[0] == 0x58 && ieee[1] == 0xe6 && ieee[2] == 0xc5 && ieee[3] == 0xff &&
                   ieee[4] == 0xfe && ieee[5] == 0x1a && ieee[6] == 0xab && ieee[7] == 0xb4)
                      ? "YES - z2m will see this as the original garage_sedan"
                      : "NO  - address did NOT take; do not pair yet");
  }

  /* ── the device configures its OWN reporting, every boot ───────────────
   * Must come AFTER Zigbee.begin(): setClusterReporting() bails with
   * "Zigbee stack not initialized" otherwise.
   * Values mirror the converter's configure(): min 0, max 65000, change 1. */
  batteryReportingPresent();

  bool joined = true;
  Serial.printf("radio started after %lu ms (NOT gating on connected())\n",
                (unsigned long)(millis() - awakeStartMs));

  /* 5. SENSOR SECOND — and a failure becomes a FAULT, never a hang. */
  bool sensorOk = initSensor();
  bool timeout = false;
  uint16_t d = readDistance(sensorOk, &timeout);

  /* 6. The band rule. Everything outside MIN_VALID_MM .. CAR_DISTANCE_MM
   *    is ABSENT, including 0 and the crosstalk phantom. */
  bool belowFloor = (d < MIN_VALID_MM);
  bool oor        = (d >= OOR_SENTINEL) || belowFloor;
  bool present    = (sensorOk && !timeout && !oor && d < CAR_DISTANCE_MM);
  bool fault      = (!sensorOk) || timeout;

  Serial.printf("d=%u  below_floor=%d  present=%d  fault=%d  sensor_ok=%d\n",
                d, belowFloor ? 1 : 0, present ? 1 : 0, fault ? 1 : 0, sensorOk ? 1 : 0);

  Serial.printf("awake=%lu ms  wake=%lu\n",
                (unsigned long)(millis() - awakeStartMs), (unsigned long)wakeCount);

#if ENABLE_BATTERY_MEASUREMENT
  /* Printed BEFORE we take a fresh reading, so on USB it still shows what the
   * cell was on the last BATTERY wake. */
  if (lastCellMv)
    Serial.printf("last_cell_mV=%u  (%.3f V, from the previous wake)\n",
                  lastCellMv, lastCellMv / 1000.0f);

  /* ── THE DRAIN EVIDENCE ───────────────────────────────────────────────────
   * On battery there is NO serial, so anything not stored in RTC memory is lost
   * for the whole run. awake_ms is what the drain figure is derived from:
   *     drain/h ~= (awake_ms/3600)*I_active + (1 - awake_ms/3600000)*I_sleep
   * It was being saved to lastAwakeMs and never read back -- the comment claimed
   * "EP5 publishes it next wake", but EP5 was removed, so the number was
   * unreachable on a battery run. Reprint it at boot instead, so the procedure is:
   *     run on battery  ->  plug USB in  ->  read the first lines.
   * RTC memory survives deep sleep AND a reset, so plugging USB in does not
   * lose it. Same trick as last_cell_mV. */
  if (lastAwakeMs)
    Serial.printf("last_awake_ms=%lu  (from the previous wake - use this for the drain)\n",
                  (unsigned long)lastAwakeMs);
  else
    Serial.println("last_cell_mV=none yet (first wake since power-on)");
#endif

  /* 7. Report EVERY wake, regardless of change. wakeCount guarantees the
   *    payload differs every time, so nothing upstream can dedup it away. */
  /* Ask whether we are actually on the network (the settle already happened in
   * the join block above, including the longer one after a steering attempt). */
  joined = actuallyJoined();

  /* ── ORDER MATTERS ────────────────────────────────────────────────────────
   * Correct the reporting record HERE -- AFTER the join, not before.
   *
   * Measured: sleep14 (no self-fix) JOINED fine; bench build (self-fix called BEFORE
   * the join) came up NOT JOINED. The only difference was this call. Writing to
   * the Zigbee stack's reporting storage before the device is on a network
   * appears to disturb the join.
   *
   * So: join first, then correct the record, then report. */
  if (joined) fixBatteryReportingRecord();

  if (joined) {
#if ENABLE_BATTERY_MEASUREMENT
    uint8_t pct = measureBatteryPct();
#else
    uint8_t pct = 0;
#endif
    pushState(present, fault, pct);      /* the proven burst shape */
    delay(REPORT_FLUSH_MS);              /* let the burst leave before we sleep */
  }

  /* 8. Awake duration: THE number that decides the drain. Measured to HERE, so it
   *    includes the report and its settle time -- the expensive part. */
  uint32_t awakeMs = millis() - awakeStartMs;
  Serial.printf("awake_ms=%lu (will be published next wake)\n", (unsigned long)awakeMs);

  /* 9. LED: 2 blinks = woke AND reported. 3 blinks = woke but no network. */
  blink(joined ? 2 : 3);

#if DEEP_SLEEP
  lastAwakeMs = awakeMs;          /* survives sleep; reprinted at the next boot */
  sleepNow();
#else
  Serial.println("BENCH build: staying awake (no sleep). loop() runs.");
#endif
}

void loop() {
#if DEEP_SLEEP
  /* Never reached: setup() ends in esp_deep_sleep_start(). */
  delay(1000);
#else
  /* BENCH only: keep sampling so a bench session can watch live and keep
   * the radio up for z2m re-interview. */
  delay(30000);
  bool timeout = false;
  bool sensorOk = initSensor();
  uint16_t d = readDistance(sensorOk, &timeout);
  bool belowFloor = (d < MIN_VALID_MM);
  bool oor = (d >= OOR_SENTINEL) || belowFloor;
  bool present = (sensorOk && !timeout && !oor && d < CAR_DISTANCE_MM);
  bool fault = (!sensorOk) || timeout;

  /* Heartbeat. With NO serial this is the only way to see liveness from outside
   * the case: 1 blink every 30 s = running; a rapid run of blinks = rebooting;
   * nothing at all = dead. */
  blink(1, 60, 0);

  batteryReportingPresent();
  Serial.printf("[BENCH] d=%u present=%d fault=%d wake=%lu\n",
                d, present ? 1 : 0, fault ? 1 : 0, (unsigned long)wakeCount);

#if ENABLE_BATTERY_MEASUREMENT
  uint8_t pctLoop = measureBatteryPct();
#else
  uint8_t pctLoop = 0;
#endif
  pushState(present, fault, pctLoop);
#endif
}
