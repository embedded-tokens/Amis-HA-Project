// Ami garage sedan presence sensor (VL53L0X) — external converter
//
// ESP32-C6 (Seeed XIAO) running sedan_sensor_v2.ino, reporting over Zigbee
// as an end device with two Binary Input clusters:
//   endpoint 1 -> sedan_present (ON = car in garage)
//   endpoint 2 -> sensor_fault  (ON = lidar dead/disconnected)
//
// Why this exists: the Espressif Zigbee SDK caches the Binary Input cluster's
// "Description" attribute in zb_storage, so after a reflash from the old
// driveway firmware the endpoint-1 description stayed "Car Present" and Z2M's
// auto-generated definition exposed it as `car_present` instead of
// `sedan_present`. This converter pins the endpoints to deterministic names
// so the entity IDs are stable regardless of what the description string says.

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const ea = exposes.access;

const fzLocal = {
    sedan_binary_input: {
        cluster: 'genBinaryInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.presentValue === undefined) return;
            const result = {};
            const on = msg.data.presentValue === 1;
            if (msg.endpoint.ID === 1) {
                result.sedan_present_1 = on ? 'ON' : 'OFF';
            } else if (msg.endpoint.ID === 2) {
                result.sensor_fault_2 = on ? 'ON' : 'OFF';
            }
            return result;
        },
    },
};

const definition = {
    zigbeeModel: ['SedanSensorV2'],
    model: 'SedanSensorV2',
    vendor: 'Ami',
    description: 'Garage sedan presence sensor (VL53L0X)',
    fromZigbee: [fzLocal.sedan_binary_input, fz.battery],
    toZigbee: [],
    exposes: [
        e.binary('sedan_present', ea.STATE, 'ON', 'OFF').withEndpoint('1'),
        e.binary('sensor_fault', ea.STATE, 'ON', 'OFF').withEndpoint('2'),
        e.battery(),
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        // CRITICAL: the firmware reports with ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT
        // addressing, which routes through the device's binding table. Without a
        // binding to the coordinator, every report is silently dropped. Bind both
        // Binary Input endpoints to the coordinator here, then enable reporting.
        const ep1 = device.getEndpoint(1);
        await ep1.bind('genBinaryInput', coordinatorEndpoint);
        await ep1.configureReporting('genBinaryInput', [
            {
                attribute: 'presentValue',
                minimumReportInterval: 0,
                maximumReportInterval: 65000,
                reportableChange: 1,
            },
        ]);
        const ep2 = device.getEndpoint(2);
        await ep2.bind('genBinaryInput', coordinatorEndpoint);
        await ep2.configureReporting('genBinaryInput', [
            {
                attribute: 'presentValue',
                minimumReportInterval: 0,
                maximumReportInterval: 65000,
                reportableChange: 1,
            },
        ]);
        // Battery: the firmware reports batteryPercentageRemaining on the
        // genPowerCfg cluster (endpoint 1) via the SAME ENDP_NOT_PRESENT
        // addressing, so it also routes through the device's binding table.
        // Bind genPowerCfg + enable its reporting or the battery % is dropped
        // exactly like the binary inputs were before this configure() existed.
        await ep1.bind('genPowerCfg', coordinatorEndpoint);
        await ep1.configureReporting('genPowerCfg', [
            {
                attribute: 'batteryPercentageRemaining',
                minimumReportInterval: 0,
                maximumReportInterval: 65000,
                reportableChange: 1,
            },
        ]);
    },
};

module.exports = definition;
