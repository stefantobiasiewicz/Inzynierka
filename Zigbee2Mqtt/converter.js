const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const extend = require('zigbee-herdsman-converters/lib/extend');
const constants = require('zigbee-herdsman-converters/lib/constants');
const e = exposes.presets;
const ea = exposes.access;

module.exports = [
    
    {
        zigbeeModel: ['Contact_v1'],
        model: 'ST_ContactSensor',
        vendor: 'Stefan Tobiasiewicz',
        description: 'Rechargeable Zigbee contact sensor',
        fromZigbee: [fz.ias_contact_alarm_1, fz.ias_enroll, fz.battery],
        toZigbee: [],
        exposes: [e.contact(), e.battery()],
        configure: async (device, coordinatorEndpoint, logger) => {
            const endpoint = device.getEndpoint(1);
            const bindClusters = ['genPowerCfg'];
            await reporting.bind(endpoint, coordinatorEndpoint, bindClusters);
            await reporting.batteryPercentageRemaining(endpoint, {min: 2, max: constants.repInterval.MINUTES_30, change: 0});  // nadpisane do debugu
        }
    },
    
    {
        zigbeeModel: ['PowerPlug'],
        model: 'ST_PowerPlug',
        vendor: 'Stefan Tobiasiewicz',
        description: 'Zigbee power switch',
        fromZigbee: [fz.on_off],
        toZigbee: [tz.on_off],
        exposes: [e.switch()]
    },
    {
        zigbeeModel: ['AirSensor'],
        model: 'ST_AirSensor',
        vendor: 'Stefan Tobiasiewicz',
        description: 'Temperature and humidyty sensor',
        fromZigbee: [fz.temperature, fz.pressure, fz.humidity, fz.ignore_basic_report], // <-- added here
        toZigbee: [],
        exposes: [e.temperature(), e.pressure(), e.humidity()],
        configure: async (device, coordinatorEndpoint, logger) => {
            const endpoint = device.getEndpoint(10);
            const bindClusters = ['msTemperatureMeasurement', 'msPressureMeasurement', 'msRelativeHumidity'];
            await reporting.bind(endpoint, coordinatorEndpoint, bindClusters);
            await reporting.temperature(endpoint, {min: 5, max: constants.repInterval.HOUR, change: 0} );
            await reporting.humidity(endpoint, {min: 5, max: constants.repInterval.HOUR, change: 0} );
            await reporting.pressure(endpoint, {min: 5, max: constants.repInterval.HOUR, change: 0} );
        }
    }
];