"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.defaultTickLabels = void 0;
var defaultTickLineConfig = {
    color: "rgb(173 172 171)",
    length: 7,
    width: 1,
    distanceFromArc: 3,
    hide: false
};
var defaultTickValueConfig = {
    formatTextValue: undefined,
    maxDecimalDigits: 2,
    style: {
        fontSize: "10px",
        fill: "rgb(173 172 171)",
    },
    hide: false,
};
var defaultTickList = [];
exports.defaultTickLabels = {
    type: 'outer',
    hideMinMax: false,
    ticks: defaultTickList,
    defaultTickValueConfig: defaultTickValueConfig,
    defaultTickLineConfig: defaultTickLineConfig
};
