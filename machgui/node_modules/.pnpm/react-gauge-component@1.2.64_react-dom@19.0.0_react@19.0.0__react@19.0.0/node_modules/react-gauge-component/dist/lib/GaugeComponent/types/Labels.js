"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.defaultLabels = exports.defaultValueLabel = void 0;
var Tick_1 = require("./Tick");
exports.defaultValueLabel = {
    formatTextValue: undefined,
    matchColorWithArc: false,
    maxDecimalDigits: 2,
    style: {
        fontSize: "35px",
        fill: '#fff',
        textShadow: "black 1px 0.5px 0px, black 0px 0px 0.03em, black 0px 0px 0.01em"
    },
    hide: false
};
exports.defaultLabels = {
    valueLabel: exports.defaultValueLabel,
    tickLabels: Tick_1.defaultTickLabels
};
