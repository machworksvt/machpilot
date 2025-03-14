"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.defaultArc = exports.getArcWidthByType = exports.defaultSubArcs = void 0;
var GaugeComponentProps_1 = require("./GaugeComponentProps");
exports.defaultSubArcs = [
    { limit: 33, color: "#5BE12C" },
    { limit: 66, color: "#F5CD19" },
    { color: "#EA4228" },
];
var getArcWidthByType = function (type) {
    var _a;
    var gaugeTypesWidth = (_a = {},
        _a[GaugeComponentProps_1.GaugeType.Grafana] = 0.25,
        _a[GaugeComponentProps_1.GaugeType.Semicircle] = 0.15,
        _a[GaugeComponentProps_1.GaugeType.Radial] = 0.2,
        _a);
    if (!type)
        type = GaugeComponentProps_1.defaultGaugeProps.type;
    return gaugeTypesWidth[type];
};
exports.getArcWidthByType = getArcWidthByType;
exports.defaultArc = {
    padding: 0.05,
    width: 0.25,
    cornerRadius: 7,
    nbSubArcs: undefined,
    emptyColor: "#5C5C5C",
    colorArray: undefined,
    subArcs: exports.defaultSubArcs,
    gradient: false
};
