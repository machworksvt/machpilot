"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.getGaugeMarginByType = exports.defaultGaugeProps = exports.GaugeType = void 0;
var Arc_1 = require("./Arc");
var Labels_1 = require("./Labels");
var Pointer_1 = require("./Pointer");
var GaugeType;
(function (GaugeType) {
    GaugeType["Semicircle"] = "semicircle";
    GaugeType["Radial"] = "radial";
    GaugeType["Grafana"] = "grafana";
})(GaugeType || (exports.GaugeType = GaugeType = {}));
exports.defaultGaugeProps = {
    id: "",
    className: "gauge-component-class",
    style: { width: "100%" },
    marginInPercent: 0.07,
    value: 33,
    minValue: 0,
    maxValue: 100,
    arc: Arc_1.defaultArc,
    labels: Labels_1.defaultLabels,
    pointer: Pointer_1.defaultPointer,
    type: GaugeType.Grafana
};
var getGaugeMarginByType = function (type) {
    var _a;
    var gaugeTypesMargin = (_a = {},
        _a[GaugeType.Grafana] = { top: 0.12, bottom: 0.00, left: 0.07, right: 0.07 },
        _a[GaugeType.Semicircle] = { top: 0.08, bottom: 0.00, left: 0.08, right: 0.08 },
        _a[GaugeType.Radial] = { top: 0.07, bottom: 0.00, left: 0.07, right: 0.07 },
        _a);
    return gaugeTypesMargin[type];
};
exports.getGaugeMarginByType = getGaugeMarginByType;
