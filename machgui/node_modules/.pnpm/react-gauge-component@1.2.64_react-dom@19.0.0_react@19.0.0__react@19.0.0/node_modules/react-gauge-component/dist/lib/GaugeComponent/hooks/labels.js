"use strict";
var __assign = (this && this.__assign) || function () {
    __assign = Object.assign || function(t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
            s = arguments[i];
            for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p))
                t[p] = s[p];
        }
        return t;
    };
    return __assign.apply(this, arguments);
};
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.calculateAnchorAndAngleByValue = exports.clearTicks = exports.clearValueLabel = exports.addValueText = exports.addText = exports.getLabelCoordsByValue = exports.addTick = exports.addTickValue = exports.addTickLine = exports.mapTick = exports.addArcTicks = exports.setupTicks = exports.setupValueLabel = exports.setupLabels = void 0;
var utils = __importStar(require("./utils"));
var constants_1 = __importDefault(require("../constants"));
var Tick_1 = require("../types/Tick");
var d3 = __importStar(require("d3"));
var GaugeComponentProps_1 = require("../types/GaugeComponentProps");
var arc_1 = require("./arc");
var setupLabels = function (gauge) {
    (0, exports.setupValueLabel)(gauge);
    (0, exports.setupTicks)(gauge);
};
exports.setupLabels = setupLabels;
var setupValueLabel = function (gauge) {
    var _a;
    var labels = gauge.props.labels;
    if (!((_a = labels === null || labels === void 0 ? void 0 : labels.valueLabel) === null || _a === void 0 ? void 0 : _a.hide))
        (0, exports.addValueText)(gauge);
};
exports.setupValueLabel = setupValueLabel;
var setupTicks = function (gauge) {
    var _a, _b, _c, _d, _e, _f, _g, _h, _j;
    var labels = gauge.props.labels;
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    if (constants_1.default.debugTicksRadius) {
        for (var index = 0; index < maxValue; index++) {
            var indexTick = (0, exports.mapTick)(index, gauge);
            (0, exports.addTick)(indexTick, gauge);
        }
    }
    else if (!((_a = labels.tickLabels) === null || _a === void 0 ? void 0 : _a.hideMinMax)) {
        var alreadyHaveMinValueTick = (_c = (_b = labels.tickLabels) === null || _b === void 0 ? void 0 : _b.ticks) === null || _c === void 0 ? void 0 : _c.some(function (tick) { return tick.value == minValue; });
        if (!alreadyHaveMinValueTick) {
            //Add min value tick
            var minValueTick = (0, exports.mapTick)(minValue, gauge);
            (0, exports.addTick)(minValueTick, gauge);
        }
        var alreadyHaveMaxValueTick = (_e = (_d = labels.tickLabels) === null || _d === void 0 ? void 0 : _d.ticks) === null || _e === void 0 ? void 0 : _e.some(function (tick) { return tick.value == maxValue; });
        if (!alreadyHaveMaxValueTick) {
            // //Add max value tick
            var maxValueTick = (0, exports.mapTick)(maxValue, gauge);
            (0, exports.addTick)(maxValueTick, gauge);
        }
    }
    if (((_g = (_f = labels.tickLabels) === null || _f === void 0 ? void 0 : _f.ticks) === null || _g === void 0 ? void 0 : _g.length) > 0) {
        (_j = (_h = labels.tickLabels) === null || _h === void 0 ? void 0 : _h.ticks) === null || _j === void 0 ? void 0 : _j.forEach(function (tick) {
            (0, exports.addTick)(tick, gauge);
        });
    }
    (0, exports.addArcTicks)(gauge);
};
exports.setupTicks = setupTicks;
var addArcTicks = function (gauge) {
    var _a;
    (_a = gauge.arcData.current) === null || _a === void 0 ? void 0 : _a.map(function (subArc) {
        if (subArc.showTick)
            return subArc.limit;
    }).forEach(function (tickValue) {
        if (tickValue)
            (0, exports.addTick)((0, exports.mapTick)(tickValue, gauge), gauge);
    });
};
exports.addArcTicks = addArcTicks;
var mapTick = function (value, gauge) {
    var tickLabels = gauge.props.labels.tickLabels;
    return {
        value: value,
        valueConfig: tickLabels === null || tickLabels === void 0 ? void 0 : tickLabels.defaultTickValueConfig,
        lineConfig: tickLabels === null || tickLabels === void 0 ? void 0 : tickLabels.defaultTickLineConfig
    };
};
exports.mapTick = mapTick;
var addTickLine = function (tick, gauge) {
    var _a, _b, _c, _d, _e, _f, _g, _h, _j, _k, _l, _m, _o, _p, _q, _r, _s, _t;
    var labels = gauge.props.labels;
    var _u = (0, exports.calculateAnchorAndAngleByValue)(tick === null || tick === void 0 ? void 0 : tick.value, gauge), tickAnchor = _u.tickAnchor, angle = _u.angle;
    var tickDistanceFromArc = ((_a = tick.lineConfig) === null || _a === void 0 ? void 0 : _a.distanceFromArc) || ((_c = (_b = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _b === void 0 ? void 0 : _b.defaultTickLineConfig) === null || _c === void 0 ? void 0 : _c.distanceFromArc) || 0;
    if (((_e = (_d = gauge.props.labels) === null || _d === void 0 ? void 0 : _d.tickLabels) === null || _e === void 0 ? void 0 : _e.type) == "outer")
        tickDistanceFromArc = -tickDistanceFromArc;
    // else tickDistanceFromArc = tickDistanceFromArc - 10;
    var coords = (0, exports.getLabelCoordsByValue)(tick === null || tick === void 0 ? void 0 : tick.value, gauge, tickDistanceFromArc);
    var tickColor = ((_f = tick.lineConfig) === null || _f === void 0 ? void 0 : _f.color) || ((_h = (_g = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _g === void 0 ? void 0 : _g.defaultTickLineConfig) === null || _h === void 0 ? void 0 : _h.color) || ((_j = Tick_1.defaultTickLabels.defaultTickLineConfig) === null || _j === void 0 ? void 0 : _j.color);
    var tickWidth = ((_k = tick.lineConfig) === null || _k === void 0 ? void 0 : _k.width) || ((_m = (_l = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _l === void 0 ? void 0 : _l.defaultTickLineConfig) === null || _m === void 0 ? void 0 : _m.width) || ((_o = Tick_1.defaultTickLabels.defaultTickLineConfig) === null || _o === void 0 ? void 0 : _o.width);
    var tickLength = ((_p = tick.lineConfig) === null || _p === void 0 ? void 0 : _p.length) || ((_r = (_q = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _q === void 0 ? void 0 : _q.defaultTickLineConfig) === null || _r === void 0 ? void 0 : _r.length) || ((_s = Tick_1.defaultTickLabels.defaultTickLineConfig) === null || _s === void 0 ? void 0 : _s.length);
    // Calculate the end coordinates based on the adjusted position
    var endX;
    var endY;
    // When inner should draw from outside to inside
    // When outer should draw from inside to outside
    if (((_t = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _t === void 0 ? void 0 : _t.type) == "inner") {
        endX = coords.x + tickLength * Math.cos((angle * Math.PI) / 180);
        endY = coords.y + tickLength * Math.sin((angle * Math.PI) / 180);
    }
    else {
        endX = coords.x - tickLength * Math.cos((angle * Math.PI) / 180);
        endY = coords.y - tickLength * Math.sin((angle * Math.PI) / 180);
    }
    // (gauge.dimensions.current.outerRadius - gauge.dimensions.current.innerRadius)
    // Create a D3 line generator
    var lineGenerator = d3.line();
    var lineCoordinates;
    // Define the line coordinates
    lineCoordinates = [[coords.x, coords.y], [endX, endY]];
    // Append a path element for the line
    gauge.g.current
        .append("path")
        .datum(lineCoordinates)
        .attr("class", constants_1.default.tickLineClassname)
        .attr("d", lineGenerator)
        // .attr("transform", `translate(${0}, ${0})`)
        .attr("stroke", tickColor)
        .attr("stroke-width", tickWidth)
        .attr("fill", "none");
    // .attr("stroke-linecap", "round")
    // .attr("stroke-linejoin", "round")
    // .attr("transform", `rotate(${angle})`);
};
exports.addTickLine = addTickLine;
var addTickValue = function (tick, gauge) {
    var _a, _b, _c, _d, _e, _f, _g, _h, _j, _k, _l, _m, _o, _p, _q, _r, _s;
    var labels = gauge.props.labels;
    var arc = gauge.props.arc;
    var arcWidth = arc.width;
    var tickValue = tick === null || tick === void 0 ? void 0 : tick.value;
    var tickAnchor = (0, exports.calculateAnchorAndAngleByValue)(tickValue, gauge).tickAnchor;
    var centerToArcLengthSubtract = 27 - arcWidth * 10;
    var isInner = ((_a = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _a === void 0 ? void 0 : _a.type) == "inner";
    if (!isInner)
        centerToArcLengthSubtract = arcWidth * 10 - 10;
    else
        centerToArcLengthSubtract -= 10;
    var tickDistanceFromArc = ((_b = tick.lineConfig) === null || _b === void 0 ? void 0 : _b.distanceFromArc) || ((_d = (_c = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _c === void 0 ? void 0 : _c.defaultTickLineConfig) === null || _d === void 0 ? void 0 : _d.distanceFromArc) || 0;
    var tickLength = ((_e = tick.lineConfig) === null || _e === void 0 ? void 0 : _e.length) || ((_g = (_f = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _f === void 0 ? void 0 : _f.defaultTickLineConfig) === null || _g === void 0 ? void 0 : _g.length) || 0;
    var _shouldHideTickLine = shouldHideTickLine(tick, gauge);
    if (!_shouldHideTickLine) {
        if (isInner) {
            centerToArcLengthSubtract += tickDistanceFromArc;
            centerToArcLengthSubtract += tickLength;
        }
        else {
            centerToArcLengthSubtract -= tickDistanceFromArc;
            centerToArcLengthSubtract -= tickLength;
        }
    }
    var coords = (0, exports.getLabelCoordsByValue)(tickValue, gauge, centerToArcLengthSubtract);
    var tickValueStyle = ((_h = tick.valueConfig) === null || _h === void 0 ? void 0 : _h.style) || (((_k = (_j = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _j === void 0 ? void 0 : _j.defaultTickValueConfig) === null || _k === void 0 ? void 0 : _k.style) || {});
    tickValueStyle = __assign({}, tickValueStyle);
    var text = '';
    var maxDecimalDigits = ((_l = tick.valueConfig) === null || _l === void 0 ? void 0 : _l.maxDecimalDigits) || ((_o = (_m = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _m === void 0 ? void 0 : _m.defaultTickValueConfig) === null || _o === void 0 ? void 0 : _o.maxDecimalDigits);
    if ((_p = tick.valueConfig) === null || _p === void 0 ? void 0 : _p.formatTextValue) {
        text = tick.valueConfig.formatTextValue(utils.floatingNumber(tickValue, maxDecimalDigits));
    }
    else if ((_r = (_q = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _q === void 0 ? void 0 : _q.defaultTickValueConfig) === null || _r === void 0 ? void 0 : _r.formatTextValue) {
        text = labels.tickLabels.defaultTickValueConfig.formatTextValue(utils.floatingNumber(tickValue, maxDecimalDigits));
    }
    else if (gauge.props.minValue === 0 && gauge.props.maxValue === 100) {
        text = utils.floatingNumber(tickValue, maxDecimalDigits).toString();
        text += "%";
    }
    else {
        text = utils.floatingNumber(tickValue, maxDecimalDigits).toString();
    }
    if (((_s = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _s === void 0 ? void 0 : _s.type) == "inner") {
        if (tickAnchor === "end")
            coords.x += 10;
        if (tickAnchor === "start")
            coords.x -= 10;
        // if (tickAnchor === "middle") coords.y -= 0;
    }
    else {
        // if(tickAnchor === "end") coords.x -= 10;
        // if(tickAnchor === "start") coords.x += 10;
        if (tickAnchor === "middle")
            coords.y += 2;
    }
    if (tickAnchor === "middle") {
        coords.y += 0;
    }
    else {
        coords.y += 3;
    }
    tickValueStyle.textAnchor = tickAnchor;
    (0, exports.addText)(text, coords.x, coords.y, gauge, tickValueStyle, constants_1.default.tickValueClassname);
};
exports.addTickValue = addTickValue;
var addTick = function (tick, gauge) {
    var labels = gauge.props.labels;
    //Make validation for sequence of values respecting DEFAULT -> DEFAULT FROM USER -> SPECIFIC TICK VALUE
    var _shouldHideTickLine = shouldHideTickLine(tick, gauge);
    var _shouldHideTickValue = shouldHideTickValue(tick, gauge);
    if (!_shouldHideTickLine)
        (0, exports.addTickLine)(tick, gauge);
    if (!constants_1.default.debugTicksRadius && !_shouldHideTickValue) {
        (0, exports.addTickValue)(tick, gauge);
    }
};
exports.addTick = addTick;
var getLabelCoordsByValue = function (value, gauge, centerToArcLengthSubtract) {
    var _a;
    if (centerToArcLengthSubtract === void 0) { centerToArcLengthSubtract = 0; }
    var labels = gauge.props.labels;
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    var type = (_a = labels.tickLabels) === null || _a === void 0 ? void 0 : _a.type;
    var _b = (0, arc_1.getCoordByValue)(value, gauge, type, centerToArcLengthSubtract, 0.93), x = _b.x, y = _b.y;
    var percent = utils.calculatePercentage(minValue, maxValue, value);
    //This corrects labels in the cener being too close from the arc
    // let isValueBetweenCenter = percent > CONSTANTS.rangeBetweenCenteredTickValueLabel[0] && 
    //                               percent < CONSTANTS.rangeBetweenCenteredTickValueLabel[1];
    // if (isValueBetweenCenter){
    //   let isInner = type == "inner";
    //   y+= isInner ? 8 : -1;
    // }
    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Radial) {
        y += 3;
    }
    return { x: x, y: y };
};
exports.getLabelCoordsByValue = getLabelCoordsByValue;
var addText = function (html, x, y, gauge, style, className, rotate) {
    if (rotate === void 0) { rotate = 0; }
    var div = gauge.g.current
        .append("g")
        .attr("class", className)
        .attr("transform", "translate(".concat(x, ", ").concat(y, ")"))
        .append("text")
        .text(html); // use html() instead of text()
    applyTextStyles(div, style);
    div.attr("transform", "rotate(".concat(rotate, ")"));
};
exports.addText = addText;
var applyTextStyles = function (div, style) {
    //Apply default styles
    Object.entries(style).forEach(function (_a) {
        var key = _a[0], value = _a[1];
        return div.style(utils.camelCaseToKebabCase(key), value);
    });
    //Apply custom styles
    if (style != undefined)
        Object.entries(style).forEach(function (_a) {
            var key = _a[0], value = _a[1];
            return div.style(utils.camelCaseToKebabCase(key), value);
        });
};
//Adds text undeneath the graft to display which percentage is the current one
var addValueText = function (gauge) {
    var _a, _b, _c;
    var labels = gauge.props.labels;
    var value = gauge.props.value;
    var valueLabel = labels === null || labels === void 0 ? void 0 : labels.valueLabel;
    var textPadding = 20;
    var text = '';
    var maxDecimalDigits = (_a = labels === null || labels === void 0 ? void 0 : labels.valueLabel) === null || _a === void 0 ? void 0 : _a.maxDecimalDigits;
    var floatValue = utils.floatingNumber(value, maxDecimalDigits);
    if (valueLabel.formatTextValue) {
        text = valueLabel.formatTextValue(floatValue);
    }
    else if (gauge.props.minValue === 0 && gauge.props.maxValue === 100) {
        text = floatValue.toString();
        text += "%";
    }
    else {
        text = floatValue.toString();
    }
    var maxLengthBeforeComputation = 4;
    var textLength = (text === null || text === void 0 ? void 0 : text.length) || 0;
    var fontRatio = textLength > maxLengthBeforeComputation ? maxLengthBeforeComputation / textLength * 1.5 : 1; // Compute the font size ratio
    var valueFontSize = (_b = valueLabel === null || valueLabel === void 0 ? void 0 : valueLabel.style) === null || _b === void 0 ? void 0 : _b.fontSize;
    var valueTextStyle = __assign({}, valueLabel.style);
    var x = gauge.dimensions.current.outerRadius;
    var y = 0;
    valueTextStyle.textAnchor = "middle";
    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Semicircle) {
        y = gauge.dimensions.current.outerRadius / 1.5 + textPadding;
    }
    else if (gauge.props.type == GaugeComponentProps_1.GaugeType.Radial) {
        y = gauge.dimensions.current.outerRadius * 1.45 + textPadding;
    }
    else if (gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana) {
        y = gauge.dimensions.current.outerRadius * 1.0 + textPadding;
    }
    //if(gauge.props.pointer.type == PointerType.Arrow){
    //  y = gauge.dimensions.current.outerRadius * 0.79 + textPadding;
    //}
    var widthFactor = gauge.props.type == GaugeComponentProps_1.GaugeType.Radial ? 0.003 : 0.003;
    fontRatio = gauge.dimensions.current.width * widthFactor * fontRatio;
    var fontSizeNumber = parseInt(valueFontSize, 10) * fontRatio;
    valueTextStyle.fontSize = fontSizeNumber + "px";
    if (valueLabel.matchColorWithArc)
        valueTextStyle.fill = ((_c = (0, arc_1.getArcDataByValue)(value, gauge)) === null || _c === void 0 ? void 0 : _c.color) || "white";
    (0, exports.addText)(text, x, y, gauge, valueTextStyle, constants_1.default.valueLabelClassname);
};
exports.addValueText = addValueText;
var clearValueLabel = function (gauge) { return gauge.g.current.selectAll(".".concat(constants_1.default.valueLabelClassname)).remove(); };
exports.clearValueLabel = clearValueLabel;
var clearTicks = function (gauge) {
    gauge.g.current.selectAll(".".concat(constants_1.default.tickLineClassname)).remove();
    gauge.g.current.selectAll(".".concat(constants_1.default.tickValueClassname)).remove();
};
exports.clearTicks = clearTicks;
var calculateAnchorAndAngleByValue = function (value, gauge) {
    var _a;
    var _b;
    var labels = gauge.props.labels;
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    var valuePercentage = utils.calculatePercentage(minValue, maxValue, value);
    var gaugeTypesAngles = (_a = {},
        _a[GaugeComponentProps_1.GaugeType.Grafana] = {
            startAngle: -20,
            endAngle: 220
        },
        _a[GaugeComponentProps_1.GaugeType.Semicircle] = {
            startAngle: 0,
            endAngle: 180
        },
        _a[GaugeComponentProps_1.GaugeType.Radial] = {
            startAngle: -42,
            endAngle: 266
        },
        _a);
    var _c = gaugeTypesAngles[gauge.props.type], startAngle = _c.startAngle, endAngle = _c.endAngle;
    var angle = startAngle + (valuePercentage * 100) * endAngle / 100;
    var isValueLessThanHalf = valuePercentage < 0.5;
    //Values between 40% and 60% are aligned in the middle
    var isValueBetweenTolerance = valuePercentage > constants_1.default.rangeBetweenCenteredTickValueLabel[0] &&
        valuePercentage < constants_1.default.rangeBetweenCenteredTickValueLabel[1];
    var tickAnchor = '';
    var isInner = ((_b = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _b === void 0 ? void 0 : _b.type) == "inner";
    if (isValueBetweenTolerance) {
        tickAnchor = "middle";
    }
    else if (isValueLessThanHalf) {
        tickAnchor = isInner ? "start" : "end";
    }
    else {
        tickAnchor = isInner ? "end" : "start";
    }
    // if(valuePercentage > 0.50) angle = angle - 180;
    return { tickAnchor: tickAnchor, angle: angle };
};
exports.calculateAnchorAndAngleByValue = calculateAnchorAndAngleByValue;
var shouldHideTickLine = function (tick, gauge) {
    var _a, _b, _c, _d;
    var labels = gauge.props.labels;
    var defaultHideValue = (_a = Tick_1.defaultTickLabels.defaultTickLineConfig) === null || _a === void 0 ? void 0 : _a.hide;
    var shouldHide = defaultHideValue;
    var defaultHideLineFromUser = (_c = (_b = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _b === void 0 ? void 0 : _b.defaultTickLineConfig) === null || _c === void 0 ? void 0 : _c.hide;
    if (defaultHideLineFromUser != undefined) {
        shouldHide = defaultHideLineFromUser;
    }
    var specificHideValueFromUser = (_d = tick.lineConfig) === null || _d === void 0 ? void 0 : _d.hide;
    if (specificHideValueFromUser != undefined) {
        shouldHide = specificHideValueFromUser;
    }
    return shouldHide;
};
var shouldHideTickValue = function (tick, gauge) {
    var _a, _b, _c, _d;
    var labels = gauge.props.labels;
    var defaultHideValue = (_a = Tick_1.defaultTickLabels.defaultTickValueConfig) === null || _a === void 0 ? void 0 : _a.hide;
    var shouldHide = defaultHideValue;
    var defaultHideValueFromUser = (_c = (_b = labels === null || labels === void 0 ? void 0 : labels.tickLabels) === null || _b === void 0 ? void 0 : _b.defaultTickValueConfig) === null || _c === void 0 ? void 0 : _c.hide;
    if (defaultHideValueFromUser != undefined) {
        shouldHide = defaultHideValueFromUser;
    }
    var specificHideValueFromUser = (_d = tick.valueConfig) === null || _d === void 0 ? void 0 : _d.hide;
    if (specificHideValueFromUser != undefined) {
        shouldHide = specificHideValueFromUser;
    }
    return shouldHide;
};
