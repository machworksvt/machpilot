"use strict";
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
exports.validateArcs = exports.clearOuterArcs = exports.clearArcs = exports.redrawArcs = exports.getCoordByValue = exports.createGradientElement = exports.getColors = exports.applyGradientColors = exports.getArcDataByPercentage = exports.getArcDataByValue = exports.applyColors = exports.setupTooltip = exports.setupArcs = exports.drawArc = exports.setArcData = exports.hideTooltip = void 0;
var utils = __importStar(require("./utils"));
var d3_1 = require("d3");
var arcHooks = __importStar(require("./arc"));
var constants_1 = __importDefault(require("../constants"));
var Tooltip_1 = require("../types/Tooltip");
var GaugeComponentProps_1 = require("../types/GaugeComponentProps");
var lodash_1 = require("lodash");
var onArcMouseMove = function (event, d, gauge) {
    //event.target.style.stroke = "#ffffff5e";
    if (d.data.tooltip != undefined) {
        var shouldChangeText = d.data.tooltip.text != gauge.tooltip.current.text();
        if (shouldChangeText) {
            gauge.tooltip.current.html(d.data.tooltip.text)
                .style("position", "absolute")
                .style("display", "block")
                .style("opacity", 1);
            applyTooltipStyles(d.data.tooltip, d.data.color, gauge);
        }
        gauge.tooltip.current.style("left", (event.pageX + 15) + "px")
            .style("top", (event.pageY - 10) + "px");
    }
    if (d.data.onMouseMove != undefined)
        d.data.onMouseMove(event);
};
var applyTooltipStyles = function (tooltip, arcColor, gauge) {
    //Apply default styles
    Object.entries(Tooltip_1.defaultTooltipStyle).forEach(function (_a) {
        var key = _a[0], value = _a[1];
        return gauge.tooltip.current.style(utils.camelCaseToKebabCase(key), value);
    });
    gauge.tooltip.current.style("background-color", arcColor);
    //Apply custom styles
    if (tooltip.style != undefined)
        Object.entries(tooltip.style).forEach(function (_a) {
            var key = _a[0], value = _a[1];
            return gauge.tooltip.current.style(utils.camelCaseToKebabCase(key), value);
        });
};
var onArcMouseLeave = function (event, d, gauge, mousemoveCbThrottled) {
    mousemoveCbThrottled.cancel();
    (0, exports.hideTooltip)(gauge);
    if (d.data.onMouseLeave != undefined)
        d.data.onMouseLeave(event);
};
var hideTooltip = function (gauge) {
    gauge.tooltip.current.html(" ").style("display", "none");
};
exports.hideTooltip = hideTooltip;
var onArcMouseOut = function (event, d, gauge) {
    event.target.style.stroke = "none";
};
var onArcMouseClick = function (event, d) {
    if (d.data.onMouseClick != undefined)
        d.data.onMouseClick(event);
};
var setArcData = function (gauge) {
    var _a, _b;
    var arc = gauge.props.arc;
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    // Determine number of arcs to display
    var nbArcsToDisplay = (arc === null || arc === void 0 ? void 0 : arc.nbSubArcs) || (((_a = arc === null || arc === void 0 ? void 0 : arc.subArcs) === null || _a === void 0 ? void 0 : _a.length) || 1);
    var colorArray = (0, exports.getColors)(nbArcsToDisplay, gauge);
    if ((arc === null || arc === void 0 ? void 0 : arc.subArcs) && !(arc === null || arc === void 0 ? void 0 : arc.nbSubArcs)) {
        var lastSubArcLimit_1 = 0;
        var lastSubArcLimitPercentageAcc_1 = 0;
        var subArcsLength_1 = [];
        var subArcsLimits_1 = [];
        var subArcsTooltip_1 = [];
        (_b = arc === null || arc === void 0 ? void 0 : arc.subArcs) === null || _b === void 0 ? void 0 : _b.forEach(function (subArc, i) {
            var _a;
            var subArcLength = 0;
            //map limit for non defined subArcs limits
            var subArcRange = 0;
            var limit = subArc.limit;
            if (subArc.length != undefined) {
                subArcLength = subArc.length;
                limit = utils.getCurrentGaugeValueByPercentage(subArcLength + lastSubArcLimitPercentageAcc_1, gauge);
            }
            else if (subArc.limit == undefined) {
                subArcRange = lastSubArcLimit_1;
                var remainingPercentageEquallyDivided = undefined;
                var remainingSubArcs = (_a = arc === null || arc === void 0 ? void 0 : arc.subArcs) === null || _a === void 0 ? void 0 : _a.slice(i);
                var remainingPercentage = (1 - utils.calculatePercentage(minValue, maxValue, lastSubArcLimit_1)) * 100;
                if (!remainingPercentageEquallyDivided) {
                    remainingPercentageEquallyDivided = (remainingPercentage / Math.max((remainingSubArcs === null || remainingSubArcs === void 0 ? void 0 : remainingSubArcs.length) || 1, 1)) / 100;
                }
                limit = lastSubArcLimit_1 + (remainingPercentageEquallyDivided * 100);
                subArcLength = remainingPercentageEquallyDivided;
            }
            else {
                subArcRange = limit - lastSubArcLimit_1;
                // Calculate arc length based on previous arc percentage
                if (i !== 0) {
                    subArcLength = utils.calculatePercentage(minValue, maxValue, limit) - lastSubArcLimitPercentageAcc_1;
                }
                else {
                    subArcLength = utils.calculatePercentage(minValue, maxValue, subArcRange);
                }
            }
            subArcsLength_1.push(subArcLength);
            subArcsLimits_1.push(limit);
            lastSubArcLimitPercentageAcc_1 = subArcsLength_1.reduce(function (count, curr) { return count + curr; }, 0);
            lastSubArcLimit_1 = limit;
            if (subArc.tooltip != undefined)
                subArcsTooltip_1.push(subArc.tooltip);
        });
        var subArcs_1 = arc.subArcs;
        gauge.arcData.current = subArcsLength_1.map(function (length, i) { return ({
            value: length,
            limit: subArcsLimits_1[i],
            color: colorArray[i],
            showTick: subArcs_1[i].showTick || false,
            tooltip: subArcs_1[i].tooltip || undefined,
            onMouseMove: subArcs_1[i].onMouseMove,
            onMouseLeave: subArcs_1[i].onMouseLeave,
            onMouseClick: subArcs_1[i].onClick
        }); });
    }
    else {
        var arcValue_1 = maxValue / nbArcsToDisplay;
        gauge.arcData.current = Array.from({ length: nbArcsToDisplay }, function (_, i) { return ({
            value: arcValue_1,
            limit: (i + 1) * arcValue_1,
            color: colorArray[i],
            tooltip: undefined,
        }); });
    }
};
exports.setArcData = setArcData;
var getGrafanaMainArcData = function (gauge, percent) {
    var _a;
    if (percent === void 0) { percent = undefined; }
    var currentPercentage = percent != undefined ? percent : utils.calculatePercentage(gauge.props.minValue, gauge.props.maxValue, gauge.props.value);
    var curArcData = (0, exports.getArcDataByPercentage)(currentPercentage, gauge);
    var firstSubArc = {
        value: currentPercentage,
        //White indicate that no arc was found and work as an alert for debug
        color: (curArcData === null || curArcData === void 0 ? void 0 : curArcData.color) || "white",
        //disabled for now because onMouseOut is not working properly with the
        //high amount of renderings of this arc
        //tooltip: curArcData?.tooltip
    };
    //This is the grey arc that will be displayed when the gauge is not full
    var secondSubArc = {
        value: 1 - currentPercentage,
        color: (_a = gauge.props.arc) === null || _a === void 0 ? void 0 : _a.emptyColor,
    };
    return [firstSubArc, secondSubArc];
};
var drawGrafanaOuterArc = function (gauge, resize) {
    if (resize === void 0) { resize = false; }
    var outerRadius = gauge.dimensions.current.outerRadius;
    //Grafana's outer arc will be populates as the standard arc data would
    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana && resize) {
        gauge.doughnut.current.selectAll(".outerSubArc").remove();
        var outerArc = (0, d3_1.arc)()
            .outerRadius(outerRadius + 7)
            .innerRadius(outerRadius + 2)
            .cornerRadius(0)
            .padAngle(0);
        var arcPaths = gauge.doughnut.current
            .selectAll("anyString")
            .data(gauge.pieChart.current(gauge.arcData.current))
            .enter()
            .append("g")
            .attr("class", "outerSubArc");
        var outerArcSubarcs = arcPaths
            .append("path")
            .attr("d", outerArc);
        (0, exports.applyColors)(outerArcSubarcs, gauge);
        var mousemoveCbThrottled_1 = (0, lodash_1.throttle)(function (event, d) { return onArcMouseMove(event, d, gauge); }, 20);
        arcPaths
            .on("mouseleave", function (event, d) { return onArcMouseLeave(event, d, gauge, mousemoveCbThrottled_1); })
            .on("mouseout", function (event, d) { return onArcMouseOut(event, d, gauge); })
            .on("mousemove", mousemoveCbThrottled_1)
            .on("click", function (event, d) { return onArcMouseClick(event, d); });
    }
};
var drawArc = function (gauge, percent) {
    var _a, _b;
    if (percent === void 0) { percent = undefined; }
    var _c = gauge.props.arc, padding = _c.padding, cornerRadius = _c.cornerRadius;
    var _d = gauge.dimensions.current, innerRadius = _d.innerRadius, outerRadius = _d.outerRadius;
    // chartHooks.clearChart(gauge);
    var data = {};
    //When gradient enabled, it'll have only 1 arc
    if ((_b = (_a = gauge.props) === null || _a === void 0 ? void 0 : _a.arc) === null || _b === void 0 ? void 0 : _b.gradient) {
        data = [{ value: 1 }];
    }
    else {
        data = gauge.arcData.current;
    }
    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana) {
        data = getGrafanaMainArcData(gauge, percent);
    }
    var arcPadding = gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana ? 0 : padding;
    var arcCornerRadius = gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana ? 0 : cornerRadius;
    var arcObj = (0, d3_1.arc)()
        .outerRadius(outerRadius)
        .innerRadius(innerRadius)
        .cornerRadius(arcCornerRadius)
        .padAngle(arcPadding);
    var arcPaths = gauge.doughnut.current
        .selectAll("anyString")
        .data(gauge.pieChart.current(data))
        .enter()
        .append("g")
        .attr("class", "subArc");
    var subArcs = arcPaths
        .append("path")
        .attr("d", arcObj);
    (0, exports.applyColors)(subArcs, gauge);
    var mousemoveCbThrottled = (0, lodash_1.throttle)(function (event, d) { return onArcMouseMove(event, d, gauge); }, 20);
    arcPaths
        .on("mouseleave", function (event, d) { return onArcMouseLeave(event, d, gauge, mousemoveCbThrottled); })
        .on("mouseout", function (event, d) { return onArcMouseOut(event, d, gauge); })
        .on("mousemove", mousemoveCbThrottled)
        .on("click", function (event, d) { return onArcMouseClick(event, d); });
};
exports.drawArc = drawArc;
var setupArcs = function (gauge, resize) {
    if (resize === void 0) { resize = false; }
    //Setup the arc
    (0, exports.setupTooltip)(gauge);
    drawGrafanaOuterArc(gauge, resize);
    (0, exports.drawArc)(gauge);
};
exports.setupArcs = setupArcs;
var setupTooltip = function (gauge) {
    //Add tooltip
    var isTooltipInTheDom = document.getElementsByClassName(constants_1.default.arcTooltipClassname).length != 0;
    if (!isTooltipInTheDom)
        (0, d3_1.select)("body").append("div").attr("class", constants_1.default.arcTooltipClassname);
    gauge.tooltip.current = (0, d3_1.select)(".".concat(constants_1.default.arcTooltipClassname));
    gauge.tooltip.current
        .on("mouseleave", function () { return arcHooks.hideTooltip(gauge); })
        .on("mouseout", function () { return arcHooks.hideTooltip(gauge); });
};
exports.setupTooltip = setupTooltip;
var applyColors = function (subArcsPath, gauge) {
    var _a, _b;
    if ((_b = (_a = gauge.props) === null || _a === void 0 ? void 0 : _a.arc) === null || _b === void 0 ? void 0 : _b.gradient) {
        var uniqueId_1 = "subArc-linear-gradient-".concat(Math.random());
        var gradEl = (0, exports.createGradientElement)(gauge.doughnut.current, uniqueId_1);
        (0, exports.applyGradientColors)(gradEl, gauge);
        subArcsPath.style("fill", function (d) { return "url(#".concat(uniqueId_1, ")"); });
    }
    else {
        subArcsPath.style("fill", function (d) { return d.data.color; });
    }
};
exports.applyColors = applyColors;
var getArcDataByValue = function (value, gauge) {
    return gauge.arcData.current.find(function (subArcData) { return value <= subArcData.limit; });
};
exports.getArcDataByValue = getArcDataByValue;
var getArcDataByPercentage = function (percentage, gauge) {
    return (0, exports.getArcDataByValue)(utils.getCurrentGaugeValueByPercentage(percentage, gauge), gauge);
};
exports.getArcDataByPercentage = getArcDataByPercentage;
var applyGradientColors = function (gradEl, gauge) {
    gauge.arcData.current.forEach(function (subArcData) {
        var _a, _b, _c, _d;
        var normalizedOffset = utils.normalize(subArcData === null || subArcData === void 0 ? void 0 : subArcData.limit, (_b = (_a = gauge === null || gauge === void 0 ? void 0 : gauge.props) === null || _a === void 0 ? void 0 : _a.minValue) !== null && _b !== void 0 ? _b : 0, (_d = (_c = gauge === null || gauge === void 0 ? void 0 : gauge.props) === null || _c === void 0 ? void 0 : _c.maxValue) !== null && _d !== void 0 ? _d : 100);
        gradEl.append("stop")
            .attr("offset", "".concat(normalizedOffset, "%"))
            .style("stop-color", subArcData.color) //end in red
            .style("stop-opacity", 1);
    });
};
exports.applyGradientColors = applyGradientColors;
//Depending on the number of levels in the chart
//This function returns the same number of colors
var getColors = function (nbArcsToDisplay, gauge) {
    var _a;
    var arc = gauge.props.arc;
    var colorsValue = [];
    if (!arc.colorArray) {
        var subArcColors = (_a = arc.subArcs) === null || _a === void 0 ? void 0 : _a.map(function (subArc) { return subArc.color; });
        colorsValue = (subArcColors === null || subArcColors === void 0 ? void 0 : subArcColors.some(function (color) { return color != undefined; })) ? subArcColors : constants_1.default.defaultColors;
    }
    else {
        colorsValue = arc.colorArray;
    }
    //defaults colorsValue to white in order to avoid compilation error
    if (!colorsValue)
        colorsValue = ["#fff"];
    //Check if the number of colors equals the number of levels
    //Otherwise make an interpolation
    var arcsEqualsColorsLength = nbArcsToDisplay === (colorsValue === null || colorsValue === void 0 ? void 0 : colorsValue.length);
    if (arcsEqualsColorsLength)
        return colorsValue;
    var colorScale = (0, d3_1.scaleLinear)()
        .domain([1, nbArcsToDisplay])
        //@ts-ignore
        .range([colorsValue[0], colorsValue[colorsValue.length - 1]]) //Use the first and the last color as range
        //@ts-ignore
        .interpolate(d3_1.interpolateHsl);
    var colorArray = [];
    for (var i = 1; i <= nbArcsToDisplay; i++) {
        colorArray.push(colorScale(i));
    }
    return colorArray;
};
exports.getColors = getColors;
var createGradientElement = function (div, uniqueId) {
    //make defs and add the linear gradient
    var lg = div.append("defs").append("linearGradient")
        .attr("id", uniqueId) //id of the gradient
        .attr("x1", "0%")
        .attr("x2", "100%")
        .attr("y1", "0%")
        .attr("y2", "0%");
    return lg;
};
exports.createGradientElement = createGradientElement;
var getCoordByValue = function (value, gauge, position, centerToArcLengthSubtract, radiusFactor) {
    var _a;
    if (position === void 0) { position = "inner"; }
    if (centerToArcLengthSubtract === void 0) { centerToArcLengthSubtract = 0; }
    if (radiusFactor === void 0) { radiusFactor = 1; }
    var positionCenterToArcLength = {
        "outer": function () { return gauge.dimensions.current.outerRadius - centerToArcLengthSubtract + 2; },
        "inner": function () { return gauge.dimensions.current.innerRadius * radiusFactor - centerToArcLengthSubtract + 9; },
        "between": function () {
            var lengthBetweenOuterAndInner = (gauge.dimensions.current.outerRadius - gauge.dimensions.current.innerRadius);
            var middlePosition = gauge.dimensions.current.innerRadius + lengthBetweenOuterAndInner - 5;
            return middlePosition;
        }
    };
    var centerToArcLength = positionCenterToArcLength[position]();
    // This normalizes the labels when distanceFromArc = 0 to be just touching the arcs 
    if (gauge.props.type === GaugeComponentProps_1.GaugeType.Grafana) {
        centerToArcLength += 5;
    }
    else if (gauge.props.type === GaugeComponentProps_1.GaugeType.Semicircle) {
        centerToArcLength += -2;
    }
    var percent = utils.calculatePercentage(gauge.props.minValue, gauge.props.maxValue, value);
    var gaugeTypesAngles = (_a = {},
        _a[GaugeComponentProps_1.GaugeType.Grafana] = {
            startAngle: utils.degToRad(-23),
            endAngle: utils.degToRad(203)
        },
        _a[GaugeComponentProps_1.GaugeType.Semicircle] = {
            startAngle: utils.degToRad(0.9),
            endAngle: utils.degToRad(179.1)
        },
        _a[GaugeComponentProps_1.GaugeType.Radial] = {
            startAngle: utils.degToRad(-39),
            endAngle: utils.degToRad(219)
        },
        _a);
    var _b = gaugeTypesAngles[gauge.props.type], startAngle = _b.startAngle, endAngle = _b.endAngle;
    var angle = startAngle + (percent) * (endAngle - startAngle);
    var coordsRadius = 1 * (gauge.dimensions.current.width / 500);
    var coord = [0, -coordsRadius / 2];
    var coordMinusCenter = [
        coord[0] - centerToArcLength * Math.cos(angle),
        coord[1] - centerToArcLength * Math.sin(angle),
    ];
    var centerCoords = [gauge.dimensions.current.outerRadius, gauge.dimensions.current.outerRadius];
    var x = (centerCoords[0] + coordMinusCenter[0]);
    var y = (centerCoords[1] + coordMinusCenter[1]);
    return { x: x, y: y };
};
exports.getCoordByValue = getCoordByValue;
var redrawArcs = function (gauge) {
    (0, exports.clearArcs)(gauge);
    (0, exports.setArcData)(gauge);
    (0, exports.setupArcs)(gauge);
};
exports.redrawArcs = redrawArcs;
var clearArcs = function (gauge) {
    gauge.doughnut.current.selectAll(".subArc").remove();
};
exports.clearArcs = clearArcs;
var clearOuterArcs = function (gauge) {
    gauge.doughnut.current.selectAll(".outerSubArc").remove();
};
exports.clearOuterArcs = clearOuterArcs;
var validateArcs = function (gauge) {
    verifySubArcsLimits(gauge);
};
exports.validateArcs = validateArcs;
/**
 * Reorders the subArcs within the gauge's arc property based on the limit property.
 * SubArcs with undefined limits are sorted last.
*/
var reOrderSubArcs = function (gauge) {
    var _a;
    var subArcs = (_a = gauge.props.arc) === null || _a === void 0 ? void 0 : _a.subArcs;
    subArcs.sort(function (a, b) {
        if (typeof a.limit === 'undefined' && typeof b.limit === 'undefined') {
            return 0;
        }
        if (typeof a.limit === 'undefined') {
            return 1;
        }
        if (typeof b.limit === 'undefined') {
            return -1;
        }
        return a.limit - b.limit;
    });
};
var verifySubArcsLimits = function (gauge) {
    var _a;
    // disabled when length implemented.
    // reOrderSubArcs(gauge);
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    var arc = gauge.props.arc;
    var subArcs = arc.subArcs;
    var prevLimit = undefined;
    for (var _i = 0, _b = ((_a = gauge.props.arc) === null || _a === void 0 ? void 0 : _a.subArcs) || []; _i < _b.length; _i++) {
        var subArc = _b[_i];
        var limit = subArc.limit;
        if (typeof limit !== 'undefined') {
            // Check if the limit is within the valid range
            if (limit < minValue || limit > maxValue)
                throw new Error("The limit of a subArc must be between the minValue and maxValue. The limit of the subArc is ".concat(limit));
            // Check if the limit is greater than the previous limit
            if (typeof prevLimit !== 'undefined') {
                if (limit <= prevLimit)
                    throw new Error("The limit of a subArc must be greater than the limit of the previous subArc. The limit of the subArc is ".concat(limit, ". If you're trying to specify length in percent of the arc, use property \"length\". refer to: https://github.com/antoniolago/react-gauge-component"));
            }
            prevLimit = limit;
        }
    }
    // If the user has defined subArcs, make sure the last subArc has a limit equal to the maxValue
    if (subArcs.length > 0) {
        var lastSubArc = subArcs[subArcs.length - 1];
        if (lastSubArc.limit < maxValue)
            lastSubArc.limit = maxValue;
    }
};
