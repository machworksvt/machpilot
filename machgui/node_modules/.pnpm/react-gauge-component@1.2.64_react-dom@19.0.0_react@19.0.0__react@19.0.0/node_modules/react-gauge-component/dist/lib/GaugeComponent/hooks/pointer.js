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
Object.defineProperty(exports, "__esModule", { value: true });
exports.clearPointerElement = exports.addPointerElement = exports.translatePointer = exports.drawPointer = void 0;
var d3_1 = require("d3");
var Pointer_1 = require("../types/Pointer");
var arc_1 = require("./arc");
var utils = __importStar(require("./utils"));
var arcHooks = __importStar(require("./arc"));
var GaugeComponentProps_1 = require("../types/GaugeComponentProps");
var drawPointer = function (gauge, resize) {
    var _a;
    if (resize === void 0) { resize = false; }
    gauge.pointer.current.context = setupContext(gauge);
    var _b = gauge.pointer.current.context, prevPercent = _b.prevPercent, currentPercent = _b.currentPercent, prevProgress = _b.prevProgress;
    var pointer = gauge.props.pointer;
    var isFirstTime = ((_a = gauge.prevProps) === null || _a === void 0 ? void 0 : _a.current.value) == undefined;
    if ((isFirstTime || resize) && gauge.props.type != GaugeComponentProps_1.GaugeType.Grafana)
        initPointer(gauge);
    var shouldAnimate = (!resize || isFirstTime) && pointer.animate;
    if (shouldAnimate) {
        gauge.doughnut.current
            .transition()
            .delay(pointer.animationDelay)
            .ease(pointer.elastic ? d3_1.easeElastic : d3_1.easeExpOut)
            .duration(pointer.animationDuration)
            .tween("progress", function () {
            var currentInterpolatedPercent = (0, d3_1.interpolateNumber)(prevPercent, currentPercent);
            return function (percentOfPercent) {
                var progress = currentInterpolatedPercent(percentOfPercent);
                if (isProgressValid(progress, prevProgress, gauge)) {
                    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana) {
                        arcHooks.clearArcs(gauge);
                        arcHooks.drawArc(gauge, progress);
                        //arcHooks.setupArcs(gauge);
                    }
                    else {
                        updatePointer(progress, gauge);
                    }
                }
                gauge.pointer.current.context.prevProgress = progress;
            };
        });
    }
    else {
        updatePointer(currentPercent, gauge);
    }
};
exports.drawPointer = drawPointer;
var setupContext = function (gauge) {
    var _a;
    var value = gauge.props.value;
    var pointer = gauge.props.pointer;
    var pointerLength = pointer.length;
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    var pointerPath = gauge.pointer.current.context.pointerPath;
    var pointerRadius = getPointerRadius(gauge);
    var length = pointer.type == Pointer_1.PointerType.Needle ? pointerLength : 0.2;
    var typesWithPath = [Pointer_1.PointerType.Needle, Pointer_1.PointerType.Arrow];
    var pointerContext = {
        centerPoint: [0, -pointerRadius / 2],
        pointerRadius: getPointerRadius(gauge),
        pathLength: gauge.dimensions.current.outerRadius * length,
        currentPercent: utils.calculatePercentage(minValue, maxValue, value),
        prevPercent: utils.calculatePercentage(minValue, maxValue, ((_a = gauge.prevProps) === null || _a === void 0 ? void 0 : _a.current.value) || minValue),
        prevProgress: 0,
        pathStr: "",
        shouldDrawPath: typesWithPath.includes(pointer.type),
        prevColor: ""
    };
    return pointerContext;
};
var initPointer = function (gauge) {
    var value = gauge.props.value;
    var pointer = gauge.props.pointer;
    var _a = gauge.pointer.current.context, shouldDrawPath = _a.shouldDrawPath, centerPoint = _a.centerPoint, pointerRadius = _a.pointerRadius, pathStr = _a.pathStr, currentPercent = _a.currentPercent, prevPercent = _a.prevPercent;
    if (shouldDrawPath) {
        gauge.pointer.current.context.pathStr = calculatePointerPath(gauge, prevPercent || currentPercent);
        gauge.pointer.current.path = gauge.pointer.current.element.append("path").attr("d", gauge.pointer.current.context.pathStr).attr("fill", pointer.color);
    }
    //Add a circle at the bottom of pointer
    if (pointer.type == Pointer_1.PointerType.Needle) {
        gauge.pointer.current.element
            .append("circle")
            .attr("cx", centerPoint[0])
            .attr("cy", centerPoint[1])
            .attr("r", pointerRadius)
            .attr("fill", pointer.color);
    }
    else if (pointer.type == Pointer_1.PointerType.Blob) {
        gauge.pointer.current.element
            .append("circle")
            .attr("cx", centerPoint[0])
            .attr("cy", centerPoint[1])
            .attr("r", pointerRadius)
            .attr("fill", pointer.baseColor)
            .attr("stroke", pointer.color)
            .attr("stroke-width", pointer.strokeWidth * pointerRadius / 10);
    }
    //Translate the pointer starting point of the arc
    setPointerPosition(pointerRadius, value, gauge);
};
var updatePointer = function (percentage, gauge) {
    var _a;
    var pointer = gauge.props.pointer;
    var _b = gauge.pointer.current.context, pointerRadius = _b.pointerRadius, shouldDrawPath = _b.shouldDrawPath, prevColor = _b.prevColor;
    setPointerPosition(pointerRadius, percentage, gauge);
    if (shouldDrawPath && gauge.props.type != GaugeComponentProps_1.GaugeType.Grafana)
        gauge.pointer.current.path.attr("d", calculatePointerPath(gauge, percentage));
    if (pointer.type == Pointer_1.PointerType.Blob) {
        var currentColor = (_a = arcHooks.getArcDataByPercentage(percentage, gauge)) === null || _a === void 0 ? void 0 : _a.color;
        var shouldChangeColor = currentColor != prevColor;
        if (shouldChangeColor)
            gauge.pointer.current.element.select("circle").attr("stroke", currentColor);
        var strokeWidth = pointer.strokeWidth * pointerRadius / 10;
        gauge.pointer.current.element.select("circle").attr("stroke-width", strokeWidth);
        gauge.pointer.current.context.prevColor = currentColor;
    }
};
var setPointerPosition = function (pointerRadius, progress, gauge) {
    var _a;
    var pointer = gauge.props.pointer;
    var pointerType = pointer.type;
    var dimensions = gauge.dimensions;
    var value = utils.getCurrentGaugeValueByPercentage(progress, gauge);
    var pointers = (_a = {},
        _a[Pointer_1.PointerType.Needle] = function () {
            // Set needle position to center
            (0, exports.translatePointer)(dimensions.current.outerRadius, dimensions.current.outerRadius, gauge);
        },
        _a[Pointer_1.PointerType.Arrow] = function () {
            var _a = (0, arc_1.getCoordByValue)(value, gauge, "inner", 0, 0.70), x = _a.x, y = _a.y;
            x -= 1;
            y += pointerRadius - 3;
            (0, exports.translatePointer)(x, y, gauge);
        },
        _a[Pointer_1.PointerType.Blob] = function () {
            var _a = (0, arc_1.getCoordByValue)(value, gauge, "between", 0, 0.75), x = _a.x, y = _a.y;
            x -= 1;
            y += pointerRadius;
            (0, exports.translatePointer)(x, y, gauge);
        },
        _a);
    return pointers[pointerType]();
};
var isProgressValid = function (currentPercent, prevPercent, gauge) {
    //Avoid unnecessary re-rendering (when progress is too small) but allow the pointer to reach the final value
    var overFlow = currentPercent > 1 || currentPercent < 0;
    var tooSmallValue = Math.abs(currentPercent - prevPercent) < 0.0001;
    var sameValueAsBefore = currentPercent == prevPercent;
    return !tooSmallValue && !sameValueAsBefore && !overFlow;
};
var calculatePointerPath = function (gauge, percent) {
    var _a = gauge.pointer.current.context, centerPoint = _a.centerPoint, pointerRadius = _a.pointerRadius, pathLength = _a.pathLength;
    var startAngle = utils.degToRad(gauge.props.type == GaugeComponentProps_1.GaugeType.Semicircle ? 0 : -42);
    var endAngle = utils.degToRad(gauge.props.type == GaugeComponentProps_1.GaugeType.Semicircle ? 180 : 223);
    var angle = startAngle + (percent) * (endAngle - startAngle);
    var topPoint = [
        centerPoint[0] - pathLength * Math.cos(angle),
        centerPoint[1] - pathLength * Math.sin(angle),
    ];
    var thetaMinusHalfPi = angle - Math.PI / 2;
    var leftPoint = [
        centerPoint[0] - pointerRadius * Math.cos(thetaMinusHalfPi),
        centerPoint[1] - pointerRadius * Math.sin(thetaMinusHalfPi),
    ];
    var thetaPlusHalfPi = angle + Math.PI / 2;
    var rightPoint = [
        centerPoint[0] - pointerRadius * Math.cos(thetaPlusHalfPi),
        centerPoint[1] - pointerRadius * Math.sin(thetaPlusHalfPi),
    ];
    var pathStr = "M ".concat(leftPoint[0], " ").concat(leftPoint[1], " L ").concat(topPoint[0], " ").concat(topPoint[1], " L ").concat(rightPoint[0], " ").concat(rightPoint[1]);
    return pathStr;
};
var getPointerRadius = function (gauge) {
    var pointer = gauge.props.pointer;
    var pointerWidth = pointer.width;
    return pointerWidth * (gauge.dimensions.current.width / 500);
};
var translatePointer = function (x, y, gauge) { return gauge.pointer.current.element.attr("transform", "translate(" + x + ", " + y + ")"); };
exports.translatePointer = translatePointer;
var addPointerElement = function (gauge) { return gauge.pointer.current.element = gauge.g.current.append("g").attr("class", "pointer"); };
exports.addPointerElement = addPointerElement;
var clearPointerElement = function (gauge) { return gauge.pointer.current.element.selectAll("*").remove(); };
exports.clearPointerElement = clearPointerElement;
