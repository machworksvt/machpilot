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
exports.clearChart = exports.centerGraph = exports.calculateRadius = exports.updateDimensions = exports.renderChart = exports.calculateAngles = exports.initChart = void 0;
var GaugeComponentProps_1 = require("../types/GaugeComponentProps");
var arcHooks = __importStar(require("./arc"));
var labelsHooks = __importStar(require("./labels"));
var pointerHooks = __importStar(require("./pointer"));
var initChart = function (gauge, isFirstRender) {
    var _a, _b, _c, _d;
    var angles = gauge.dimensions.current.angles;
    if ((_b = (_a = gauge.resizeObserver) === null || _a === void 0 ? void 0 : _a.current) === null || _b === void 0 ? void 0 : _b.disconnect) {
        (_d = (_c = gauge.resizeObserver) === null || _c === void 0 ? void 0 : _c.current) === null || _d === void 0 ? void 0 : _d.disconnect();
    }
    var updatedValue = (JSON.stringify(gauge.prevProps.current.value) !== JSON.stringify(gauge.props.value));
    if (updatedValue && !isFirstRender) {
        (0, exports.renderChart)(gauge, false);
        return;
    }
    gauge.container.current.select("svg").remove();
    gauge.svg.current = gauge.container.current.append("svg");
    gauge.g.current = gauge.svg.current.append("g"); //Used for margins
    gauge.doughnut.current = gauge.g.current.append("g").attr("class", "doughnut");
    //gauge.outerDougnut.current = gauge.g.current.append("g").attr("class", "doughnut");
    (0, exports.calculateAngles)(gauge);
    gauge.pieChart.current
        .value(function (d) { return d.value; })
        //.padAngle(15)
        .startAngle(angles.startAngle)
        .endAngle(angles.endAngle)
        .sort(null);
    //Set up pointer
    pointerHooks.addPointerElement(gauge);
    (0, exports.renderChart)(gauge, true);
};
exports.initChart = initChart;
var calculateAngles = function (gauge) {
    var angles = gauge.dimensions.current.angles;
    if (gauge.props.type == GaugeComponentProps_1.GaugeType.Semicircle) {
        angles.startAngle = -Math.PI / 2 + 0.02;
        angles.endAngle = Math.PI / 2 - 0.02;
    }
    else if (gauge.props.type == GaugeComponentProps_1.GaugeType.Radial) {
        angles.startAngle = -Math.PI / 1.37;
        angles.endAngle = Math.PI / 1.37;
    }
    else if (gauge.props.type == GaugeComponentProps_1.GaugeType.Grafana) {
        angles.startAngle = -Math.PI / 1.6;
        angles.endAngle = Math.PI / 1.6;
    }
};
exports.calculateAngles = calculateAngles;
//Renders the chart, should be called every time the window is resized
var renderChart = function (gauge, resize) {
    var _a;
    var _b, _c, _d, _e, _f;
    if (resize === void 0) { resize = false; }
    var dimensions = gauge.dimensions;
    var arc = gauge.props.arc;
    var labels = gauge.props.labels;
    //if resize recalculate dimensions, clear chart and redraw
    //if not resize, treat each prop separately
    if (resize) {
        (0, exports.updateDimensions)(gauge);
        //Set dimensions of svg element and translations
        gauge.g.current.attr("transform", "translate(" + dimensions.current.margin.left + ", " + 35 + ")");
        //Set the radius to lesser of width or height and remove the margins
        //Calculate the new radius
        (0, exports.calculateRadius)(gauge);
        gauge.doughnut.current.attr("transform", "translate(" + dimensions.current.outerRadius + ", " + dimensions.current.outerRadius + ")");
        //Hide tooltip failsafe (sometimes subarcs events are not fired)
        gauge.doughnut.current
            .on("mouseleave", function () { return arcHooks.hideTooltip(gauge); })
            .on("mouseout", function () { return arcHooks.hideTooltip(gauge); });
        var arcWidth = arc.width;
        dimensions.current.innerRadius = dimensions.current.outerRadius * (1 - arcWidth);
        (0, exports.clearChart)(gauge);
        arcHooks.setArcData(gauge);
        arcHooks.setupArcs(gauge, resize);
        labelsHooks.setupLabels(gauge);
        if (!((_c = (_b = gauge.props) === null || _b === void 0 ? void 0 : _b.pointer) === null || _c === void 0 ? void 0 : _c.hide))
            pointerHooks.drawPointer(gauge, resize);
        var gaugeTypeHeightCorrection = (_a = {},
            _a[GaugeComponentProps_1.GaugeType.Semicircle] = 50,
            _a[GaugeComponentProps_1.GaugeType.Radial] = 55,
            _a[GaugeComponentProps_1.GaugeType.Grafana] = 55,
            _a);
        var boundHeight = gauge.doughnut.current.node().getBoundingClientRect().height;
        var boundWidth = gauge.container.current.node().getBoundingClientRect().width;
        var gaugeType = gauge.props.type;
        gauge.svg.current
            .attr("width", boundWidth)
            .attr("height", boundHeight + gaugeTypeHeightCorrection[gaugeType]);
    }
    else {
        var arcsPropsChanged = (JSON.stringify(gauge.prevProps.current.arc) !== JSON.stringify(gauge.props.arc));
        var pointerPropsChanged = (JSON.stringify(gauge.prevProps.current.pointer) !== JSON.stringify(gauge.props.pointer));
        var valueChanged = (JSON.stringify(gauge.prevProps.current.value) !== JSON.stringify(gauge.props.value));
        var ticksChanged = (JSON.stringify((_d = gauge.prevProps.current.labels) === null || _d === void 0 ? void 0 : _d.tickLabels) !== JSON.stringify(labels.tickLabels));
        var shouldRedrawArcs = arcsPropsChanged;
        if (shouldRedrawArcs) {
            arcHooks.clearArcs(gauge);
            arcHooks.setArcData(gauge);
            arcHooks.setupArcs(gauge, resize);
        }
        //If pointer is hidden there's no need to redraw it when only value changes
        var shouldRedrawPointer = pointerPropsChanged || (valueChanged && !((_f = (_e = gauge.props) === null || _e === void 0 ? void 0 : _e.pointer) === null || _f === void 0 ? void 0 : _f.hide));
        if ((shouldRedrawPointer)) {
            pointerHooks.drawPointer(gauge);
        }
        if (arcsPropsChanged || ticksChanged) {
            labelsHooks.clearTicks(gauge);
            labelsHooks.setupTicks(gauge);
        }
        if (valueChanged) {
            labelsHooks.clearValueLabel(gauge);
            labelsHooks.setupValueLabel(gauge);
        }
    }
};
exports.renderChart = renderChart;
var updateDimensions = function (gauge) {
    var marginInPercent = gauge.props.marginInPercent;
    var dimensions = gauge.dimensions;
    var divDimensions = gauge.container.current.node().getBoundingClientRect(), divWidth = divDimensions.width, divHeight = divDimensions.height;
    if (dimensions.current.fixedHeight == 0)
        dimensions.current.fixedHeight = divHeight + 200;
    //Set the new width and horizontal margins
    var isMarginBox = typeof marginInPercent == 'number';
    var marginLeft = isMarginBox ? marginInPercent : marginInPercent.left;
    var marginRight = isMarginBox ? marginInPercent : marginInPercent.right;
    var marginTop = isMarginBox ? marginInPercent : marginInPercent.top;
    var marginBottom = isMarginBox ? marginInPercent : marginInPercent.bottom;
    dimensions.current.margin.left = divWidth * marginLeft;
    dimensions.current.margin.right = divWidth * marginRight;
    dimensions.current.width = divWidth - dimensions.current.margin.left - dimensions.current.margin.right;
    dimensions.current.margin.top = dimensions.current.fixedHeight * marginTop;
    dimensions.current.margin.bottom = dimensions.current.fixedHeight * marginBottom;
    dimensions.current.height = dimensions.current.width / 2 - dimensions.current.margin.top - dimensions.current.margin.bottom;
    //gauge.height.current = divHeight - dimensions.current.margin.top - dimensions.current.margin.bottom;
};
exports.updateDimensions = updateDimensions;
var calculateRadius = function (gauge) {
    var dimensions = gauge.dimensions;
    //The radius needs to be constrained by the containing div
    //Since it is a half circle we are dealing with the height of the div
    //Only needs to be half of the width, because the width needs to be 2 * radius
    //For the whole arc to fit
    //First check if it is the width or the height that is the "limiting" dimension
    if (dimensions.current.width < 2 * dimensions.current.height) {
        //Then the width limits the size of the chart
        //Set the radius to the width - the horizontal margins
        dimensions.current.outerRadius = (dimensions.current.width - dimensions.current.margin.left - dimensions.current.margin.right) / 2;
    }
    else {
        dimensions.current.outerRadius =
            dimensions.current.height - dimensions.current.margin.top - dimensions.current.margin.bottom + 35;
    }
    (0, exports.centerGraph)(gauge);
};
exports.calculateRadius = calculateRadius;
//Calculates new margins to make the graph centered
var centerGraph = function (gauge) {
    var dimensions = gauge.dimensions;
    dimensions.current.margin.left =
        dimensions.current.width / 2 - dimensions.current.outerRadius + dimensions.current.margin.right;
    gauge.g.current.attr("transform", "translate(" + dimensions.current.margin.left + ", " + (dimensions.current.margin.top) + ")");
};
exports.centerGraph = centerGraph;
var clearChart = function (gauge) {
    //Remove the old stuff
    labelsHooks.clearTicks(gauge);
    labelsHooks.clearValueLabel(gauge);
    pointerHooks.clearPointerElement(gauge);
    arcHooks.clearArcs(gauge);
};
exports.clearChart = clearChart;
