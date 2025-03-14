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
Object.defineProperty(exports, "__esModule", { value: true });
exports.GaugeComponent = void 0;
var react_1 = __importStar(require("react"));
var d3_1 = require("d3");
var GaugeComponentProps_1 = require("./types/GaugeComponentProps");
var chartHooks = __importStar(require("./hooks/chart"));
var arcHooks = __importStar(require("./hooks/arc"));
var utils_1 = require("./hooks/utils");
var Dimensions_1 = require("./types/Dimensions");
var Pointer_1 = require("./types/Pointer");
var Arc_1 = require("./types/Arc");
/*
GaugeComponent creates a gauge chart using D3
The chart is responsive and will have the same width as the "container"
The radius of the gauge depends on the width and height of the container
It will use whichever is smallest of width or height
The svg element surrounding the gauge will always be square
"container" is the div where the chart should be placed
*/
var GaugeComponent = function (props) {
    var svg = (0, react_1.useRef)({});
    var tooltip = (0, react_1.useRef)({});
    var g = (0, react_1.useRef)({});
    var doughnut = (0, react_1.useRef)({});
    var isFirstRun = (0, react_1.useRef)(true);
    var currentProgress = (0, react_1.useRef)(0);
    var pointer = (0, react_1.useRef)(__assign({}, Pointer_1.defaultPointerRef));
    var container = (0, react_1.useRef)({});
    var arcData = (0, react_1.useRef)([]);
    var pieChart = (0, react_1.useRef)((0, d3_1.pie)());
    var dimensions = (0, react_1.useRef)(__assign({}, Dimensions_1.defaultDimensions));
    var mergedProps = (0, react_1.useRef)(props);
    var prevProps = (0, react_1.useRef)({});
    var resizeObserver = (0, react_1.useRef)({});
    var selectedRef = (0, react_1.useRef)(null);
    var gauge = {
        props: mergedProps.current,
        resizeObserver: resizeObserver,
        prevProps: prevProps,
        svg: svg,
        g: g,
        dimensions: dimensions,
        doughnut: doughnut,
        isFirstRun: isFirstRun,
        currentProgress: currentProgress,
        pointer: pointer,
        container: container,
        arcData: arcData,
        pieChart: pieChart,
        tooltip: tooltip
    };
    //Merged properties will get the default props and overwrite by the user's defined props
    //To keep the original default props in the object
    var updateMergedProps = function () {
        var _a, _b;
        var defaultValues = __assign({}, GaugeComponentProps_1.defaultGaugeProps);
        gauge.props = mergedProps.current = (0, utils_1.mergeObjects)(defaultValues, props);
        if (((_a = gauge.props.arc) === null || _a === void 0 ? void 0 : _a.width) == ((_b = GaugeComponentProps_1.defaultGaugeProps.arc) === null || _b === void 0 ? void 0 : _b.width)) {
            var mergedArc = mergedProps.current.arc;
            mergedArc.width = (0, Arc_1.getArcWidthByType)(gauge.props.type);
        }
        if (gauge.props.marginInPercent == GaugeComponentProps_1.defaultGaugeProps.marginInPercent)
            mergedProps.current.marginInPercent = (0, GaugeComponentProps_1.getGaugeMarginByType)(gauge.props.type);
        arcHooks.validateArcs(gauge);
    };
    var shouldInitChart = function () {
        var arcsPropsChanged = (JSON.stringify(prevProps.current.arc) !== JSON.stringify(mergedProps.current.arc));
        var pointerPropsChanged = (JSON.stringify(prevProps.current.pointer) !== JSON.stringify(mergedProps.current.pointer));
        var valueChanged = (JSON.stringify(prevProps.current.value) !== JSON.stringify(mergedProps.current.value));
        var minValueChanged = (JSON.stringify(prevProps.current.minValue) !== JSON.stringify(mergedProps.current.minValue));
        var maxValueChanged = (JSON.stringify(prevProps.current.maxValue) !== JSON.stringify(mergedProps.current.maxValue));
        return arcsPropsChanged || pointerPropsChanged || valueChanged || minValueChanged || maxValueChanged;
    };
    (0, react_1.useLayoutEffect)(function () {
        updateMergedProps();
        isFirstRun.current = (0, utils_1.isEmptyObject)(container.current);
        if (isFirstRun.current)
            container.current = (0, d3_1.select)(selectedRef.current);
        if (shouldInitChart())
            chartHooks.initChart(gauge, isFirstRun.current);
        gauge.prevProps.current = mergedProps.current;
    }, [props]);
    // useEffect(() => {
    //   const observer = new MutationObserver(function () {
    //     setTimeout(() => window.dispatchEvent(new Event('resize')), 10);
    //     if (!selectedRef.current?.offsetParent) return;
    //     chartHooks.renderChart(gauge, true);
    //     observer.disconnect()
    //   });
    //   observer.observe(selectedRef.current?.parentNode, {attributes: true, subtree: false});
    //   return () => observer.disconnect();
    // }, [selectedRef.current?.parentNode?.offsetWidth, selectedRef.current?.parentNode?.offsetHeight]);
    (0, react_1.useEffect)(function () {
        var handleResize = function () { return chartHooks.renderChart(gauge, true); };
        //Set up resize event listener to re-render the chart everytime the window is resized
        window.addEventListener("resize", handleResize);
        return function () { return window.removeEventListener("resize", handleResize); };
    }, [props]);
    // useEffect(() => {
    //   console.log(selectedRef.current?.offsetWidth)
    //   // workaround to trigger recomputing of gauge size on first load (e.g. F5)
    //   setTimeout(() => window.dispatchEvent(new Event('resize')), 10);
    // }, [selectedRef.current?.parentNode]);
    (0, react_1.useEffect)(function () {
        var element = selectedRef.current;
        if (!element)
            return;
        // Create observer instance
        var observer = new ResizeObserver(function () {
            chartHooks.renderChart(gauge, true);
        });
        // Store observer reference
        gauge.resizeObserver.current = observer;
        // Observe parent node
        if (element.parentNode) {
            observer.observe(element.parentNode);
        }
        // Cleanup
        return function () {
            var _a;
            if (gauge.resizeObserver) {
                (_a = gauge.resizeObserver.current) === null || _a === void 0 ? void 0 : _a.disconnect();
                delete gauge.resizeObserver.current;
            }
        };
    }, []);
    var id = props.id, style = props.style, className = props.className, type = props.type;
    return (react_1.default.createElement("div", { id: id, className: "".concat(gauge.props.type, "-gauge").concat(className ? ' ' + className : ''), style: style, ref: function (svg) { return (selectedRef.current = svg); } }));
};
exports.GaugeComponent = GaugeComponent;
exports.default = GaugeComponent;
