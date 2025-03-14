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
Object.defineProperty(exports, "__esModule", { value: true });
exports.camelCaseToKebabCase = exports.getCurrentGaugeValueByPercentage = exports.getCurrentGaugePercentageByValue = exports.degToRad = exports.normalize = exports.floatingNumber = exports.percentToRad = exports.mergeObjects = exports.isEmptyObject = exports.calculatePercentage = void 0;
var calculatePercentage = function (minValue, maxValue, value) {
    if (value < minValue) {
        return 0;
    }
    else if (value > maxValue) {
        return 1;
    }
    else {
        var percentage = (value - minValue) / (maxValue - minValue);
        return (percentage);
    }
};
exports.calculatePercentage = calculatePercentage;
var isEmptyObject = function (obj) {
    return Object.keys(obj).length === 0 && obj.constructor === Object;
};
exports.isEmptyObject = isEmptyObject;
var mergeObjects = function (obj1, obj2) {
    var mergedObj = __assign({}, obj1);
    Object.keys(obj2).forEach(function (key) {
        var val1 = obj1[key];
        var val2 = obj2[key];
        if (Array.isArray(val1) && Array.isArray(val2)) {
            mergedObj[key] = val2;
        }
        else if (typeof val1 === 'object' && typeof val2 === 'object') {
            mergedObj[key] = (0, exports.mergeObjects)(val1, val2);
        }
        else if (val2 !== undefined) {
            mergedObj[key] = val2;
        }
    });
    return mergedObj;
};
exports.mergeObjects = mergeObjects;
//Returns the angle (in rad) for the given 'percent' value where percent = 1 means 100% and is 180 degree angle
var percentToRad = function (percent, angle) {
    return percent * (Math.PI / angle);
};
exports.percentToRad = percentToRad;
var floatingNumber = function (value, maxDigits) {
    if (maxDigits === void 0) { maxDigits = 2; }
    return Math.round(value * Math.pow(10, maxDigits)) / Math.pow(10, maxDigits);
};
exports.floatingNumber = floatingNumber;
// Function to normalize a value between a new min and max
function normalize(value, min, max) {
    return ((value - min) / (max - min)) * 100;
}
exports.normalize = normalize;
var degToRad = function (degrees) {
    return degrees * (Math.PI / 180);
};
exports.degToRad = degToRad;
var getCurrentGaugePercentageByValue = function (value, gauge) { return (0, exports.calculatePercentage)(gauge.minValue, gauge.maxValue, value); };
exports.getCurrentGaugePercentageByValue = getCurrentGaugePercentageByValue;
var getCurrentGaugeValueByPercentage = function (percentage, gauge) {
    var minValue = gauge.props.minValue;
    var maxValue = gauge.props.maxValue;
    var value = minValue + (percentage) * (maxValue - minValue);
    return value;
};
exports.getCurrentGaugeValueByPercentage = getCurrentGaugeValueByPercentage;
var camelCaseToKebabCase = function (str) { return str.replace(/[A-Z]/g, function (letter) { return "-".concat(letter.toLowerCase()); }); };
exports.camelCaseToKebabCase = camelCaseToKebabCase;
