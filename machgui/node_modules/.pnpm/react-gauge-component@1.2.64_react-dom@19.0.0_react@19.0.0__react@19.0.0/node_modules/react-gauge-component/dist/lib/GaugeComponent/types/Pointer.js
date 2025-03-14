"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.defaultPointer = exports.defaultPointerRef = exports.defaultPointerContext = exports.PointerType = void 0;
var PointerType;
(function (PointerType) {
    PointerType["Needle"] = "needle";
    PointerType["Blob"] = "blob";
    PointerType["Arrow"] = "arrow";
})(PointerType || (exports.PointerType = PointerType = {}));
exports.defaultPointerContext = {
    centerPoint: [0, 0],
    pointerRadius: 0,
    pathLength: 0,
    currentPercent: 0,
    prevPercent: 0,
    prevProgress: 0,
    pathStr: "",
    shouldDrawPath: false,
    prevColor: ""
};
exports.defaultPointerRef = {
    element: undefined,
    path: undefined,
    context: exports.defaultPointerContext
};
exports.defaultPointer = {
    type: PointerType.Needle,
    color: "#5A5A5A",
    baseColor: "white",
    length: 0.70,
    width: 20,
    animate: true,
    elastic: false,
    hide: false,
    animationDuration: 3000,
    animationDelay: 100,
    strokeWidth: 8
};
