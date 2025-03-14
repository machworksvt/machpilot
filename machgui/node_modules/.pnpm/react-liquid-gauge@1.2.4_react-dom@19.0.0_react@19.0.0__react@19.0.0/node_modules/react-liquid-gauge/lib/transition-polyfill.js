'use strict';

var _d3Selection = require('d3-selection');

var _d3Transition = require('d3-transition');

// https://github.com/d3/d3-transition/blob/master/src/selection/index.js
_d3Selection.selection.prototype.transition = _d3Selection.selection.prototype.transition || _d3Transition.transition;
_d3Selection.selection.prototype.interrupt = _d3Selection.selection.prototype.interrupt || _d3Transition.interrupt;