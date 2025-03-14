'use strict';

Object.defineProperty(exports, "__esModule", {
    value: true
});

var _propTypes = require('prop-types');

var _propTypes2 = _interopRequireDefault(_propTypes);

var _react = require('react');

var _react2 = _interopRequireDefault(_react);

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

var Gradient = function Gradient(props) {
    return _react2.default.createElement(
        'defs',
        null,
        _react2.default.createElement('linearGradient', props)
    );
};

Gradient.propTypes = {
    x1: _propTypes2.default.string,
    x2: _propTypes2.default.string,
    y1: _propTypes2.default.string,
    y2: _propTypes2.default.string,
    children: _propTypes2.default.oneOfType([_propTypes2.default.arrayOf(_propTypes2.default.object), _propTypes2.default.arrayOf(_propTypes2.default.node), _propTypes2.default.node])
};

Gradient.defaultProps = {
    x1: '0%',
    x2: '0%',
    y1: '100%',
    y2: '0%'
};

exports.default = Gradient;