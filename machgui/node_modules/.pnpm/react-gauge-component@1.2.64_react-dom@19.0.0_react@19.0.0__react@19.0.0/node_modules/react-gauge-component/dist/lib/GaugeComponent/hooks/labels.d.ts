import { Gauge } from '../types/Gauge';
import { Tick } from '../types/Tick';
import React from 'react';
export declare const setupLabels: (gauge: Gauge) => void;
export declare const setupValueLabel: (gauge: Gauge) => void;
export declare const setupTicks: (gauge: Gauge) => void;
export declare const addArcTicks: (gauge: Gauge) => void;
export declare const mapTick: (value: number, gauge: Gauge) => Tick;
export declare const addTickLine: (tick: Tick, gauge: Gauge) => void;
export declare const addTickValue: (tick: Tick, gauge: Gauge) => void;
export declare const addTick: (tick: Tick, gauge: Gauge) => void;
export declare const getLabelCoordsByValue: (value: number, gauge: Gauge, centerToArcLengthSubtract?: number) => {
    x: number;
    y: number;
};
export declare const addText: (html: any, x: number, y: number, gauge: Gauge, style: React.CSSProperties, className: string, rotate?: number) => void;
export declare const addValueText: (gauge: Gauge) => void;
export declare const clearValueLabel: (gauge: Gauge) => any;
export declare const clearTicks: (gauge: Gauge) => void;
export declare const calculateAnchorAndAngleByValue: (value: number, gauge: Gauge) => {
    tickAnchor: string;
    angle: number;
};
