import { Gauge } from "../types/Gauge";
export declare const initChart: (gauge: Gauge, isFirstRender: boolean) => void;
export declare const calculateAngles: (gauge: Gauge) => void;
export declare const renderChart: (gauge: Gauge, resize?: boolean) => void;
export declare const updateDimensions: (gauge: Gauge) => void;
export declare const calculateRadius: (gauge: Gauge) => void;
export declare const centerGraph: (gauge: Gauge) => void;
export declare const clearChart: (gauge: Gauge) => void;
