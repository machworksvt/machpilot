/// <reference types="react" />
import { Arc } from "./Arc";
import { Labels } from './Labels';
import { PointerProps } from "./Pointer";
export declare enum GaugeType {
    Semicircle = "semicircle",
    Radial = "radial",
    Grafana = "grafana"
}
export interface GaugeInnerMarginInPercent {
    top: number;
    bottom: number;
    left: number;
    right: number;
}
export interface GaugeComponentProps {
    /** Gauge element will inherit this. */
    id?: string;
    /** Gauge element will inherit this. */
    className?: string;
    /** Gauge element will inherit this. */
    style?: React.CSSProperties;
    /** This configures the canvas margin in relationship with the gauge.
     * Default values:
     * [GaugeType.Grafana]: { top: 0.12, bottom: 0.00, left: 0.07, right: 0.07 },
        [GaugeType.Semicircle]: { top: 0.08, bottom: 0.00, left: 0.07, right: 0.07 },
        [GaugeType.Radial]: { top: 0.07, bottom: 0.00, left: 0.07, right: 0.07 },
    */
    marginInPercent?: GaugeInnerMarginInPercent | number;
    /** Current pointer value. */
    value?: number;
    /** Minimum value possible for the Gauge. */
    minValue?: number;
    /** Maximum value possible for the Gauge. */
    maxValue?: number;
    /** This configures the arc of the Gauge. */
    arc?: Arc;
    /** This configures the labels of the Gauge. */
    labels?: Labels;
    /** This configures the pointer of the Gauge. */
    pointer?: PointerProps;
    /** This configures the type of the Gauge. */
    type?: "semicircle" | "radial" | "grafana";
}
export declare const defaultGaugeProps: GaugeComponentProps;
export declare const getGaugeMarginByType: (type: string) => GaugeInnerMarginInPercent | number;
