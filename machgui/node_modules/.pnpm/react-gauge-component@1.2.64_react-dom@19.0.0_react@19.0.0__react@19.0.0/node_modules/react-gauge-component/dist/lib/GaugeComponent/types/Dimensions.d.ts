export interface Margin {
    top: number;
    right: number;
    bottom: number;
    left: number;
}
export interface Angles {
    startAngle: number;
    endAngle: number;
    startAngleDeg: number;
    endAngleDeg: number;
}
export interface Dimensions {
    width: number;
    height: number;
    margin: Margin;
    angles: Angles;
    outerRadius: number;
    innerRadius: number;
    fixedHeight: number;
}
export declare const defaultMargins: Margin;
export declare const defaultAngles: Angles;
export declare const defaultDimensions: Dimensions;
