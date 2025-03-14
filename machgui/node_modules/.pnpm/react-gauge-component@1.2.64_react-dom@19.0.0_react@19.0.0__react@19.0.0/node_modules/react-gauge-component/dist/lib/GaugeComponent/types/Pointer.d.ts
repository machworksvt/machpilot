export interface PointerProps {
    /** Pointer type */
    type?: "needle" | "blob" | "arrow";
    /** Pointer color */
    color?: string;
    /** Enabling this flag will hide the pointer */
    hide?: boolean;
    /** Pointer color of the central circle */
    baseColor?: string;
    /** Pointer length */
    length?: number;
    /** This is a factor to multiply by the width of the gauge */
    width?: number;
    /** This enables pointer animation for transiction between values when enabled */
    animate?: boolean;
    /** This gives animation an elastic transiction between values */
    elastic?: boolean;
    /** Animation duration in ms */
    animationDuration?: number;
    /** Animation delay in ms */
    animationDelay?: number;
    /** Stroke width of the pointer */
    strokeWidth?: number;
}
export interface PointerRef {
    element: any;
    path: any;
    context: PointerContext;
}
export interface PointerContext {
    centerPoint: number[];
    pointerRadius: number;
    pathLength: number;
    currentPercent: number;
    prevPercent: number;
    prevProgress: number;
    pathStr: string;
    shouldDrawPath: boolean;
    prevColor: string;
}
export declare enum PointerType {
    Needle = "needle",
    Blob = "blob",
    Arrow = "arrow"
}
export declare const defaultPointerContext: PointerContext;
export declare const defaultPointerRef: PointerRef;
export declare const defaultPointer: PointerProps;
