import { Tooltip } from './Tooltip';
export interface Arc {
    /** The corner radius of the arc. */
    cornerRadius?: number;
    /** The padding between subArcs, in rad. */
    padding?: number;
    /** The width of the arc given in percent of the radius. */
    width?: number;
    /** The number of subArcs, this overrides "subArcs" limits. */
    nbSubArcs?: number;
    /** Boolean flag that enables or disables gradient mode, which
     * draws a single arc with provided colors. */
    gradient?: boolean;
    /** The colors of the arcs, this overrides "subArcs" colors. */
    colorArray?: Array<string>;
    /** Color of the grafana's empty subArc  */
    emptyColor?: string;
    /** list of sub arcs segments of the whole arc. */
    subArcs?: Array<SubArc>;
}
export interface SubArc {
    /** The limit of the subArc, in accord to the gauge value. */
    limit?: number;
    /** The color of the subArc */
    color?: string | number;
    /** The length of the subArc, in percent */
    length?: number;
    /** Whether or not to show the tick */
    showTick?: boolean;
    /** Tooltip that appears onHover of the subArc */
    tooltip?: Tooltip;
    /** This will trigger onClick of the subArc */
    onClick?: () => void;
    /** This will trigger onMouseMove of the subArc */
    onMouseMove?: () => void;
    /** This will trigger onMouseMove of the subArc */
    onMouseLeave?: () => void;
}
export declare const defaultSubArcs: SubArc[];
export declare const getArcWidthByType: (type: string) => number;
export declare const defaultArc: Arc;
