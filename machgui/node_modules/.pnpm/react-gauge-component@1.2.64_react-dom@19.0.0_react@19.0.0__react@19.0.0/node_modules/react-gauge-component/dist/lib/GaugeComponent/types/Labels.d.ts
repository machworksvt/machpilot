/// <reference types="react" />
import { TickLabels } from './Tick';
export interface Labels {
    /** This configures the central value label. */
    valueLabel?: ValueLabel;
    /** This configures the ticks and it's values labels. */
    tickLabels?: TickLabels;
}
export interface ValueLabel {
    /** This function enables to format the central value text as you wish. */
    formatTextValue?: (value: any) => string;
    /** This will sync the value label color with the current value of the Gauge. */
    matchColorWithArc?: boolean;
    /** This enables configuration for the number of decimal digits of the
     * central value label */
    maxDecimalDigits?: number;
    /** Central label value will inherit this */
    style?: React.CSSProperties;
    /** This hides the central value label if true */
    hide?: boolean;
}
export declare const defaultValueLabel: ValueLabel;
export declare const defaultLabels: Labels;
