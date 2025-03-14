/// <reference types="react" />
import { GaugeComponentProps } from './GaugeComponentProps';
import { SubArc } from './Arc';
import { Dimensions } from './Dimensions';
export interface Gauge {
    props: GaugeComponentProps;
    prevProps: React.MutableRefObject<GaugeComponentProps>;
    svg: React.MutableRefObject<any>;
    g: React.MutableRefObject<any>;
    doughnut: React.MutableRefObject<any>;
    resizeObserver: React.MutableRefObject<any>;
    pointer: React.MutableRefObject<any>;
    container: React.MutableRefObject<any>;
    isFirstRun: React.MutableRefObject<boolean>;
    currentProgress: React.MutableRefObject<number>;
    dimensions: React.MutableRefObject<Dimensions>;
    arcData: React.MutableRefObject<SubArc[]>;
    pieChart: React.MutableRefObject<any>;
    tooltip: React.MutableRefObject<any>;
}
