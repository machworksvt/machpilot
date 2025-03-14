import React from "react";
import { GaugeComponent } from "react-gauge-component";

const JetEngineRPM = () => (
  <GaugeComponent
    type="grafana"
    id=""
    className="gauge-component-class"
    value={33}
    minValue={0}
    maxValue={100}
    arc={{
      cornerRadius: 7,
      padding: 0.05,
      width: 0.25,
      nbSubArcs: undefined,
      colorArray: ["#5BE12C","#F5CD19","#EA4228"],
      emptyColor: "#5C5C5C",
      gradient: false,
      subArcs: undefined
    }}
    pointer={{
      type: "needle",
      hide: false,
      color: "#464A4F",
      baseColor: "#464A4F",
      length: 0.7,
      animate: true,
      elastic: false,
      animationDuration: 3000,
      animationDelay: 100,
      width: 20,
      strokeWidth: 8
    }}
    labels={{
      valueLabel: {
        matchColorWithArc: false,
        formatTextValue: undefined,
        style: {
  "fontSize": "35px",
  "fill": "#fff",
  "textShadow": "1px 1px 0px black, 0 0 2.5em black, 0 0 0.2em black"
},
        maxDecimalDigits: 2,
        hide: false
      },
      tickLabels: {
        type: "outer",
        hideMinMax: false,
        ticks: undefined,
        defaultTickValueConfig: {
          formatTextValue: undefined,
          style: {
  "fontSize": "10px",
  "fill": "#464A4F",
  "textShadow": "1px 1px 0px black, 0 0 2.5em black, 0 0 0.2em black"
},
          maxDecimalDigits: 2,
          hide: false
        },
        defaultTickLineConfig: {
          width: 1,
          length: 7,
          color: "rgb(173, 172, 171)",
          distanceFromArc: 3,
          hide: false
        }
      }
    }}
  />
);

export default JetEngineRPM;