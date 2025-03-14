import React from "react";
import { GaugeComponent } from "react-gauge-component";

const Abrir = () => (
  <GaugeComponent
    type="semicircle"
    id=""
    className="gauge-component-class"
    style={{
  "width": "600px",
  "height": "600px"
}}
    marginInPercent={{
  "top": 0.01,
  "bottom": 0,
  "left": 0.07,
  "right": 0.07
}}
    value={33}
    minValue={0}
    maxValue={122}
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
        type: "inner",
        hideMinMax: false,
        ticks: [
  {
    "value": 10
  },
  {
    "value": 20
  },
  {
    "value": 38
  },
  {
    "value": 50
  },
  {
    "value": 60
  },
  {
    "value": 75
  },
  {
    "value": 100
  },
  {
    "value": 110
  }
],
        defaultTickValueConfig: {
          formatTextValue: undefined,
          style: {
  "fontSize": "10px",
  "fill": "#FFFFFF",
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

export default Abrir;