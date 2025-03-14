import React from "react";
import { GaugeComponent } from "react-gauge-component";

const JETENGINESTUFF = () => (
  <GaugeComponent
    arc={{
      nbSubArcs: 15,
      colorArray: ["#5BE12C","#F5CD19","#EA4228"],
      width: 0.3,
      padding: 0.003
    }}
    pointer={{ type: "needle" }}
    value={64}
    minValue={0}
    maxValue={122}
  />
);

export default JETENGINESTUFF;