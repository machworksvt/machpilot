import React from "react";
import GaugeComponent from "react-gauge-component";

// Conversion helper to format kbit/s values.
const kbitsToMbits = (value) => {
  if (value >= 1000) {
    value = value / 1000;
    return Number.isInteger(value)
      ? value.toFixed(0) + " mbit/s"
      : value.toFixed(1) + " mbit/s";
  } else {
    return value.toFixed(0) + " kbit/s";
  }
};

const CustomGauge = (props) => {
  const { value = 0, style, ...rest } = props;
  return (
    <GaugeComponent
      value={value}
      arc={{
        nbSubArcs: 150,
        colorArray: ['#5BE12C', '#F5CD19', '#EA4228'],
        width: 0.3,
        padding: 0.003,
      }}
      labels={{
        valueLabel: {
          style: { fontSize: 40 },
          formatTextValue: kbitsToMbits,
        },
        tickLabels: {
          type: "outer",
          ticks: [
            { value: 100 },
            { value: 200 },
            { value: 300 },
            { value: 400 },
            { value: 500 },
            { value: 600 },
            { value: 700 },
            { value: 800 },
            { value: 900 },
            { value: 1000 },
            { value: 1500 },
            { value: 2000 },
            { value: 2500 },
            { value: 5000},
          ],
          defaultTickValueConfig: {
            formatTextValue: kbitsToMbits,
          },
        },
      }}
      maxValue={5000}
      style={style}
      {...rest}
    />
  );
};

export default CustomGauge;

