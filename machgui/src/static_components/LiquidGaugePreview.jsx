import React from "react";
import LiquidFillGauge from "react-liquid-gauge";
import { color } from "d3-color";
import { interpolateRgb } from "d3-interpolate";

const LiquidGaugePreview = (props) => {
  const {
    liquidValue,
    radius,
    textSize,
    textOffsetX,
    textOffsetY,
    riseAnimation,
    waveAnimation,
    waveFrequency,
    waveAmplitude,
    gradient,
    startColor,
    endColor,
  } = props;

  const interpolate = interpolateRgb(startColor, endColor);
  const fillColor = interpolate(liquidValue / 100);
  const gradientStops = [
    {
      key: "0%",
      stopColor: color(fillColor).darker(0.5).toString(),
      stopOpacity: 1,
      offset: "0%"
    },
    {
      key: "50%",
      stopColor: fillColor,
      stopOpacity: 0.75,
      offset: "50%"
    },
    {
      key: "100%",
      stopColor: color(fillColor).brighter(0.5).toString(),
      stopOpacity: 0.5,
      offset: "100%"
    }
  ];

  return (
    <div
      style={{
        border: "1px solid #ccc",
        width: "100%",
        minHeight: "800px",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        overflow: "visible"
      }}
    >
      <LiquidFillGauge
        style={{ margin: "0 auto" }}
        width={radius * 2}
        height={radius * 2}
        value={liquidValue}
        percent="%"
        textSize={textSize}
        textOffsetX={textOffsetX}
        textOffsetY={textOffsetY}
        textRenderer={(props) => {
          const value = Math.round(props.value);
          const r = Math.min(props.height / 2, props.width / 2);
          const textPixels = (props.textSize * r / 2);
          const valueStyle = {
            fontSize: textPixels
          };
          const percentStyle = {
            fontSize: textPixels * 0.6
          };
          return (
            <tspan>
              <tspan className="value" style={valueStyle}>
                {value}
              </tspan>
              <tspan style={percentStyle}>{props.percent}</tspan>
            </tspan>
          );
        }}
        riseAnimation={riseAnimation}
        waveAnimation={waveAnimation}
        waveFrequency={waveFrequency}
        waveAmplitude={waveAmplitude}
        gradient={gradient}
        gradientStops={gradientStops}
        circleStyle={{ fill: fillColor }}
        waveStyle={{ fill: fillColor }}
        textStyle={{
          fill: color("#444").toString(),
          fontFamily: "Arial"
        }}
        waveTextStyle={{
          fill: color("#fff").toString(),
          fontFamily: "Arial"
        }}
        onClick={() => {}}
      />
    </div>
  );
};

export default LiquidGaugePreview;
