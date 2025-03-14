import React, { useState } from "react";
import { Allotment } from "allotment";
import "allotment/dist/style.css";
import "./styles.css";
import GaugeForm from "./static_components/GaugeForm";
import GaugePreview from "./static_components/GaugePreview";
import LiquidGaugeForm from "./static_components/LiquidGaugeForm";
import LiquidGaugePreview from "./static_components/LiquidGaugePreview";

class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, errorMsg: "" };
  }
  
  static getDerivedStateFromError(error) {
    return { hasError: true, errorMsg: error.toString() };
  }
  
  componentDidCatch(error, info) {
    console.error("ErrorBoundary caught an error:", error, info);
  }
  
  resetError = () => {
    this.setState({ hasError: false, errorMsg: "" });
  };

  render() {
    if (this.state.hasError) {
      return (
        <div style={{ color: "red", padding: "20px" }}>
          <div>Error: {this.state.errorMsg}</div>
          <button onClick={this.resetError}>Try Again</button>
        </div>
      );
    }
    return this.props.children;
  }
}

// Helper functions for Gauge code generation.
const parseCommaSeparatedListToObjects = (str, keyName) => {
  if (!str.trim()) return undefined;
  if (str.trim().startsWith("[")) {
    try {
      return JSON.parse(str);
    } catch (e) {
      return undefined;
    }
  }
  const arr = str.split(",").map((s) => s.trim()).filter((s) => s !== "");
  return arr.map((item) => {
    const num = Number(item);
    return { [keyName]: isNaN(num) ? item : num };
  });
};

const parseFunction = (str) => {
  if (!str.trim()) return undefined;
  try {
    return new Function("value", `return (${str})(value);`);
  } catch (e) {
    return undefined;
  }
};

function App() {
  // State to choose which component set to use.
  const [componentSet, setComponentSet] = useState("gauge"); // "gauge" or "liquid"

  // Shared field style.
  const fieldStyle = { display: "flex", flexDirection: "column", marginBottom: "10px" };

  // Common Gauge state variables (as in your original code)
  const [componentName, setComponentName] = useState("");
  const [value, setValue] = useState("33");
  const [minValue, setMinValue] = useState("0");
  const [maxValue, setMaxValue] = useState("100");

  const [gaugeType, setGaugeType] = useState("grafana");
  const [gaugeId, setGaugeId] = useState("");
  const [gaugeClassName, setGaugeClassName] = useState("gauge-component-class");

  const [cornerRadius, setCornerRadius] = useState(7);
  const [arcPadding, setArcPadding] = useState(0.05);
  const [arcWidth, setArcWidth] = useState(gaugeType === "grafana" ? 0.25 : gaugeType === "semicircle" ? 0.15 : 0.2);
  const [nbSubArcs, setNbSubArcs] = useState("");
  const [colorArray, setColorArray] = useState("#5BE12C,#F5CD19,#EA4228");
  const [emptyColor, setEmptyColor] = useState("#5C5C5C");
  const [gradient, setGradient] = useState(false);
  const [subArcs, setSubArcs] = useState("");

  const [pointerType, setPointerType] = useState("needle");
  const [pointerHide, setPointerHide] = useState(false);
  const [pointerColor, setPointerColor] = useState("#464A4F");
  const [pointerBaseColor, setPointerBaseColor] = useState("#464A4F");
  const [pointerLength, setPointerLength] = useState(0.7);
  const [pointerAnimate, setPointerAnimate] = useState(true);
  const [pointerElastic, setPointerElastic] = useState(false);
  const [pointerAnimationDuration, setPointerAnimationDuration] = useState(3000);
  const [pointerAnimationDelay, setPointerAnimationDelay] = useState(100);
  const [pointerWidth, setPointerWidth] = useState(20);
  const [pointerStrokeWidth, setPointerStrokeWidth] = useState(8);

  const [valueLabelMatchColor, setValueLabelMatchColor] = useState(false);
  const [valueLabelFormat, setValueLabelFormat] = useState("");
  const [valueLabelFontSize, setValueLabelFontSize] = useState("35px");
  const [valueLabelFill, setValueLabelFill] = useState("#fff");
  const [valueLabelTextShadow, setValueLabelTextShadow] = useState("1px 1px 0px black, 0 0 2.5em black, 0 0 0.2em black");
  const [valueLabelMaxDecimalDigits, setValueLabelMaxDecimalDigits] = useState(2);
  const [valueLabelHide, setValueLabelHide] = useState(false);

  const [tickLabelsType, setTickLabelsType] = useState("outer");
  const [tickLabelsHideMinMax, setTickLabelsHideMinMax] = useState(false);
  const [tickLabelsTicks, setTickLabelsTicks] = useState("");
  const [tickDefaultFormat, setTickDefaultFormat] = useState("");
  const [tickDefaultFontSize, setTickDefaultFontSize] = useState("10px");
  const [tickDefaultFill, setTickDefaultFill] = useState("#464A4F");
  const [tickDefaultTextShadow, setTickDefaultTextShadow] = useState("1px 1px 0px black, 0 0 2.5em black, 0 0 0.2em black");
  const [tickDefaultMaxDecimalDigits, setTickDefaultMaxDecimalDigits] = useState(2);
  const [tickDefaultHide, setTickDefaultHide] = useState(false);
  const [tickLineWidth, setTickLineWidth] = useState(1);
  const [tickLineLength, setTickLineLength] = useState(7);
  const [tickLineColor, setTickLineColor] = useState("rgb(173, 172, 171)");
  const [tickLineDistance, setTickLineDistance] = useState(3);
  const [tickLineHide, setTickLineHide] = useState(false);

  // Liquid Gauge state variables
  const [liquidValue, setLiquidValue] = useState(50);
  const [radius, setRadius] = useState(200);
  const [textSize, setTextSize] = useState(1);
  const [textOffsetX, setTextOffsetX] = useState(0);
  const [textOffsetY, setTextOffsetY] = useState(0);
  const [riseAnimation, setRiseAnimation] = useState(true);
  const [waveAnimation, setWaveAnimation] = useState(true);
  const [waveFrequency, setWaveFrequency] = useState(2);
  const [waveAmplitude, setWaveAmplitude] = useState(1);
  const [liquidGradient, setLiquidGradient] = useState(true);
  const [startColor, setStartColor] = useState("#6495ed");
  const [endColor, setEndColor] = useState("#dc143c");

  // Generate final JSX code for saving.
  const generateJsxCode = () => {
    if (!componentName.trim()) {
      alert("Please enter a component name!");
      return "";
    }
    if (componentSet === "gauge") {
      const valueLabelStyleObj = {
        fontSize: valueLabelFontSize,
        fill: valueLabelFill,
        textShadow: valueLabelTextShadow,
      };
      const tickDefaultStyleObj = {
        fontSize: tickDefaultFontSize,
        fill: tickDefaultFill,
        textShadow: tickDefaultTextShadow,
      };
      return `
import React from "react";
import { GaugeComponent } from "react-gauge-component";

const ${componentName} = () => (
  <GaugeComponent
    type="${gaugeType}"
    id="${gaugeId}"
    className="${gaugeClassName}"
    value={${value.trim() === "" || isNaN(Number(value)) ? 33 : Number(value)}}
    minValue={${minValue.trim() === "" || isNaN(Number(minValue)) ? 0 : Number(minValue)}}
    maxValue={${maxValue.trim() === "" || isNaN(Number(maxValue)) ? 100 : Number(maxValue)}}
    arc={{
      cornerRadius: ${cornerRadius},
      padding: ${arcPadding},
      width: ${arcWidth},
      nbSubArcs: ${nbSubArcs ? Number(nbSubArcs) : "undefined"},
      colorArray: ${JSON.stringify(colorArray.split(","))},
      emptyColor: "${emptyColor}",
      gradient: ${gradient},
      subArcs: ${subArcs ? JSON.stringify(parseCommaSeparatedListToObjects(subArcs, "limit"), null, 2) : "undefined"}
    }}
    pointer={{
      type: "${pointerType}",
      hide: ${pointerHide},
      color: "${pointerColor}",
      baseColor: "${pointerBaseColor}",
      length: ${pointerLength},
      animate: ${pointerAnimate},
      elastic: ${pointerElastic},
      animationDuration: ${pointerAnimationDuration},
      animationDelay: ${pointerAnimationDelay},
      width: ${pointerWidth},
      strokeWidth: ${pointerStrokeWidth}
    }}
    labels={{
      valueLabel: {
        matchColorWithArc: ${valueLabelMatchColor},
        formatTextValue: ${valueLabelFormat || "undefined"},
        style: ${JSON.stringify(valueLabelStyleObj, null, 2)},
        maxDecimalDigits: ${valueLabelMaxDecimalDigits},
        hide: ${valueLabelHide}
      },
      tickLabels: {
        type: "${tickLabelsType}",
        hideMinMax: ${tickLabelsHideMinMax},
        ticks: ${tickLabelsTicks ? JSON.stringify(parseCommaSeparatedListToObjects(tickLabelsTicks, "value"), null, 2) : "undefined"},
        defaultTickValueConfig: {
          formatTextValue: ${tickDefaultFormat ? `(${tickDefaultFormat})` : "undefined"},
          style: ${JSON.stringify(tickDefaultStyleObj, null, 2)},
          maxDecimalDigits: ${tickDefaultMaxDecimalDigits},
          hide: ${tickDefaultHide}
        },
        defaultTickLineConfig: {
          width: ${tickLineWidth},
          length: ${tickLineLength},
          color: "${tickLineColor}",
          distanceFromArc: ${tickLineDistance},
          hide: ${tickLineHide}
        }
      }
    }}
  />
);

export default ${componentName};
      `.trim();
    } else {
      return `
import React, { useState } from "react";
import LiquidFillGauge from "react-liquid-gauge";
import { color } from "d3-color";
import { interpolateRgb } from "d3-interpolate";

const ${componentName} = () => {
  const [value, setValue] = useState(${liquidValue});
  const radius = ${radius};
  const startColor = "${startColor}";
  const endColor = "${endColor}";
  const interpolate = interpolateRgb(startColor, endColor);
  const fillColor = interpolate(value / 100);
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
    <LiquidFillGauge
      style={{ margin: "0 auto" }}
      width={radius * 2}
      height={radius * 2}
      value={value}
      percent="%"
      textSize={${textSize}}
      textOffsetX={${textOffsetX}}
      textOffsetY={${textOffsetY}}
      textRenderer={(props) => {
        const value = Math.round(props.value);
        const r = Math.min(props.height / 2, props.width / 2);
        const textPixels = (props.textSize * r / 2);
        const valueStyle = { fontSize: textPixels };
        const percentStyle = { fontSize: textPixels * 0.6 };
        return (
          <tspan>
            <tspan className="value" style={valueStyle}>{value}</tspan>
            <tspan style={percentStyle}>{props.percent}</tspan>
          </tspan>
        );
      }}
      riseAnimation={${riseAnimation}}
      waveAnimation={${waveAnimation}}
      waveFrequency={${waveFrequency}}
      waveAmplitude={${waveAmplitude}}
      gradient={${liquidGradient}}
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
      onClick={() => {
        setValue(Math.random() * 100);
      }}
    />
  );
};

export default ${componentName};
      `.trim();
    }
  };

  const saveComponent = async () => {
    const jsxCode = generateJsxCode();
    if (!jsxCode) return;
    try {
      const response = await fetch("http://localhost:5000/save-component", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ componentName, code: jsxCode }),
      });
      const result = await response.json();
      alert(result.message);
    } catch (error) {
      console.error("Error saving component:", error);
      alert("Error saving component");
    }
  };

  // Computed values for the Gauge preview.
  const computedValue = value.trim() === "" || isNaN(Number(value)) ? 33 : Number(value);
  const computedMinValue = minValue.trim() === "" || isNaN(Number(minValue)) ? 0 : Number(minValue);
  const computedMaxValue = maxValue.trim() === "" || isNaN(Number(maxValue)) ? 100 : Number(maxValue);

  return (
    <Allotment style={{ height: "100vh", fontFamily: "Arial, sans-serif" }}>
      <Allotment.Pane minSize={300} maxSize={500}>
        <div className="scroll-container" style={{ paddingRight: 4, overflowY: "auto", height: "100%" }}>
          <div style={{ marginBottom: "20px" }}>
            <label>Select Component Type: </label>
            <select value={componentSet} onChange={(e) => setComponentSet(e.target.value)}>
              <option value="gauge">Gauge</option>
              <option value="liquid">Liquid Gauge</option>
            </select>
          </div>
          {componentSet === "gauge" ? (
            <GaugeForm
              fieldStyle={fieldStyle}
              componentName={componentName}
              setComponentName={setComponentName}
              value={value}
              setValue={setValue}
              minValue={minValue}
              setMinValue={setMinValue}
              maxValue={maxValue}
              setMaxValue={setMaxValue}
              gaugeType={gaugeType}
              setGaugeType={setGaugeType}
              gaugeId={gaugeId}
              setGaugeId={setGaugeId}
              gaugeClassName={gaugeClassName}
              setGaugeClassName={setGaugeClassName}
              cornerRadius={cornerRadius}
              setCornerRadius={setCornerRadius}
              arcPadding={arcPadding}
              setArcPadding={setArcPadding}
              arcWidth={arcWidth}
              setArcWidth={setArcWidth}
              nbSubArcs={nbSubArcs}
              setNbSubArcs={setNbSubArcs}
              colorArray={colorArray}
              setColorArray={setColorArray}
              emptyColor={emptyColor}
              setEmptyColor={setEmptyColor}
              gradient={gradient}
              setGradient={setGradient}
              subArcs={subArcs}
              setSubArcs={setSubArcs}
              pointerType={pointerType}
              setPointerType={setPointerType}
              pointerHide={pointerHide}
              setPointerHide={setPointerHide}
              pointerColor={pointerColor}
              setPointerColor={setPointerColor}
              pointerBaseColor={pointerBaseColor}
              setPointerBaseColor={setPointerBaseColor}
              pointerLength={pointerLength}
              setPointerLength={setPointerLength}
              pointerAnimate={pointerAnimate}
              setPointerAnimate={setPointerAnimate}
              pointerElastic={pointerElastic}
              setPointerElastic={setPointerElastic}
              pointerAnimationDuration={pointerAnimationDuration}
              setPointerAnimationDuration={setPointerAnimationDuration}
              pointerAnimationDelay={pointerAnimationDelay}
              setPointerAnimationDelay={setPointerAnimationDelay}
              pointerWidth={pointerWidth}
              setPointerWidth={setPointerWidth}
              pointerStrokeWidth={pointerStrokeWidth}
              setPointerStrokeWidth={setPointerStrokeWidth}
              valueLabelMatchColor={valueLabelMatchColor}
              setValueLabelMatchColor={setValueLabelMatchColor}
              valueLabelFormat={valueLabelFormat}
              setValueLabelFormat={setValueLabelFormat}
              valueLabelFontSize={valueLabelFontSize}
              setValueLabelFontSize={setValueLabelFontSize}
              valueLabelFill={valueLabelFill}
              setValueLabelFill={setValueLabelFill}
              valueLabelTextShadow={valueLabelTextShadow}
              setValueLabelTextShadow={setValueLabelTextShadow}
              valueLabelMaxDecimalDigits={valueLabelMaxDecimalDigits}
              setValueLabelMaxDecimalDigits={setValueLabelMaxDecimalDigits}
              valueLabelHide={valueLabelHide}
              setValueLabelHide={setValueLabelHide}
              tickLabelsType={tickLabelsType}
              setTickLabelsType={setTickLabelsType}
              tickLabelsHideMinMax={tickLabelsHideMinMax}
              setTickLabelsHideMinMax={setTickLabelsHideMinMax}
              tickLabelsTicks={tickLabelsTicks}
              setTickLabelsTicks={setTickLabelsTicks}
              tickDefaultFormat={tickDefaultFormat}
              setTickDefaultFormat={setTickDefaultFormat}
              tickDefaultFontSize={tickDefaultFontSize}
              setTickDefaultFontSize={setTickDefaultFontSize}
              tickDefaultFill={tickDefaultFill}
              setTickDefaultFill={setTickDefaultFill}
              tickDefaultTextShadow={tickDefaultTextShadow}
              setTickDefaultTextShadow={setTickDefaultTextShadow}
              tickDefaultMaxDecimalDigits={tickDefaultMaxDecimalDigits}
              setTickDefaultMaxDecimalDigits={setTickDefaultMaxDecimalDigits}
              tickDefaultHide={tickDefaultHide}
              setTickDefaultHide={setTickDefaultHide}
              tickLineWidth={tickLineWidth}
              setTickLineWidth={setTickLineWidth}
              tickLineLength={tickLineLength}
              setTickLineLength={setTickLineLength}
              tickLineColor={tickLineColor}
              setTickLineColor={setTickLineColor}
              tickLineDistance={tickLineDistance}
              setTickLineDistance={setTickLineDistance}
              tickLineHide={tickLineHide}
              setTickLineHide={setTickLineHide}
              saveComponent={saveComponent}
            />
          ) : (
            <LiquidGaugeForm
              fieldStyle={fieldStyle}
              componentName={componentName}
              setComponentName={setComponentName}
              liquidValue={liquidValue}
              setLiquidValue={setLiquidValue}
              radius={radius}
              setRadius={setRadius}
              textSize={textSize}
              setTextSize={setTextSize}
              textOffsetX={textOffsetX}
              setTextOffsetX={setTextOffsetX}
              textOffsetY={textOffsetY}
              setTextOffsetY={setTextOffsetY}
              riseAnimation={riseAnimation}
              setRiseAnimation={setRiseAnimation}
              waveAnimation={waveAnimation}
              setWaveAnimation={setWaveAnimation}
              waveFrequency={waveFrequency}
              setWaveFrequency={setWaveFrequency}
              waveAmplitude={waveAmplitude}
              setWaveAmplitude={setWaveAmplitude}
              gradient={liquidGradient}
              setGradient={setLiquidGradient}
              startColor={startColor}
              setStartColor={setStartColor}
              endColor={endColor}
              setEndColor={setEndColor}
              saveComponent={saveComponent}
            />
          )}
        </div>
      </Allotment.Pane>
      <Allotment.Pane>
        <ErrorBoundary>
        {componentSet === "gauge" ? (
          <GaugePreview
            gaugeType={gaugeType}
            gaugeId={gaugeId}
            gaugeClassName={gaugeClassName}
            computedValue={computedValue}
            computedMinValue={computedMinValue}
            computedMaxValue={computedMaxValue}
            cornerRadius={cornerRadius}
            arcPadding={arcPadding}
            arcWidth={arcWidth}
            nbSubArcs={nbSubArcs}
            colorArray={colorArray}
            emptyColor={emptyColor}
            gradient={gradient}
            subArcs={subArcs}
            pointerType={pointerType}
            pointerHide={pointerHide}
            pointerColor={pointerColor}
            pointerBaseColor={pointerBaseColor}
            pointerLength={pointerLength}
            pointerAnimate={pointerAnimate}
            pointerElastic={pointerElastic}
            pointerAnimationDuration={pointerAnimationDuration}
            pointerAnimationDelay={pointerAnimationDelay}
            pointerWidth={pointerWidth}
            pointerStrokeWidth={pointerStrokeWidth}
            valueLabelMatchColor={valueLabelMatchColor}
            valueLabelFormat={valueLabelFormat}
            valueLabelFontSize={valueLabelFontSize}
            valueLabelFill={valueLabelFill}
            valueLabelTextShadow={valueLabelTextShadow}
            valueLabelMaxDecimalDigits={valueLabelMaxDecimalDigits}
            valueLabelHide={valueLabelHide}
            tickLabelsType={tickLabelsType}
            tickLabelsHideMinMax={tickLabelsHideMinMax}
            tickLabelsTicks={tickLabelsTicks}
            tickDefaultFormat={tickDefaultFormat}
            tickDefaultFontSize={tickDefaultFontSize}
            tickDefaultFill={tickDefaultFill}
            tickDefaultTextShadow={tickDefaultTextShadow}
            tickDefaultMaxDecimalDigits={tickDefaultMaxDecimalDigits}
            tickDefaultHide={tickDefaultHide}
            tickLineWidth={tickLineWidth}
            tickLineLength={tickLineLength}
            tickLineColor={tickLineColor}
            tickLineDistance={tickLineDistance}
            tickLineHide={tickLineHide}
            parseCommaSeparatedListToObjects={parseCommaSeparatedListToObjects}
            parseFunction={parseFunction}
          />
        ) : (
          <LiquidGaugePreview
            liquidValue={liquidValue}
            radius={radius}
            textSize={textSize}
            textOffsetX={textOffsetX}
            textOffsetY={textOffsetY}
            riseAnimation={riseAnimation}
            waveAnimation={waveAnimation}
            waveFrequency={waveFrequency}
            waveAmplitude={waveAmplitude}
            gradient={liquidGradient}
            startColor={startColor}
            endColor={endColor}
          />
        )}
        </ErrorBoundary>
      </Allotment.Pane>
    </Allotment>
  );
}



export default App;
