import React from "react";
import "../styles.css";

const GaugeForm = (props) => {
  // Destructure all props needed by the form
  const {
    fieldStyle,
    componentName,
    setComponentName,
    value,
    setValue,
    minValue,
    setMinValue,
    maxValue,
    setMaxValue,
    gaugeType,
    setGaugeType,
    gaugeId,
    setGaugeId,
    gaugeClassName,
    setGaugeClassName,
    cornerRadius,
    setCornerRadius,
    arcPadding,
    setArcPadding,
    arcWidth,
    setArcWidth,
    nbSubArcs,
    setNbSubArcs,
    colorArray,
    setColorArray,
    emptyColor,
    setEmptyColor,
    gradient,
    setGradient,
    subArcs,
    setSubArcs,
    pointerType,
    setPointerType,
    pointerHide,
    setPointerHide,
    pointerColor,
    setPointerColor,
    pointerBaseColor,
    setPointerBaseColor,
    pointerLength,
    setPointerLength,
    pointerAnimate,
    setPointerAnimate,
    pointerElastic,
    setPointerElastic,
    pointerAnimationDuration,
    setPointerAnimationDuration,
    pointerAnimationDelay,
    setPointerAnimationDelay,
    pointerWidth,
    setPointerWidth,
    pointerStrokeWidth,
    setPointerStrokeWidth,
    valueLabelMatchColor,
    setValueLabelMatchColor,
    valueLabelFormat,
    setValueLabelFormat,
    valueLabelFontSize,
    setValueLabelFontSize,
    valueLabelFill,
    setValueLabelFill,
    valueLabelTextShadow,
    setValueLabelTextShadow,
    valueLabelMaxDecimalDigits,
    setValueLabelMaxDecimalDigits,
    valueLabelHide,
    setValueLabelHide,
    tickLabelsType,
    setTickLabelsType,
    tickLabelsHideMinMax,
    setTickLabelsHideMinMax,
    tickLabelsTicks,
    setTickLabelsTicks,
    tickDefaultFormat,
    setTickDefaultFormat,
    tickDefaultFontSize,
    setTickDefaultFontSize,
    tickDefaultFill,
    setTickDefaultFill,
    tickDefaultTextShadow,
    setTickDefaultTextShadow,
    tickDefaultMaxDecimalDigits,
    setTickDefaultMaxDecimalDigits,
    tickDefaultHide,
    setTickDefaultHide,
    tickLineWidth,
    setTickLineWidth,
    tickLineLength,
    setTickLineLength,
    tickLineColor,
    setTickLineColor,
    tickLineDistance,
    setTickLineDistance,
    tickLineHide,
    setTickLineHide,
    saveComponent,
  } = props;

  return (
    <div style={{ padding: "20px", overflowY: "auto", height: "100%" }}>
      <h2 style={{ marginBottom: "20px" }}>Gauge Component Creator</h2>
      <div className="form-container">
        <form style={{ display: "flex", flexDirection: "column", gap: "10px" }}>
          {/* COMPONENT NAME */}
          <div style={fieldStyle}>
            <label>Component Name:</label>
            <input
              type="text"
              value={componentName}
              onChange={(e) => setComponentName(e.target.value)}
              placeholder="Enter component name"
            />
          </div>
          {/* GENERAL PROPS */}
          <fieldset style={{ border: "none" }}>
            <legend>General Props</legend>
            <div style={fieldStyle}>
              <label>Type:</label>
              <select value={gaugeType} onChange={(e) => setGaugeType(e.target.value)}>
                <option value="grafana">Grafana</option>
                <option value="semicircle">Semicircle</option>
                <option value="radial">Radial</option>
              </select>
            </div>
            <div style={fieldStyle}>
              <label>ID:</label>
              <input type="text" value={gaugeId} onChange={(e) => setGaugeId(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Class Name:</label>
              <input type="text" value={gaugeClassName} onChange={(e) => setGaugeClassName(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Value:</label>
              <input type="number" value={value} onChange={(e) => setValue(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Min Value:</label>
              <input type="number" value={minValue} onChange={(e) => setMinValue(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Max Value:</label>
              <input type="number" value={maxValue} onChange={(e) => setMaxValue(e.target.value)} />
            </div>
          </fieldset>
          {/* ARC PROPS */}
          <fieldset style={{ border: "none" }}>
            <legend>Arc Props</legend>
            <div style={fieldStyle}>
              <label>Corner Radius:</label>
              <input
                type="number"
                value={cornerRadius}
                onChange={(e) => setCornerRadius(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Padding:</label>
              <input
                type="number"
                step="0.01"
                value={arcPadding}
                onChange={(e) => setArcPadding(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Width:</label>
              <input
                type="number"
                step="0.01"
                value={arcWidth}
                onChange={(e) => setArcWidth(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Number of SubArcs (optional):</label>
              <input type="text" value={nbSubArcs} onChange={(e) => setNbSubArcs(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Color Array (comma-separated):</label>
              <input type="text" value={colorArray} onChange={(e) => setColorArray(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Empty Color:</label>
              <input type="color" value={emptyColor} onChange={(e) => setEmptyColor(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Gradient:</label>
              <input type="checkbox" checked={gradient} onChange={(e) => setGradient(e.target.checked)} />
            </div>
            <div style={fieldStyle}>
              <label>SubArcs (comma-separated limits):</label>
              <input
                type="text"
                value={subArcs}
                onChange={(e) => setSubArcs(e.target.value)}
                placeholder="e.g. 20,40,60"
              />
            </div>
          </fieldset>
          {/* POINTER PROPS */}
          <fieldset style={{ border: "none" }}>
            <legend>Pointer Props</legend>
            <div style={fieldStyle}>
              <label>Type:</label>
              <select value={pointerType} onChange={(e) => setPointerType(e.target.value)}>
                <option value="needle">Needle</option>
                <option value="blob">Blob</option>
                <option value="arrow">Arrow</option>
              </select>
            </div>
            <div style={fieldStyle}>
              <label>Hide Pointer:</label>
              <input
                type="checkbox"
                checked={pointerHide}
                onChange={(e) => setPointerHide(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Color:</label>
              <input type="color" value={pointerColor} onChange={(e) => setPointerColor(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Base Color:</label>
              <input type="color" value={pointerBaseColor} onChange={(e) => setPointerBaseColor(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Length:</label>
              <input
                type="number"
                step="0.01"
                value={pointerLength}
                onChange={(e) => setPointerLength(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Animate Pointer:</label>
              <input
                type="checkbox"
                checked={pointerAnimate}
                onChange={(e) => setPointerAnimate(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Elastic Pointer:</label>
              <input
                type="checkbox"
                checked={pointerElastic}
                onChange={(e) => setPointerElastic(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Animation Duration:</label>
              <input
                type="number"
                value={pointerAnimationDuration}
                onChange={(e) => setPointerAnimationDuration(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Animation Delay:</label>
              <input
                type="number"
                value={pointerAnimationDelay}
                onChange={(e) => setPointerAnimationDelay(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Pointer Width:</label>
              <input
                type="number"
                value={pointerWidth}
                onChange={(e) => setPointerWidth(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Stroke Width (for blob):</label>
              <input
                type="number"
                value={pointerStrokeWidth}
                onChange={(e) => setPointerStrokeWidth(Number(e.target.value))}
              />
            </div>
          </fieldset>
          {/* LABELS PROPS */}
          <fieldset style={{ border: "none" }}>
            <legend>Labels Props</legend>
            <h4>Value Label</h4>
            <div style={fieldStyle}>
              <label>Match Color With Arc:</label>
              <input
                type="checkbox"
                checked={valueLabelMatchColor}
                onChange={(e) => setValueLabelMatchColor(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Format Text Value (function as string):</label>
              <input
                type="text"
                value={valueLabelFormat}
                onChange={(e) => setValueLabelFormat(e.target.value)}
                placeholder="e.g. value => value + 'ºC'"
              />
            </div>
            <div style={fieldStyle}>
              <label>Font Size:</label>
              <input
                type="text"
                value={valueLabelFontSize}
                onChange={(e) => setValueLabelFontSize(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Fill Color:</label>
              <input type="color" value={valueLabelFill} onChange={(e) => setValueLabelFill(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Text Shadow:</label>
              <input
                type="text"
                value={valueLabelTextShadow}
                onChange={(e) => setValueLabelTextShadow(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Max Decimal Digits:</label>
              <input
                type="number"
                value={valueLabelMaxDecimalDigits}
                onChange={(e) => setValueLabelMaxDecimalDigits(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Hide Value Label:</label>
              <input
                type="checkbox"
                checked={valueLabelHide}
                onChange={(e) => setValueLabelHide(e.target.checked)}
              />
            </div>
            <h4>Tick Labels</h4>
            <div style={fieldStyle}>
              <label>Type:</label>
              <select value={tickLabelsType} onChange={(e) => setTickLabelsType(e.target.value)}>
                <option value="outer">Outer</option>
                <option value="inner">Inner</option>
              </select>
            </div>
            <div style={fieldStyle}>
              <label>Hide Min/Max:</label>
              <input
                type="checkbox"
                checked={tickLabelsHideMinMax}
                onChange={(e) => setTickLabelsHideMinMax(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Ticks (comma-separated values):</label>
              <input
                type="text"
                value={tickLabelsTicks}
                onChange={(e) => setTickLabelsTicks(e.target.value)}
                placeholder="e.g. 20,40,60"
              />
            </div>
            <h5>Default Tick Value Config</h5>
            <div style={fieldStyle}>
              <label>Format Text Value (function as string):</label>
              <input
                type="text"
                value={tickDefaultFormat}
                onChange={(e) => setTickDefaultFormat(e.target.value)}
                placeholder="e.g. value => value"
              />
            </div>
            <div style={fieldStyle}>
              <label>Font Size:</label>
              <input
                type="text"
                value={tickDefaultFontSize}
                onChange={(e) => setTickDefaultFontSize(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Fill Color:</label>
              <input type="color" value={tickDefaultFill} onChange={(e) => setTickDefaultFill(e.target.value)} />
            </div>
            <div style={fieldStyle}>
              <label>Text Shadow:</label>
              <input
                type="text"
                value={tickDefaultTextShadow}
                onChange={(e) => setTickDefaultTextShadow(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Max Decimal Digits:</label>
              <input
                type="number"
                value={tickDefaultMaxDecimalDigits}
                onChange={(e) => setTickDefaultMaxDecimalDigits(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Hide Tick Value:</label>
              <input
                type="checkbox"
                checked={tickDefaultHide}
                onChange={(e) => setTickDefaultHide(e.target.checked)}
              />
            </div>
            <h5>Default Tick Line Config</h5>
            <div style={fieldStyle}>
              <label>Width:</label>
              <input
                type="number"
                step="0.01"
                value={tickLineWidth}
                onChange={(e) => setTickLineWidth(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Length:</label>
              <input
                type="number"
                step="0.01"
                value={tickLineLength}
                onChange={(e) => setTickLineLength(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Color:</label>
              <input
                type="text"
                value={tickLineColor}
                onChange={(e) => setTickLineColor(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Distance From Arc:</label>
              <input
                type="number"
                step="0.01"
                value={tickLineDistance}
                onChange={(e) => setTickLineDistance(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Hide Tick Line:</label>
              <input
                type="checkbox"
                checked={tickLineHide}
                onChange={(e) => setTickLineHide(e.target.checked)}
              />
            </div>
          </fieldset>
        </form>
      </div>
      <button
  onClick={saveComponent}
  className="save-component-button"
>
  Save Component
</button>
    </div>
  );
};

export default GaugeForm;
