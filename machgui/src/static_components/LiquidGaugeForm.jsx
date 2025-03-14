import React from "react";
import "../styles.css";

const LiquidGaugeForm = (props) => {
  const {
    fieldStyle,
    componentName,
    setComponentName,
    liquidValue,
    setLiquidValue,
    radius,
    setRadius,
    textSize,
    setTextSize,
    textOffsetX,
    setTextOffsetX,
    textOffsetY,
    setTextOffsetY,
    riseAnimation,
    setRiseAnimation,
    waveAnimation,
    setWaveAnimation,
    waveFrequency,
    setWaveFrequency,
    waveAmplitude,
    setWaveAmplitude,
    gradient,
    setGradient,
    startColor,
    setStartColor,
    endColor,
    setEndColor,
    saveComponent,
  } = props;

  return (
    <div style={{ padding: "20px", overflowY: "auto", height: "100%" }}>
      <h2 style={{ marginBottom: "20px" }}>Liquid Gauge Component Creator</h2>
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
          <fieldset style={{ border: "none" }}>
            <legend>Liquid Gauge Props</legend>
            <div style={fieldStyle}>
              <label>Value (0-100):</label>
              <input
                type="number"
                value={liquidValue}
                onChange={(e) => setLiquidValue(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Radius:</label>
              <input
                type="number"
                value={radius}
                onChange={(e) => setRadius(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Text Size (relative):</label>
              <input
                type="number"
                step="0.1"
                value={textSize}
                onChange={(e) => setTextSize(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Text Offset X:</label>
              <input
                type="number"
                value={textOffsetX}
                onChange={(e) => setTextOffsetX(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Text Offset Y:</label>
              <input
                type="number"
                value={textOffsetY}
                onChange={(e) => setTextOffsetY(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Rise Animation:</label>
              <input
                type="checkbox"
                checked={riseAnimation}
                onChange={(e) => setRiseAnimation(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Wave Animation:</label>
              <input
                type="checkbox"
                checked={waveAnimation}
                onChange={(e) => setWaveAnimation(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Wave Frequency:</label>
              <input
                type="number"
                step="0.1"
                value={waveFrequency}
                onChange={(e) => setWaveFrequency(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Wave Amplitude:</label>
              <input
                type="number"
                step="0.1"
                value={waveAmplitude}
                onChange={(e) => setWaveAmplitude(Number(e.target.value))}
              />
            </div>
            <div style={fieldStyle}>
              <label>Gradient:</label>
              <input
                type="checkbox"
                checked={gradient}
                onChange={(e) => setGradient(e.target.checked)}
              />
            </div>
            <div style={fieldStyle}>
              <label>Start Color:</label>
              <input
                type="color"
                value={startColor}
                onChange={(e) => setStartColor(e.target.value)}
              />
            </div>
            <div style={fieldStyle}>
              <label>End Color:</label>
              <input
                type="color"
                value={endColor}
                onChange={(e) => setEndColor(e.target.value)}
              />
            </div>
          </fieldset>
        </form>
      </div>
      <button onClick={saveComponent} className="save-component-button">
        Save Component
      </button>
    </div>
  );
};

export default LiquidGaugeForm;
