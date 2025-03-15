import React from 'react';
import GaugeComponent from 'react-gauge-component'; // Ensure this is the ROS2-compatible gauge component

/**
 * EngineRpmGauge displays the engine RPM using a gauge with three custom segments.
 *
 * Props:
 *   - rpm: The current RPM (a number between 0 and 125000)
 *
 * Note: The gauge is divided into three sections:
 *   • 0 to 6,700: Dark orange (#FF8C00)
 *   • 6,700 to 37,000: Black (#000000)
 *   • 37,000 to 125,000: White (#FFFFFF)
 * The needle is thin and red, and the label shows the RPM in thousands with two decimals.
 */
const EngineRpmGauge = ({ rpm }) => {
  return (
    <div style={gaugePanelStyle}>
      <h4>Engine RPM</h4>
      <GaugeComponent
        value={rpm}
        min={0}
        max={125000}
        // Define segments with custom start and end values and team colors
        segments={[
          { start: 0, end: 6700, color: "#FF8C00" },
          { start: 6700, end: 37000, color: "#000000" },
          { start: 37000, end: 125000, color: "#FFFFFF" }
        ]}
        // Set the needle style: thin and red.
        needle={{ color: "#FF0000", width: 2 }}
        // Format the labels to show RPM in thousands with 2 decimals.
        labelFormatter={(val) => `${(val / 1000).toFixed(2)}k RPM`}
        style={{ width: '250px', height: '250px' }}
      />
    </div>
  );
};

const gaugePanelStyle = {
  background: '#f4f4f4',
  border: '1px solid #ddd',
  padding: '15px',
  margin: '15px',
  borderRadius: '4px',
  textAlign: 'center'
};

export default EngineRpmGauge;
