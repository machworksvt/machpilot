// src/components/EngineDataDumpPanel.js
import React from 'react';

const EngineDataDumpPanel = ({
  engineData,
  pumpRpm,
  throttle,
  errors,
  fuelAmbient,
  glowPlugs,
  lastRunInfo,
  ngReg,
  statistics,
  systemInfo,
  systemInfo2,
  voltageCurrent,
}) => {
  // Combine all the data into a single object for display.
  const allData = {
    engineData,
    pumpRpm,
    throttle,
    errors,
    fuelAmbient,
    glowPlugs,
    lastRunInfo,
    ngReg,
    statistics,
    systemInfo,
    systemInfo2,
    voltageCurrent,
  };

  return (
    <div style={panelStyle}>
      <h2>Engine Data Dump</h2>
      <pre style={preStyle}>
        {JSON.stringify(allData, null, 2)}
      </pre>
    </div>
  );
};

const panelStyle = {
  background: '#fff',
  border: '1px solid #ddd',
  borderRadius: '4px',
  padding: '15px',
  margin: '15px',
  overflowY: 'auto',
  maxHeight: '100%', // Adjust as needed for your layout
};

const preStyle = {
  fontSize: '12px',
  textAlign: 'left',
  whiteSpace: 'pre-wrap',
};

export default EngineDataDumpPanel;
