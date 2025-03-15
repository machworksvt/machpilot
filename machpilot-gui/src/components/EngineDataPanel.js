// src/components/EngineDataPanel.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';

// Component for displaying engine data
const EngineDataPanel = () => {
  // Subscribe to the engine data topic.
  // The messageType should exactly match your custom message (as defined in your ROS2 package).
  const engineData = useROSSubscription('/h20pro/engine_data', 'interfaces/msg/EngineData');

  return (
    <div className="panel engine-data-panel" style={panelStyle}>
      <h3>Engine Data</h3>
      {engineData ? (
        // When a message is received, display its fields.
        <div>
          <p><strong>Set RPM:</strong> {engineData.set_rpm}</p>
          <p><strong>Real RPM:</strong> {engineData.real_rpm}</p>
          <p><strong>EGT:</strong> {engineData.egt} °C</p>
          <p>
            <strong>State:</strong> {engineData.state_name} (<em>{engineData.state}</em>)
          </p>
          <p><strong>Pump Power:</strong> {engineData.pump_power} %</p>
        </div>
      ) : (
        // Display a waiting message until data is received.
        <p>Waiting for engine data...</p>
      )}
    </div>
  );
};


// A simple inline style for panels (adjust as needed)
const panelStyle = {
  background: '#f4f4f4',
  border: '1px solid #ddd',
  padding: '10px',
  margin: '10px',
  borderRadius: '4px'
};

export default EngineDataPanel;
