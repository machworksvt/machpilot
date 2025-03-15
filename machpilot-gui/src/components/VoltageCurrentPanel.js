// src/components/VoltageCurrentPanel.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';

// Component for displaying voltage and current data
const VoltageCurrentPanel = () => {
  const voltageCurrent = useROSSubscription('/h20pro/voltage_current', 'interfaces/msg/VoltageCurrent');

  return (
    <div className="panel voltage-current-panel">
      <h3>Voltage & Current</h3>
      {voltageCurrent ? (
        <div>
          <p><strong>Battery Voltage:</strong> {voltageCurrent.battery_voltage} V</p>
          <p><strong>Engine Current:</strong> {voltageCurrent.battery_current} A</p>
          <p><strong>Flags:</strong> {voltageCurrent.flags}</p>
        </div>
      ) : (
        <p>Waiting for voltage/current data...</p>
      )}
    </div>
  );
};

export default VoltageCurrentPanel;
