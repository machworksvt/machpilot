// src/components/EngineDataPanel.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';
import EngineRpmGauge from './RpmGauge';

// Component for displaying engine data
const EngineDataPanel = () => {
  // Subscribe to the engine data topic.
  // The messageType should exactly match your custom message (as defined in your ROS2 package).
  const engineData = useROSSubscription('/h20pro/engine_data', 'interfaces/msg/EngineData');
  const rpm = engineData ? engineData.real_rpm : 0;
  const set_rpm = engineData ? engineData.set_rpm : 0;
  const state = engineData ? engineData.state : 0;
  const state_name = engineData ? engineData.state_name : 'OFF';
  const egt = engineData ? engineData.egt : 0;
  const pump_power = engineData ? engineData.pump_power : 0;


  return (
    <div className="panel engine-data-panel" >
      <h3>Engine Data</h3>
        <div>
          <p><strong>Set RPM:</strong> {set_rpm}</p>
          <p><strong>Real RPM:</strong> {rpm}</p>
          <p><strong>EGT:</strong> {egt} °C</p>
          <p>
            <strong>State:</strong> {state_name} (<em>{state}</em>)
          </p>
          <p><strong>Pump Power:</strong> {pump_power} %</p>
        </div>
    </div>
  );
};

export default EngineDataPanel;
