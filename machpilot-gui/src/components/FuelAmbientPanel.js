// src/components/FuelAmbientPanel.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';

/**
 * FuelAmbientPanel displays fuel and ambient data from the engine.
 * It subscribes to the '/h20pro/fuel_ambient' topic and expects the
 * custom message type 'interfaces/msg/FuelAmbient'.
 */
const FuelAmbientPanel = () => {
  const fuelAmbient = useROSSubscription('/h20pro/fuel_ambient', 'interfaces/msg/FuelAmbient');

  return (
    <div className="panel" style={panelStyle}>
      <h3>Fuel & Ambient</h3>
      {fuelAmbient ? (
        <div>
          <p><strong>Fuel Flow:</strong> {fuelAmbient.fuel_flow} ml/min</p>
          <p><strong>Fuel Consumed:</strong> {fuelAmbient.fuel_consumed} ml</p>
          <p><strong>Engine Box Pressure:</strong> {fuelAmbient.engine_box_pressure} mbar</p>
          <p><strong>Ambient Temp:</strong> {fuelAmbient.ambient_temperature} °C</p>
        </div>
      ) : (
        <p>Waiting for fuel/ambient data...</p>
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

export default FuelAmbientPanel;
