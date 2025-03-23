// src/components/PumpTestPanel.js
import React from 'react';
import GenericActionExecutor from './GenericActionExecutor';

const PumpTestPanel = () => {
  // Define fields for PumpTest action.
  const fields = [
    { name: 'pump_power_percent', label: 'Pump Power (%)', type: 'number', defaultValue: 50.0 },
    { name: 'fuel_ml', label: 'Fuel (ml)', type: 'number', defaultValue: 100 },
  ];

  // Optionally, you could supply initial feedback and result.
  const initialFeedback = { fuel_pumped_ml: 0, fuel_pump_rate: 0.0 };
  const initialResult = { success: false };

  return (
    <GenericActionExecutor
      friendlyName="Pump Test Action"
      serverName="/pump_test"
      actionType="interfaces/action/PumpTest"
      fields={fields}
      initialFeedback={initialFeedback}
      initialResult={initialResult}
    />
  );
};

export default PumpTestPanel;
