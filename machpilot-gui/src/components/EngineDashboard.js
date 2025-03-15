// src/components/EngineDashboard.js
import React from 'react';
import EngineDataPanel from './EngineDataPanel';
import VoltageCurrentPanel from './VoltageCurrentPanel';
import ThrottleControl from './ThrottleControl';
import FuelAmbientPanel from './FuelAmbientPanel';
import ErrorsPanel from './ErrorsPanel';
import StarterTestPanel from './StarterTestPanel';

const EngineDashboard = () => {
  return (
    <div className="engine-dashboard" style={dashboardStyle}>
      <div style={gridStyle}>
        <EngineDataPanel />
        <VoltageCurrentPanel />
        <FuelAmbientPanel />
        <ErrorsPanel />
        <ThrottleControl />
        <StarterTestPanel />
        {/* Add additional panels as you create them */}
      </div>
    </div>
  );
};

const dashboardStyle = {
  padding: '20px',
  fontFamily: 'Arial, sans-serif'
};

const gridStyle = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))',
  gridGap: '15px'
};

export default EngineDashboard;