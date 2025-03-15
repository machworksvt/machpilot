// src/components/DashboardHeader.js
import React, { useContext } from 'react';
import { ROSContext } from '../ROSContext';

/**
 * DashboardHeader is a reusable header component for your ground station dashboards.
 * It displays the vehicle name, dashboard name, and a connection status indicator.
 *
 * Props:
 *   - vehicleName: Name of the vehicle (e.g., "ICARUS")
 *   - dashboardName: Name of the dashboard (e.g., "Engine Dashboard")
 */
const DashboardHeader = ({ vehicleName, dashboardName }) => {
  const { connectionStatus } = useContext(ROSContext);

  // Inline styles for the header
  const headerStyle = {
    background: '#333',
    color: '#fff',
    padding: '15px 25px',
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    fontFamily: 'Arial, sans-serif'
  };

  const titleStyle = {
    margin: 0
  };

  const statusStyle = {
    padding: '8px 12px',
    borderRadius: '5px',
    background:
      connectionStatus === 'Connected'
        ? 'green'
        : connectionStatus === 'Error'
        ? 'orange'
        : 'red',
    color: '#fff',
    fontWeight: 'bold'
  };

  return (
    <header style={headerStyle}>
      <div>
        <h1 style={titleStyle}>{vehicleName}</h1>
        <h3 style={{ ...titleStyle, fontWeight: 'normal' }}>{dashboardName}</h3>
      </div>
      <div>
        <span style={statusStyle}>{connectionStatus}</span>
      </div>
    </header>
  );
};

export default DashboardHeader;
