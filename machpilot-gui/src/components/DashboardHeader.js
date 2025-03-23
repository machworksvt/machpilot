// src/components/DashboardHeader.js
import React, { useContext } from 'react';
import { ROSContext } from '../ROSContext'; // Assuming you're using context for connection status
import logo from '../assets/Icarus_Flat_Banner_4Kfit_B.png'; // Adjust the path as needed
import refreshIcon from '../assets/refresh-cw-alt-3-svgrepo-com.svg'; // Adjust the path as needed

/**
 * DashboardHeader is a reusable header component for your ground station dashboards.
 * It displays a logo on the left, the vehicle name, the dashboard name, and a connection indicator.
 *
 * Props:
 *   - vehicleName: Name of the vehicle (e.g., "ICARUS")
 *   - dashboardName: Name of the dashboard (e.g., "Engine Dashboard")
 */
const DashboardHeader = ({ vehicleName, dashboardName }) => {
  const { connectionStatus } = useContext(ROSContext);

  const handleRefresh = () => {
    window.location.reload();
  };

  const headerStyle = {
    background: '#b16200',
    color: '#fff',
    padding: '5px 15px',
    display: 'flex',
    alignItems: 'center',
    fontFamily: 'MW Font', // Use your custom font here
  };

  const logoStyle = {
    height: '70px', // adjust size as needed
    marginRight: '15px',
  };

  const titleContainerStyle = {
    flex: '1',
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
    fontWeight: 'bold',
    marginLeft: '10px',
  };

  const iconStyle = {
    width: '25px',
    height: '25px',
  };

  const buttonStyle = {
    backgroundColor: '#007BFF',
    color: 'white',
    border: 'none',
    padding: '2px', // Reduced padding for a smaller button
    borderRadius: '4px',
    cursor: 'pointer',
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center', // Center the icon inside the button
    marginRight: '10px',
  };

  // Container for refresh button and status indicator
  const refreshStatusContainerStyle = {
    display: 'flex',
    alignItems: 'center',
  };

  return (
    <header style={headerStyle}>
      <img src={logo} alt="Logo" style={logoStyle} />
      <div style={titleContainerStyle}>
        <h1 style={{ margin: 40 }}>{dashboardName}</h1>
      </div>
      <div style={refreshStatusContainerStyle}>
        {connectionStatus !== 'Connected' && (
          <button onClick={handleRefresh} style={buttonStyle}>
            <img src={refreshIcon} alt="Refresh" style={iconStyle} />
          </button>
        )}
        <span style={statusStyle}>{connectionStatus}</span>
      </div>
    </header>
  );
};

export default DashboardHeader;