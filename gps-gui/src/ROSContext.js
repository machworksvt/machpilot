// src/ROSContext.js
import React, { createContext, useState, useEffect } from 'react';
import ros from './rosConnection';

// Create a context with a default value
export const ROSContext = createContext({
  connectionStatus: 'Disconnected'
});

// Provider component to wrap your app
export const ROSProvider = ({ children }) => {
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');

  useEffect(() => {
    // Handlers for connection events
    const handleConnection = () => setConnectionStatus('Connected');
    const handleError = () => setConnectionStatus('Error');
    const handleClose = () => setConnectionStatus('Disconnected');

    // Add event listeners
    ros.on('connection', handleConnection);
    ros.on('error', handleError);
    ros.on('close', handleClose);

    // Check immediately if connection is already open
    if (ros.socket && ros.socket.readyState === WebSocket.OPEN) {
      setConnectionStatus('Connected');
    }

    // Cleanup listeners on unmount
    return () => {
      ros.off('connection', handleConnection);
      ros.off('error', handleError);
      ros.off('close', handleClose);
    };
  }, []);

  return (
    <ROSContext.Provider value={{ connectionStatus }}>
      {children}
    </ROSContext.Provider>
  );
};
