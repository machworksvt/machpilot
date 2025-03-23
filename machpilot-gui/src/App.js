// src/App.js
import React from 'react';
import { ROSProvider } from './ROSContext';
import DashboardHeader from './components/DashboardHeader';
import EngineDashboard from './components/EngineDashboard';

function App() {
  return (
    <ROSProvider>
      <div className="App">
        <DashboardHeader />
        <EngineDashboard />
      </div>
    </ROSProvider>
  );
}

export default App;
