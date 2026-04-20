import React from 'react';
import { ROSProvider } from './ROSContext';
import GPSMap from './components/GPSMap';

function App() {
  return (
    <ROSProvider>
      <div className="App">
        <GPSMap />
      </div>
    </ROSProvider>
  );
}

export default App;