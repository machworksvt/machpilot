// src/App.jsx
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function App() {
  const [message, setMessage] = useState('Waiting for data...');

  useEffect(() => {
    // Connect to rosbridge WebSocket server
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', () => console.log('Connected to ROS'));
    ros.on('error', (error) => console.error('Error connecting to ROS: ', error));
    ros.on('close', () => console.log('Connection to ROS closed'));

    // Subscribe to a topic
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/random_numbers',
      messageType: 'std_msgs/Float32'
    });

    topic.subscribe((msg) => {
      setMessage(msg.data.toString());
    });

    // Cleanup on unmount
    return () => {
      topic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div>
      <h1>ROS 2 Data in React</h1>
      <p>Random Number: {message}</p>
    </div>
  );
}

export default App;
