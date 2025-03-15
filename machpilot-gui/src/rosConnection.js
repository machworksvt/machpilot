// src/rosConnection.js

const ros = new window.ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', () =>{
    console.log('Connected to ROS');
});

ros.on('error', (error) =>{
    console.error('Error connecting to ROS: ', error);
});

ros.on('close', () =>{
    console.log('Disconnected from ROS');
});

export default ros;