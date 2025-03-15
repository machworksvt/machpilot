// src/components/ThrottleControl.js
import React, { useState } from 'react';
import useROSPublisher from '../useROSPublisher';

/**
 * ThrottleControl provides a slider for setting the throttle value.
 * When the slider moves, it updates the local state and publishes the new throttle value
 * to the ROS topic (e.g., "/h20pro/throttle_command") with message type "std_msgs/msg/Float32".
 */
const ThrottleControl = () => {
  // Local state for the slider value (0 to 100% throttle)
  const [throttleValue, setThrottleValue] = useState(0);

  // Create a publisher for the throttle command using our custom hook.
  // This assumes the ROS topic for throttle commands is "/h20pro/throttle_command".
  const publishThrottle = useROSPublisher('/h20pro/throttle_command', 'std_msgs/msg/Float32');

  // handleChange is triggered when the slider value changes.
  const handleChange = (e) => {
    // Convert the input value (string) to a number.
    const newValue = parseFloat(e.target.value);
    setThrottleValue(newValue);
    // Publish the new throttle value. The message structure for std_msgs/msg/Float32
    // expects a field "data".
    publishThrottle({ data: newValue });
  };

  return (
    <div style={{ margin: '20px' }}>
      <h2>Throttle Control</h2>
      {/* Slider for throttle value between 0 and 100 */}
      <input
        type="range"
        min="0"
        max="100"
        value={throttleValue}
        onChange={handleChange}
        style={{ width: '100%' }}
      />
      <p>Throttle: {throttleValue}%</p>
    </div>
  );
};

export default ThrottleControl;
