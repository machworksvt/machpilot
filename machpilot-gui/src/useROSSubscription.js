// src/useROSSubscription.js
import { useEffect, useState } from 'react';
import ros from './rosConnection';
import {Topic} from 'roslib';

// Custom hook to subscribe to a ROS topic.
// - topicName: the name of the topic (e.g., "/h20pro/engine_data")
// - messageType: the full type of the message (e.g., "interfaces/msg/EngineData")
const useROSSubscription = (topicName, messageType) => {
  const [message, setMessage] = useState(null);

  useEffect(() => {
    // Create a new topic instance using ROSLIB.
    const topic = new Topic({
      ros: ros,
      name: topicName,
      messageType: messageType
    });

    // Subscribe to the topic. Every time a message arrives, update state.
    topic.subscribe((msg) => {
      setMessage(msg);
    });

    // Cleanup function: unsubscribe when the component unmounts.
    return () => {
      topic.unsubscribe();
    };
  }, [topicName, messageType]);

  return message;
};

export default useROSSubscription;
