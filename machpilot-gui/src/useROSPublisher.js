// src/useROSPublisher.js
import { useMemo } from 'react';
import ros from './rosConnection';
import {Topic, Message} from 'roslib';

/**
 * useROSPublisher is a custom hook that creates a ROSLIB Topic and returns a function to publish messages.
 * @param {string} topicName - The name of the ROS topic (e.g., "/h20pro/throttle_command")
 * @param {string} messageType - The type of the message (e.g., "std_msgs/msg/Float32")
 * @returns {Function} publishMessage - A function that accepts a message data object and publishes it.
 */
const useROSPublisher = (topicName, messageType) => {
  // useMemo ensures that the topic instance is only created once unless topicName or messageType changes.
  const topic = useMemo(() => {
    return new Topic({
      ros: ros,
      name: topicName,
      messageType: messageType,
    });
  }, [topicName, messageType]);

  // publishMessage accepts an object representing the message data.
  const publishMessage = (message) => {
    topic.publish(message);
    console.log(`Published to ${topicName}:`, message);
  };

  return publishMessage;
};

export default useROSPublisher;
