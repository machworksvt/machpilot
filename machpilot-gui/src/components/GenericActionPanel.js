import React, { useState, useEffect, useMemo } from 'react';
import ROSLIB from 'roslib';
import ros from '../rosConnection';

/**
 * GenericActionPanelROS2 is a generic component for calling any ROS2 action using the ROSLIB.ActionClient API.
 *
 * Props:
 *  - title: Title of the panel.
 *  - serverName: The name of the action server (e.g., "/h20pro/starter_test").
 *  - actionType: The ROS2 action type string (e.g., "interfaces/StarterTestAction").
 *    (This should match your generated action type.)
 *  - fields: An array of input field definitions for the goal.
 *            Each field is an object: { name, label, type (optional), defaultValue (optional) }.
 *  - formatGoal: (Optional) A function that receives the form values and returns the goal message.
 *
 * This implementation:
 *   - Creates an ActionClient with the proper properties (“name” and “actionType”).
 *   - Creates a Goal, registers 'feedback' and 'result' callbacks, and sends the goal.
 */
const GenericActionPanel = ({
  title,
  serverName,
  actionType,
  fields = [],
  formatGoal,
}) => {
  // Build initial form state from provided fields.
  const initialFormValues = fields.reduce((acc, field) => {
    acc[field.name] = field.defaultValue || '';
    return acc;
  }, {});
  const [formValues, setFormValues] = useState(initialFormValues);

  // State for tracking action status, feedback, result, and the active goal.
  const [actionState, setActionState] = useState('Idle'); // Idle, Executing, Feedback, Succeeded, Aborted, etc.
  const [feedback, setFeedback] = useState(null);
  const [result, setResult] = useState(null);
  const [goal, setGoal] = useState(null);

  // Create the ROS2 ActionClient once using the correct property names.
  const actionClient = useMemo(() => {
    return new ROSLIB.ActionClient({
      ros: ros,
      name: serverName,       // <-- Note: use "name" (not "serverName")
      actionType: actionType, // <-- Use "actionType" as provided
    });
  }, [serverName, actionType]);

  // Use the underlying ROS connection as a proxy for server availability.
  const [serverAvailable, setServerAvailable] = useState(false);
  useEffect(() => {
    if (ros.socket && ros.socket.readyState === WebSocket.OPEN) {
      setServerAvailable(true);
    } else {
      setServerAvailable(false);
    }
    // Optionally, you could add listeners or a polling interval.
  }, []);

  // Handler for input field changes.
  const handleInputChange = (e, fieldName) => {
    setFormValues({
      ...formValues,
      [fieldName]: e.target.value,
    });
  };

  // Function to send the goal.
  const sendGoal = () => {
    // Do not send if the server isn't available or an action is already active.
    if (!serverAvailable || actionState !== 'Idle') return;

    // Build the goal message.
    const goalMessage = formatGoal ? formatGoal(formValues) : formValues;

    // Update state and create a new goal.
    setActionState('Executing');
    setFeedback(null);
    setResult(null);
    const newGoal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: goalMessage,
    });
    setGoal(newGoal);

    // Register the feedback callback.
    newGoal.on('feedback', (feedbackMsg) => {
      console.log('Feedback:', feedbackMsg);
      setFeedback(feedbackMsg);
      setActionState('Feedback');
    });

    // Register the result callback.
    newGoal.on('result', (resultMsg) => {
      console.log('Result:', resultMsg);
      setResult(resultMsg);
      setActionState(resultMsg.result && resultMsg.result.success ? 'Succeeded' : 'Aborted');
      setGoal(null);
    });

    // Send the goal.
    newGoal.send();
  };

  // Function to cancel the active goal.
  const cancelGoal = () => {
    if (goal && goal.cancel) {
      goal.cancel();
      setActionState('Cancelled');
      setGoal(null);
    }
  };

  return (
    <div style={panelStyle}>
      <h2>{title}</h2>
      {/* Render input fields if provided */}
      {fields.map((field) => (
        <div key={field.name} style={{ marginBottom: '10px' }}>
          <label htmlFor={field.name}>{field.label}:</label>
          <input
            id={field.name}
            type={field.type || 'text'}
            value={formValues[field.name]}
            onChange={(e) => handleInputChange(e, field.name)}
            style={{ marginLeft: '10px' }}
          />
        </div>
      ))}
      <div style={{ marginBottom: '10px' }}>
        <strong>Action State:</strong> {actionState}
      </div>
      {feedback && (
        <div style={{ marginBottom: '10px' }}>
          <strong>Feedback:</strong> {JSON.stringify(feedback)}
        </div>
      )}
      {result && (
        <div style={{ marginBottom: '10px' }}>
          <strong>Result:</strong> {JSON.stringify(result)}
        </div>
      )}
      <button onClick={sendGoal} disabled={!serverAvailable || actionState !== 'Idle'}>
        {serverAvailable ? 'Send Goal' : 'Action Server Unavailable'}
      </button>
      {goal && goal.cancel && (
        <button onClick={cancelGoal} style={{ marginLeft: '10px' }}>
          Cancel Goal
        </button>
      )}
    </div>
  );
};

const panelStyle = {
  background: '#f4f4f4',
  border: '1px solid #ddd',
  padding: '15px',
  margin: '15px',
  borderRadius: '4px',
};

export default GenericActionPanel;
