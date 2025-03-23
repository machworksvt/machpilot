// src/components/GenericActionExecutor.js
import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import ros from '../rosConnection';

const GenericActionExecutor = ({
  friendlyName,
  serverName,
  actionType,
  fields = [],
  initialFeedback = {},
  initialResult = {},
}) => {
  // Build initial goal state from fields (store as strings for inputs)
  const initGoal = fields.reduce((acc, f) => {
    acc[f.name] = f.defaultValue !== undefined ? String(f.defaultValue) : '';
    return acc;
  }, {});
  const [goalMessage, setGoalMessage] = useState(initGoal);
  const [feedback, setFeedback] = useState(initialFeedback);
  const [result, setResult] = useState(initialResult);
  const [actionState, setActionState] = useState('Idle'); // Idle, Executing, Feedback, Succeeded, Aborted, Cancelled, Timeout, etc.
  const [actionObj, setActionObj] = useState(null);
  const [serverAvailable, setServerAvailable] = useState(false);

  useEffect(() => {
    // Log current state for debugging.
    console.log("ros.socket:", ros.socket);
    if (ros.socket && ros.socket.readyState === WebSocket.OPEN) {
      setServerAvailable(true);
      console.log("Server available");
    } else {
      setServerAvailable(false);
      console.log("Server NOT available");
    }
  }, []);

  const handleInputChange = (e, fieldName) => {
    const newVal = e.target.value;
    setGoalMessage(prev => ({ ...prev, [fieldName]: newVal }));
  };

  const sendGoal = () => {
    console.log("Attempting to send goal. Server available:", serverAvailable, "Action state:", actionState);
    if (!serverAvailable || actionState !== 'Idle') {
      console.log("Cannot send goal; conditions not met.");
      return;
    }
    setActionState('Executing');
    setFeedback(initialFeedback);
    setResult(initialResult);

    // Convert numeric fields from strings to numbers
    const convertedGoal = { ...goalMessage };
    fields.forEach(f => {
      if ((f.type || 'text') === 'number') {
        convertedGoal[f.name] = Number(goalMessage[f.name]);
      }
    });

    const actionClient = new ROSLIB.Action({
      ros,
      name: serverName, // ROS2-style property
      actionType,
      timeout: 5,
    });

    const newGoal = actionClient.sendGoal(
      convertedGoal,
      (res) => {
        console.log('Result:', res);
        setResult(res);
        setActionState(res.result && res.result.success ? 'Succeeded' : 'Aborted');
        setActionObj(null);
      },
      (fb) => {
        console.log('Feedback:', fb);
        setFeedback(fb);
        setActionState('Feedback');
      }
    );
    setActionObj(newGoal);
  };

  const cancelGoal = () => {
    if (actionObj && actionObj.cancel) {
      actionObj.cancel();
      setActionState('Cancelled');
      setActionObj(null);
      console.log("Goal cancelled.");
    }
  };

  const resetExecutor = () => {
    setActionState('Idle');
    setFeedback(initialFeedback);
    setResult(initialResult);
    setActionObj(null);
    console.log("Reset to Idle.");
  };

  const showReset = !actionObj && ['Succeeded', 'Aborted', 'Cancelled', 'Timeout'].includes(actionState);

  return (
    <div style={containerStyle}>
      <h2>{friendlyName}</h2>
      {fields.map((f) => (
        <div key={f.name} style={inputGroupStyle}>
          <label htmlFor={f.name} style={labelStyle}>{f.label}:</label>
          <input
            id={f.name}
            type={f.type || 'text'}
            value={goalMessage[f.name]}
            onChange={(e) => handleInputChange(e, f.name)}
            style={inputStyle}
          />
        </div>
      ))}
      <div style={buttonContainerStyle}>
        {/* Remove disabled condition for debugging */}
        <button onClick={sendGoal} style={buttonStyle}>
          {serverAvailable ? 'Send Goal' : 'Server Unavailable'}
        </button>
        {actionObj && actionObj.cancel && (
          <button onClick={cancelGoal} style={{ ...buttonStyle, marginLeft: '5px' }}>
            Cancel
          </button>
        )}
        {showReset && (
          <button onClick={resetExecutor} style={{ ...buttonStyle, marginLeft: '5px' }}>
            Reset
          </button>
        )}
      </div>
      <div style={{ marginTop: '5px' }}>
        <strong>Action State:</strong> {actionState}
      </div>
      <div style={sectionStyle}>
        <h3>Feedback</h3>
        <pre style={preStyle}>{JSON.stringify(feedback, null, 2)}</pre>
      </div>
      <div style={sectionStyle}>
        <h3>Result</h3>
        <pre style={preStyle}>{JSON.stringify(result, null, 2)}</pre>
      </div>
      <div style={sectionStyle}>
        <h3>Goal Message (Debug)</h3>
        <pre style={preStyle}>{JSON.stringify(goalMessage, null, 2)}</pre>
      </div>
    </div>
  );
};

const containerStyle = {
  padding: '10px',
  border: '1px solid #ddd',
  borderRadius: '4px',
  background: '#f9f9f9',
  textAlign: 'center',
  maxWidth: '600px',
  margin: 'auto',
};

const inputGroupStyle = {
  margin: '5px 0',
  display: 'flex',
  justifyContent: 'center',
  alignItems: 'center',
};

const labelStyle = {
  marginRight: '5px',
  fontWeight: 'bold',
};

const inputStyle = {
  padding: '5px',
  fontSize: '16px',
  width: '100px',
};

const sectionStyle = {
  margin: '5px 0',
};

const preStyle = {
  textAlign: 'left',
  background: '#eee',
  padding: '5px',
  borderRadius: '4px',
};

const buttonContainerStyle = {
  marginTop: '10px',
};

const buttonStyle = {
  padding: '8px 15px',
  fontSize: '14px',
  borderRadius: '4px',
  border: 'none',
  backgroundColor: '#007BFF',
  color: '#fff',
  cursor: 'pointer',
};

export default GenericActionExecutor;
