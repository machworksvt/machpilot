// src/components/ErrorsPanel.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';

/**
 * ErrorsPanel subscribes to the '/h20pro/errors' topic and displays any
 * error information. It expects the custom message type 'interfaces/msg/Errors'.
 */
const ErrorsPanel = () => {
  const errors = useROSSubscription('/h20pro/errors', 'interfaces/msg/Errors');

  return (
    <div className="panel">
      <h3>Errors</h3>
      {errors ? (
        <div>
          <p><strong>Error Mask:</strong> {errors.error_mask.toString(16)}</p>
          {errors.error_messages && errors.error_messages.length > 0 ? (
            <ul>
              {errors.error_messages.map((err, index) => (
                <li key={index}>{err}</li>
              ))}
            </ul>
          ) : (
            <p>No errors reported.</p>
          )}
          <p><strong>Reset Required:</strong> {errors.reset_required ? 'Yes' : 'No'}</p>
        </div>
      ) : (
        <p>Waiting for error data...</p>
      )}
    </div>
  );
};

export default ErrorsPanel;
