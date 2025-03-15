import React from 'react';
import GenericActionPanel from './GenericActionPanel';

/**
 * StarterTestPanel calls the StarterTest action.
 * Since StarterTest.action has an empty goal, no input fields are required.
 */
const StarterTestPanel = () => {
  const fields = []; // No input fields needed.
  const formatGoal = () => ({}); // Return an empty goal.

  return (
    <GenericActionPanel
      title="Starter Test Action"
      serverName="/h20pro/starter_test"
      actionType="interfaces/StarterTestAction"  // Adjust to match your generated type.
      fields={fields}
      formatGoal={formatGoal}
    />
  );
};

export default StarterTestPanel;
