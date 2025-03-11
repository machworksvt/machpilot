import React from "react";

const Pane = ({ id, text, color, onSplit }) => {
  const handleKeyDown = (event) => {
    const key = event.key.toLowerCase();
    if (key === "v") {
      // Pressing 'v' triggers a horizontal split
      onSplit(id, "horizontal");
    } else if (key === "h") {
      // Pressing 'h' triggers a vertical split
      onSplit(id, "vertical");
    }
  };

  return (
    <div
      tabIndex={0} // Makes the div focusable to receive keyboard events.
      style={{
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        background: color,
        width: "100%",
        height: "100%",
        cursor: "pointer"
      }}
      onKeyDown={handleKeyDown}
    >
      {text}
    </div>
  );
};

export default Pane;
