import { useState } from "react";
import { Allotment } from "allotment";
import "allotment/dist/style.css";
import Pane from "./components/Pane";
import MachaMuchi from "./components/MachaMuchi";
import JetEngineRPM from "./components/JetEngineRPM";
import "./styles.css";


const componentsMapping = {
  default: Pane,
  gauge: MachaMuchi,
  jet_rpm: JetEngineRPM,
};


const splitPaneNode = (node, targetId, direction) => {
  if (node.isLeaf && node.id === targetId) {
    return {
      id: node.id, // generate a new id for the container.
      isLeaf: false,
      splitDirection: direction, // "vertical" or "horizontal"
      children: [
        {
          id: Date.now(),
          text: `${node.text} - A`,
          color: "lightgreen",
          isLeaf: true,
          componentType: node.componentType || "default",
          gaugeValue: node.gaugeValue || 0,
        },
        {
          id: Date.now() + 1,
          text: `${node.text} - B`,
          color: "lightcoral",
          isLeaf: true,
          componentType: node.componentType || "default",
          gaugeValue: node.gaugeValue || 0,
        },
      ],
    };
  }
  if (!node.isLeaf && node.children) {
    return {
      ...node,
      children: node.children.map((child) =>
        splitPaneNode(child, targetId, direction)
      ),
    };
  }
  return node;
};


const updateComponentType = (node, targetId, componentType) => {
  if (node.isLeaf && node.id === targetId) {
    return { ...node, componentType };
  }
  if (!node.isLeaf && node.children) {
    return {
      ...node,
      children: node.children.map((child) =>
        updateComponentType(child, targetId, componentType)
      ),
    };
  }
  return node;
};


const updateGaugeValue = (node, targetId, newValue) => {
  if (node.isLeaf && node.id === targetId) {
    return { ...node, gaugeValue: newValue };
  }
  if (!node.isLeaf && node.children) {
    return {
      ...node,
      children: node.children.map((child) =>
        updateGaugeValue(child, targetId, newValue)
      ),
    };
  }
  return node;
};


const PaneComponent = ({ node, onSplit, onChangeComponent, onUpdateGauge }) => {
  if (node.isLeaf) {
    // Determine which component to render based on node.componentType.
    const ComponentToRender = componentsMapping[node.componentType] || Pane;
    return (
      <div style={{ height: "100%", width: "100%", position: "relative" }}>
        <ComponentToRender
          id={node.id}
          text={node.text}
          color={node.color}
          onSplit={onSplit}
          
          {...(node.componentType === "gauge" || node.componentType === "jet_rpm" && {
            value: node.gaugeValue || 300,
          })}
        />
        {/* Dropdown to select the component type */}
        <select
          onChange={(e) => onChangeComponent(node.id, e.target.value)}
          value={node.componentType || "default"}
          style={{ position: "absolute", bottom: 5, right: 5 }}
        >
          <option value="default">Default Pane</option>
          <option value="gauge">Gauge Component</option>
          <option value="jet_rpm">Jet Engine RPM</option> 
        </select>
        {/* For gauge nodes, add an input to update the gauge value */}
        {(node.componentType === "gauge" || node.componentType === "jet_rpm") && (
  <input
    type="number"
    value={node.gaugeValue || 0}
    onChange={(e) => onUpdateGauge(node.id, parseInt(e.target.value, 10))}
    style={{
      position: "absolute",
      bottom: 35,
      right: 5,
      width: 50,
    }}
  />
)}

      </div>
    );
  } else {
    return (
      <Allotment vertical={node.splitDirection === "vertical"}>
        {node.children.map((child) => (
          <Allotment.Pane key={child.id}>
            <PaneComponent
              node={child}
              onSplit={onSplit}
              onChangeComponent={onChangeComponent}
              onUpdateGauge={onUpdateGauge}
            />
          </Allotment.Pane>
        ))}
      </Allotment>
    );
  }
};

const App = () => {
  
  const [rootPane, setRootPane] = useState({
    id: 1,
    text: "Pane 1",
    color: "lightblue",
    isLeaf: true,
    componentType: "default",
    gaugeValue: 0,
  });

  // Function to split a pane.
  const splitPane = (id, direction) => {
    setRootPane((prev) => splitPaneNode(prev, id, direction));
  };

  // Function to change a pane's component type.
  const changePaneComponent = (id, newComponentType) => {
    setRootPane((prev) => updateComponentType(prev, id, newComponentType));
  };

  // Function to update the gauge value for a pane.
  const changeGaugeValue = (id, newValue) => {
    setRootPane((prev) => updateGaugeValue(prev, id, newValue));
  };

  return (
    <div style={{ height: "100vh", width: "100vw" }}>
      <PaneComponent
        node={rootPane}
        onSplit={splitPane}
        onChangeComponent={changePaneComponent}
        onUpdateGauge={changeGaugeValue}
      />
    </div>
  );
};

export default App;
