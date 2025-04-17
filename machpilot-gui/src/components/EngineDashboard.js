// src/components/EngineDashboard.js
import React from 'react';
import useROSSubscription from '../useROSSubscription';
import useROSPublisher from '../useROSPublisher';
import GaugeComponent from 'react-gauge-component';
import EngineDataDumpPanel from './EngineDataDumpPanel';
import GenericActionExecutor from './GenericActionExecutor';
import PumpTestPanel from './PumpTestPanel';
import ros from '../rosConnection';
import * as ROSLIB from 'roslib';
import { useState } from 'react';

const EngineDashboard = () => {
  const engineData = useROSSubscription('/h20pro/engine_data', 'interfaces/msg/EngineData');
  const pumpRpm = useROSSubscription('/h20pro/pump_rpm', 'interfaces/msg/engine_data2');
  const throttle = useROSSubscription('/h20pro/throttle_command', 'std_msgs/msg/Float32');
  const errors = useROSSubscription('/h20pro/errors', 'interfaces/msg/Errors');
  const fuelAmbient = useROSSubscription('/h20pro/fuel_ambient', 'interfaces/msg/FuelAmbient');
  const glowPlugs = useROSSubscription('/h20pro/glow_plugs', 'interfaces/msg/GlowPlugs');
  const lastRunInfo = useROSSubscription('/h20pro/last_run_info', 'interfaces/msg/LastRunInfo');
  const ngReg = useROSSubscription('/h20pro/ng_reg', 'interfaces/msg/NgReg');
  const statistics = useROSSubscription('/h20pro/statistics', 'interfaces/msg/Statistics');
  const systemInfo = useROSSubscription('/h20pro/system_info', 'interfaces/msg/SystemInfo');
  const systemInfo2 = useROSSubscription('/h20pro/system_info2', 'interfaces/msg/PumpRpm');
  const voltageCurrent = useROSSubscription('/h20pro/voltage_current', 'interfaces/msg/VoltageCurrent');

  var throttlePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/h20pro/throttle_command',
    messageType: 'std_msgs/msg/Float32',
  });
  /*
    engine_data_pub_ = this->create_publisher<interfaces::msg::EngineData>("/h20pro/engine_data", 10);
    engine2_data_pub_ = this->create_publisher<interfaces::msg::PumpRpm>("/h20pro/engine_data2", 10);
    errors_current_pub_ = this->create_publisher<interfaces::msg::Errors>("/h20pro/errors", 10);
    fuel_ambient_pub_ = this->create_publisher<interfaces::msg::FuelAmbient>("/h20pro/fuel_ambient", 10);
    glow_plugs_pub_ = this->create_publisher<interfaces::msg::GlowPlugs>("/h20pro/glow_plugs", 10);
    last_run_info_pub_ = this->create_publisher<interfaces::msg::LastRunInfo>("/h20pro/last_run_info", 10);
    ng_reg_pub_ = this->create_publisher<interfaces::msg::NgReg>("/h20pro/ng_reg", 10);
    statistics_pub_ = this->create_publisher<interfaces::msg::Statistics>("/h20pro/statistics", 10);
    system_info_pub_ = this->create_publisher<interfaces::msg::SystemInfo>("/h20pro/system_info", 10);
    system_info2_pub_ = this->create_publisher<interfaces::msg::SystemInfo2>("/h20pro/system_info2", 10);
    voltage_current_pub_ = this->create_publisher<interfaces::msg::VoltageCurrent>("/h20pro/voltage_current", 10);
  */

  // Decode engineData
  const rpm = engineData ? engineData.real_rpm : 0;
  const egt = engineData ? engineData.egt : 0;
  const stateName = engineData ? engineData.state_name : 'UNKNOWN';

  // Decode pumpRpm
  const pump_rpm = pumpRpm ? pumpRpm.pump_rpm : 0;

  // Decode voltateCurrent
  const batteryVoltage = voltageCurrent ? voltageCurrent.battery_voltage : 0;
  const batteryCurrent = voltageCurrent ? voltageCurrent.battery_current : 0;

  // Decode errors
  const errorsList = errors ? errors.errors : [];
  const resetRequired = errors ? errors.reset_required : false;

  // Decode fuelAmbient
  const fuelFlow = fuelAmbient ? fuelAmbient.fuel_flow : 0;
  const fuelConsumed = fuelAmbient ? fuelAmbient.fuel_consumed : 0;
  const engineBoxPressure = fuelAmbient ? fuelAmbient.engine_box_pressure : 0;
  const ambientTemperature = fuelAmbient ? fuelAmbient.ambient_temperature : 0;

  // Decode glowPlugs
  const GP1_V = glowPlugs ? glowPlugs.glow_plug_v[0] : 0;
  const GP2_V = glowPlugs ? glowPlugs.glow_plug_v[1] : 0;
  const GP1_I = glowPlugs ? glowPlugs.glow_plug_i[0] : 0;
  const GP2_I = glowPlugs ? glowPlugs.glow_plug_i[1] : 0;  


  // New subscriptions / values for the right column gauges:
  const glowPlug1Voltage = engineData ? engineData.glowplug1_voltage : 0;
  const glowPlug1Current = engineData ? engineData.glowplug1_current : 0;
  const glowPlug2Voltage = engineData ? engineData.glowplug2_voltage : 0;
  const glowPlug2Current = engineData ? engineData.glowplug2_current : 0;


  const throttle_command = throttle ? throttle.data : 0;

  var primeAction = new ROSLIB.Action({
    ros: ros,
    name: '/h20pro/prime',
    actionType: 'interfaces/action/Prime',
    timeout: 5,
  });

  var startAction = new ROSLIB.Action({
    ros: ros,
    name: '/h20pro/start',
    actionType: 'interfaces/action/Start',
    timeout: 5,
  });

  var starterTestAction = new ROSLIB.Action({
    ros: ros,
    name: '/h20pro/starter_test',
    actionType: 'interfaces/action/StarterTest',
    timeout: 5,
  });

  // All data from ROS2 has been decoded at this point
  // The rest is just

  const formatRPMtick = (value) => {
    if (value >= 1000) {
      value = value / 1000;
      if (Number.isInteger(value)) {
        return value.toFixed(0) + 'k';
      } else {
        return value.toFixed(1) + 'k';
      }
    } else {
      return value.toFixed(0);
    }
  };

  const formatRPM = (value) => {
    return formatRPMtick(value);
  };

  const formatPressure = (value) => {
    // do the same conversion to thousands as the rpm
      value = value / 1000;
      if (Number.isInteger(value)) {
        return value.toFixed(0) + ' bar';
      } else {
        return value.toFixed(1) + ' bar';
      }
  };

  const [isRunningStartTest, setIsRunningStartTest] = useState(false);
  const [startTestButtonText, setStartTestButtonText] = useState('Start Test');

  const startTestButton = () => {
    console.log("Start Test Button Clicked");

    var goal = {};

    setIsRunningStartTest(true);
    setStartTestButtonText('Running Test...');
    var goal_id = starterTestAction.sendGoal(goal,
    function(result) {
      console.log(result);
      setIsRunningStartTest(false);
      setStartTestButtonText('Start Test');
    },
    function(feedback) {
      console.log(feedback);
    },
    );
    console.log(goal_id);
  };

  const [isPriming, setIsPriming] = useState(false);
  const [primeButtonText, setPrimeButtonText] = useState('Prime');

  const primeButton = () => {
    console.log("Prime Button Clicked");
    var goal = {
      pump_power_percent: 10
    };

    setIsPriming(true);
    setPrimeButtonText('Priming...');
    var goal_id = primeAction.sendGoal(goal, 
    function(result) {
      console.log(result);
      setIsPriming(false);
      setPrimeButtonText('Prime');
    },
    function(feedback) {
      console.log(feedback);
    },
    );
  };

  const [isStarting, setIsStarting] = useState(false);
  const [startButtonText, setStartButtonText] = useState('Start');

  const startButton = () => {
    console.log("Start Button Clicked");
    var goal = {};

    setIsStarting(true);
    setStartButtonText('Starting...');
    var goal_id = startAction.sendGoal(goal, 
    function(result) {
      console.log(result);
      setIsStarting(false);
      setStartButtonText('Start');
    },
    function(feedback) {
      console.log(feedback);
    },
    );
  };

  const [isKilling, setIsKilling] = useState(false);
  const [killButtonText, setKillButtonText] = useState('Kill Engine');

  const killService = new ROSLIB.Service({
    ros: ros,
    name: '/h20pro/kill',
    serviceType: 'std_srvs/srv/Trigger',
  });

  const killEngine = () => {
    console.log("Kill Engine Button Clicked");
    var request = {};

    setIsKilling(true);
    setKillButtonText('Killing...');
    killService.callService(request, function(result) {
      console.log(result);
      setIsKilling(false);
      setKillButtonText('Kill Engine');
    });
  };
  //final button for killing the engine

  const [throttleValue, setThrottleValue] = useState(0);

  const handleThrottleChange = (e) => {
    const newValue = parseFloat(e.target.value);
    setThrottleValue(newValue);
    throttlePublisher.publish({ data: newValue });
  };

  // Dynamic style for the state header:
  const stateHeaderStyle = {
    width: '100%',
    padding: '2px',
    textAlign: 'center',
    fontSize: '50px',
    fontFamily: 'MW Font',
    fontWeight: 'bold',
    color: stateName === 'RUNNING' ? 'green' : '#fff',
  };

  const primeButtonStyle = {
    backgroundColor: '#2E8B57', // SeaGreen
    color: isPriming ? '#fff' : '#000',
    border: 'none',
    borderRadius: '5px',
    padding: '10px 20px',
    fontSize: '20px',
    fontFamily: 'MW Font',
    fontWeight: 'bold',
    cursor: 'crosshair',
    position: 'relative',
    zIndex: 1000,
  };

  const startButtonStyle = {
    backgroundColor: '#2E8B57', // SeaGreen
    color: isStarting ? '#fff' : '#000',
    border: 'none',
    borderRadius: '5px',
    padding: '10px 20px',
    fontSize: '20px',
    fontFamily: 'MW Font',
    fontWeight: 'bold',
    cursor: 'crosshair',
    position: 'relative',
    zIndex: 1000,
  };

  const killButtonStyle = {
    backgroundColor: '#8B0000', // DarkRed
    color: isKilling ? '#fff' : '#000',
    border: 'none',
    borderRadius: '5px',
    padding: '10px 20px',
    fontSize: '20px',
    fontFamily: 'MW Font',
    fontWeight: 'bold',
    cursor: 'crosshair',
    position: 'relative',
    zIndex: 1000
  };

  const startTestButtonStyle = {
    backgroundColor: '#8B0000', // DarkRed
    color: isRunningStartTest ? '#fff' : '#000',
    border: 'none',
    borderRadius: '5px',
    padding: '10px 20px',
    fontSize: '20px',
    fontFamily: 'MW Font',
    fontWeight: 'bold',
    cursor: 'crosshair',
    position: 'relative',
    zIndex: 1000
  };

  return (
    <div className="dashboard engine-dashboard" style={dashboardStyle}>
      <div style={gridStyle}>
        <div style={columnStyle}>
          {/* Left column elements */}
          <button onClick={primeButton} disabled={isPriming} style={primeButtonStyle}>{primeButtonText}</button>
          <button onClick={startButton} disabled={isStarting} style={startButtonStyle}>{startButtonText}</button>
          <button onClick={killEngine} disabled={isKilling} style={killButtonStyle}>{killButtonText}</button>
          <button onClick={startTestButton} disabled={isRunningStartTest} style={startTestButtonStyle}>{startTestButtonText}</button>
          <input
          type="range"
          min="0"
          max="100"
          step="1"
          value={throttleValue}
          onChange={handleThrottleChange}
          style={{ width: '100%', zIndex: 1000 }}
          />
        </div>
        <div style={centerColumnStyle}>
          {/* Center column elements */}
          <div style={stateHeaderStyle}>
            {'State: ' + stateName}
          </div>
          <div style={centerGaugeGridStyle}>
            {/* Top Left: RPM Gauge */}
            <div style={gaugeContainerStyle}>
              <div style={centerGaugeStyle}>
                <GaugeComponent
                  arc={{
                    startAngle: 135,
                    endAngle: 45,
                    subArcs: [
                      { limit: 6700, color: '#5BE12C', showTick: true },
                      { limit: 37000, color: '#F5CD19', showTick: true },
                      { limit: 125000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.3,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: valueLabelStyle,
                      formatTextValue: formatRPM,
                    },
                    tickLabels: {
                      type: "outer",
                      defaultTickValueConfig: {
                        formatTextValue: formatRPMtick,
                        style: tickLabelStyle,
                      },
                      ticks: [
                        { value: 45000 },
                        { value: 55000 },
                        { value: 65000 },
                        { value: 75000 },
                        { value: 85000 },
                        { value: 95000 },
                        { value: 105000 },
                        { value: 115000 },
                      ],
                    },
                  }}
                  value={rpm}
                  maxValue={125000}
                  pointer={{ type: "arrow", elastic: true }}
                />
              </div>
              <div style={gaugeTitleStyle}>RPM</div>
            </div>
            {/* Top Right: EGT Gauge */}
            <div style={gaugeContainerStyle}>
              <div style={centerGaugeStyle}>
                <GaugeComponent
                  arc={{
                    subArcs: [
                      { limit: 1000, color: '#5BE12C', showTick: true },
                      { limit: 2000, color: '#F5CD19', showTick: true },
                      { limit: 5000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.3,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: valueLabelStyle,
                      formatTextValue: formatPressure,
                    },
                    tickLabels: {
                      type: 'outer',
                      ticks: [
                        { value: 3000 },
                        { value: 4000 },
                      ],
                      defaultTickValueConfig: {
                        style: tickLabelStyle,
                        formatTextValue: formatPressure,
                      },
                    },
                  }}
                  value={engineBoxPressure}
                  maxValue={5000}
                />
              </div>
              <div style={gaugeTitleStyle}>Combustor Pressure</div>
            </div>
            {/* Bottom Left: Engine Box Pressure Gauge */}
            <div style={gaugeContainerStyle}>
              <div style={centerGaugeStyle}>
                <GaugeComponent
                  arc={{
                    subArcs: [
                      { limit: 1000, color: '#5BE12C', showTick: true },
                      { limit: 2000, color: '#F5CD19', showTick: true },
                      { limit: 5000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.3,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: valueLabelStyle,
                      formatTextValue: formatPressure,
                    },
                    tickLabels: {
                      type: 'outer',
                      ticks: [
                        { value: 3000 },
                        { value: 4000 },
                      ],
                      defaultTickValueConfig: {
                        style: tickLabelStyle,
                        formatTextValue: formatPressure,
                      },
                    },
                  }}
                  value={engineBoxPressure}
                  maxValue={5000}
                />
              </div>
              <div style={gaugeTitleStyle}>Combustor Pressure</div>
            </div>
            {/* Bottom Right: Engine Box Temperature Gauge */}
            <div style={gaugeContainerStyle}>
              <div style={centerGaugeStyle}>
                <GaugeComponent
                  arc={{
                    subArcs: [
                      { limit: 22, color: '#5BE12C', showTick: true },
                      { limit: 350, color: '#F5CD19', showTick: true },
                      { limit: 1000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.3,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: valueLabelStyle,
                      formatTextValue: (value) => value.toFixed(0) + ' °C',
                    },
                    tickLabels: {
                      type: 'outer',
                      ticks: [
                        { value: 200 },
                        { value: 450 },
                        { value: 550 },
                        { value: 650 },
                        { value: 750 },
                        { value: 850 },
                        { value: 950 },
                      ],
                      defaultTickValueConfig: {
                        style: tickLabelStyle,
                        formatTextValue: (value) => value.toFixed(0) + ' °C',
                      },
                    },
                  }}
                  value={ambientTemperature}
                  maxValue={1000}
                />
              </div>
              <div style={gaugeTitleStyle}>Combustor Temp</div>
            </div>
          </div>
        </div>
        <div style={columnStyle}>
          {/*
          {/* Right column elements */}
          {/* Two larger battery gauges (half size of center gauges) */}
          <div style={{ flex: '1' }}>
          {/* Right column: Dump all data */}
          <EngineDataDumpPanel
            engineData={engineData}
            pumpRpm={pumpRpm}
            throttle={throttle}
            errors={errors}
            fuelAmbient={fuelAmbient}
            glowPlugs={glowPlugs}
            lastRunInfo={lastRunInfo}
            ngReg={ngReg}
            statistics={statistics}
            systemInfo={systemInfo}
            systemInfo2={systemInfo2}
            voltageCurrent={voltageCurrent}
          />
        </div>
        </div>
      </div>
    </div>
  );
};

// Common styles for left/center column gauges
const tickLabelStyle = {
  fontSize: '20px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const valueLabelStyle = {
  fontSize: '40px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const dashboardStyle = {
  padding: '20px',
  fontFamily: 'Arial, sans-serif',
};

const columnStyle = {
  display: 'flex',
  flexDirection: 'column',
  gap: '15px',
};

const gaugeTitleStyle = {
  fontSize: '40px',
  marginTop: '20px',
  textAlign: 'center',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
  borderTop: '1px solid #fff',
};

const centerGaugeStyle = {
  width: '100%',
  height: '250px',
};

// Grid layout styles for the dashboard
const gridStyle = {
  display: 'grid',
  gridTemplateColumns: '25% 50% 25%',
  gridGap: '15px',
  boxSizing: 'border-box',
};

const centerColumnStyle = {
  display: 'flex',
  flexDirection: 'column',
  gap: '5px',
  boxSizing: 'border-box',
  borderRight: '1px solid #fff',
  borderLeft: '1px solid #fff',
};

const centerGaugeGridStyle = {
  display: 'grid',
  gridTemplateColumns: '1fr 1fr',
  gap: '2px',
  maxWidth: '100%',
  boxSizing: 'border-box',
};

const gaugeContainerStyle = {
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center',
};

// Right column grid for the four smaller glow plug gauges
const rightBottomGaugeGridStyle = {
  display: 'grid',
  gridTemplateColumns: '1fr 1fr',
  gap: '5px',
  width: '100%',
  boxSizing: 'border-box',
};

/* --- Right Column Specific Styles --- */
// Big gauges (half size of center gauges – approx 125px high)
const rightLargeGaugeStyle = {
  width: '100%',
  height: '125px',
};

const rightLargeValueLabelStyle = {
  fontSize: '25px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const rightLargeTickLabelStyle = {
  fontSize: '10px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const rightLargeGaugeTitleStyle = {
  fontSize: '20px',
  marginTop: '10px',
  textAlign: 'center',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
  borderTop: '1px solid #fff',
};

// Small gauges (quarter size of center gauges – approx 62px high)
const rightSmallGaugeStyle = {
  width: '100%',
  height: '62px',
};

const rightSmallValueLabelStyle = {
  fontSize: '12px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const rightSmallTickLabelStyle = {
  fontSize: '6px',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
};

const rightSmallGaugeTitleStyle = {
  fontSize: '15px',
  marginTop: '10px',
  textAlign: 'center',
  color: '#fff',
  fontFamily: 'MW Font',
  fontWeight: 'bold',
  borderTop: '1px solid #fff',
};

export default EngineDashboard;