import React, { useRef, useEffect } from "react";
import * as d3 from "d3";

const LiquidGauge = ({ value = 50, config = {} }) => {
  const containerRef = useRef(null);

  useEffect(() => {
    // Clear any existing content in the container.
    d3.select(containerRef.current).selectAll("*").remove();

    // Dimensions and configuration
    const width = config.width || 200;
    const height = config.height || 200;
    const radius = Math.min(width, height) / 2;
    const waveHeight = config.waveHeight || 10; // amplitude of the wave

    // Create the main SVG element.
    const svg = d3
      .select(containerRef.current)
      .append("svg")
      .attr("width", width)
      .attr("height", height);

    // Center the gauge in a group element.
    const group = svg
      .append("g")
      .attr("transform", `translate(${width / 2}, ${height / 2})`);

    // Draw the background circle.
    group
      .append("circle")
      .attr("r", radius)
      .attr("fill", config.circleColor || "#e0e0e0");

    // Generate wave data (a sine wave that will be used as a clip path).
    const wavePoints = 100;
    const data = [];
    for (let i = 0; i <= wavePoints; i++) {
      const x = (i / wavePoints) * width;
      const angle = (i / wavePoints) * 2 * Math.PI;
      // Calculate y based on the sine wave; the vertical offset positions the wave based on the value.
      const y = Math.sin(angle) * waveHeight + (height * (1 - value / 100));
      data.push({ x, y });
    }

    // Create an area generator for the wave shape.
    const area = d3
      .area()
      .x((d) => d.x)
      .y0(height) // bottom of the gauge
      .y1((d) => d.y)
      .curve(d3.curveBasis);

    // Create a unique clip path id.
    const clipId = "clipWave" + Math.random().toString(36).substr(2, 9);

    // Append a clip path to the SVG defs.
    svg
      .append("defs")
      .append("clipPath")
      .attr("id", clipId)
      .append("path")
      .datum(data)
      .attr("d", area)
      .attr("transform", `translate(${-width / 2}, ${-height / 2})`);

    // Draw the liquid fill circle using the clip path.
    group
      .append("circle")
      .attr("r", radius)
      .attr("fill", config.waveColor || "#178BCA")
      .attr("clip-path", `url(#${clipId})`);

    // Add a text label displaying the value.
    group
      .append("text")
      .attr("text-anchor", "middle")
      .attr("dy", ".35em")
      .style("font-size", config.textSize || "30px")
      .text(`${value}%`);
  }, [value, config]);

  return <div ref={containerRef} />;
};

export default LiquidGauge;

