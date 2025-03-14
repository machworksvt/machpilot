import React from "react";
import { GaugeComponent } from "react-gauge-component";

const GaugePreview = (props) => {
  const {
    gaugeType,
    gaugeId,
    gaugeClassName,
    computedValue,
    computedMinValue,
    computedMaxValue,
    cornerRadius,
    arcPadding,
    arcWidth,
    nbSubArcs,
    colorArray,
    emptyColor,
    gradient,
    subArcs,
    pointerType,
    pointerHide,
    pointerColor,
    pointerBaseColor,
    pointerLength,
    pointerAnimate,
    pointerElastic,
    pointerAnimationDuration,
    pointerAnimationDelay,
    pointerWidth,
    pointerStrokeWidth,
    valueLabelMatchColor,
    valueLabelFormat,
    valueLabelFontSize,
    valueLabelFill,
    valueLabelTextShadow,
    valueLabelMaxDecimalDigits,
    valueLabelHide,
    tickLabelsType,
    tickLabelsHideMinMax,
    tickLabelsTicks,
    tickDefaultFormat,
    tickDefaultFontSize,
    tickDefaultFill,
    tickDefaultTextShadow,
    tickDefaultMaxDecimalDigits,
    tickDefaultHide,
    tickLineWidth,
    tickLineLength,
    tickLineColor,
    tickLineDistance,
    tickLineHide,
    parseCommaSeparatedListToObjects,
    parseFunction,
  } = props;

  return (
    <div
      style={{
        border: "1px solid #ccc",
        width: "100%",
        minHeight: "800px",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        overflow: "visible"
      }}
    >
      <GaugeComponent
        type={gaugeType}
        id={gaugeId}
        className={gaugeClassName}
        style={{ width: "100%", height: "100%" }}
        value={computedValue}
        minValue={computedMinValue}
        maxValue={computedMaxValue}
        arc={{
          cornerRadius,
          padding: arcPadding,
          width: arcWidth,
          nbSubArcs: nbSubArcs ? Number(nbSubArcs) : undefined,
          colorArray: colorArray.split(","),
          emptyColor,
          gradient,
          subArcs: parseCommaSeparatedListToObjects(subArcs, "limit")
        }}
        pointer={{
          type: pointerType,
          hide: pointerHide,
          color: pointerColor,
          baseColor: pointerBaseColor,
          length: pointerLength,
          animate: pointerAnimate,
          elastic: pointerElastic,
          animationDuration: pointerAnimationDuration,
          animationDelay: pointerAnimationDelay,
          width: pointerWidth,
          strokeWidth: pointerStrokeWidth
        }}
        labels={{
          valueLabel: {
            matchColorWithArc: valueLabelMatchColor,
            formatTextValue: parseFunction(valueLabelFormat),
            style: {
              fontSize: valueLabelFontSize,
              fill: valueLabelFill,
              textShadow: valueLabelTextShadow
            },
            maxDecimalDigits: valueLabelMaxDecimalDigits,
            hide: valueLabelHide
          },
          tickLabels: {
            type: tickLabelsType,
            hideMinMax: tickLabelsHideMinMax,
            ticks: parseCommaSeparatedListToObjects(tickLabelsTicks, "value"),
            defaultTickValueConfig: {
              formatTextValue: parseFunction(tickDefaultFormat),
              style: {
                fontSize: tickDefaultFontSize,
                fill: tickDefaultFill,
                textShadow: tickDefaultTextShadow
              },
              maxDecimalDigits: tickDefaultMaxDecimalDigits,
              hide: tickDefaultHide
            },
            defaultTickLineConfig: {
              width: tickLineWidth,
              length: tickLineLength,
              color: tickLineColor,
              distanceFromArc: tickLineDistance,
              hide: tickLineHide
            }
          }
        }}
      />
    </div>
  );
};

export default GaugePreview;
