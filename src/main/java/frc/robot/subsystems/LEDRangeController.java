package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import org.littletonrobotics.junction.Logger;

public class LEDRangeController {

  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);

  private final int startIndex;
  private final int endIndex;
  private final double maxError;
  private final boolean invert;
  private final String logPath;

  LEDRangeController(
      int startIndex, int endIndex, double maxError, boolean invert, String logPath) {
    this.startIndex = startIndex;
    this.endIndex = endIndex;
    this.maxError = maxError;
    this.invert = invert;
    this.logPath = logPath;
  }

  public void disable(CANdle candle) {
    setLEDs(candle, 0);
  }

  public void update(CANdle candle, double error) {
    int availableLEDS = Math.abs(startIndex - endIndex) + 1;
    int litLEDcount = (int) Math.round((error / maxError) * availableLEDS);
    if (litLEDcount < 0) {
      litLEDcount = 0;
    }

    if (invert) {
      litLEDcount = availableLEDS - litLEDcount;
    }
    setLEDs(candle, litLEDcount);
  }

  private void setLEDs(CANdle candle, int count) {
    int rangeStartIndex = Math.min(startIndex, endIndex);
    int rangeEndIndex = Math.max(startIndex, endIndex);
    Logger.recordOutput(logPath, count);
    candle.setControl(new SolidColor(rangeStartIndex, rangeEndIndex).withColor(OFF));
    if (count <= 0) {
      candle.setControl(new SolidColor(rangeStartIndex, rangeEndIndex).withColor(OFF));
    } else if (count > getAvailableLEDs()) {
      candle.setControl(new SolidColor(rangeStartIndex, rangeEndIndex).withColor(RED));

    } else {

      int indexStep = Integer.compare(endIndex, startIndex);
      int lastLitIndex = startIndex + (indexStep * (count - 1));
      int litStartIndex = Math.min(startIndex, lastLitIndex);
      int litEndIndex = Math.max(startIndex, lastLitIndex);
      candle.setControl(new SolidColor(litStartIndex, litEndIndex).withColor(GREEN));
    }
  }

  private int getAvailableLEDs() {
    return Math.abs(endIndex - startIndex) + 1;
  }
}
