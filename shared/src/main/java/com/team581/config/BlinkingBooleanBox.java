package com.team581.config;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingBooleanBox {
  private final String name;
  private final boolean blinkWhenValueIsTrue;
  private final boolean forceNt;
  private final Timer timer = new Timer();
  private boolean currentValue;

  public BlinkingBooleanBox(String name, boolean blinkWhen) {
    this(name, blinkWhen, false);
  }

  public BlinkingBooleanBox(String name, boolean blinkWhen, boolean forceNt) {
    this.name = name;
    this.blinkWhenValueIsTrue = blinkWhen;
    this.forceNt = forceNt;
    this.timer.start();
  }

  public Color get() {
    boolean shouldBlink = Math.floor(timer.get() / 0.1) % 2 == 0;

    if (blinkWhenValueIsTrue) {
      if (currentValue) { // Blinks when true: green/black
        return shouldBlink ? Color.kLimeGreen : Color.kBlack;
      } else { // Solid when false: green
        return Color.kLimeGreen;
      }
    } else {
      if (currentValue) { // Solid when true: green
        return Color.kLimeGreen;
      } else { // Blinks when false: red/black
        return shouldBlink ? Color.kRed : Color.kBlack;
      }
    }
  }

  public void update(boolean value) {
    this.currentValue = value;
    if (forceNt) {
      DogLog.forceNt.log(name, get().toHexString());
    } else {
      DogLog.log(name, get().toHexString());
    }
  }
}
