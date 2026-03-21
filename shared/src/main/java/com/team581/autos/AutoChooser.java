package com.team581.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser<T extends Enum<T> & AutoSelectionBase> {
  private final SendableChooser<T> chooser = new SendableChooser<>();
  private final T defaultSelection;

  public AutoChooser(T[] selections, T defaultSelection) {
    this.defaultSelection = defaultSelection;

    SmartDashboard.putData("Autos/SelectedAuto", chooser);

    for (T selection : selections) {
      chooser.addOption(selection.toString(), selection);
    }

    chooser.setDefaultOption(defaultSelection.toString(), defaultSelection);
  }

  public T getSelectedAuto() {
    var selected = chooser.getSelected();

    if (selected == null) {
      return defaultSelection;
    }

    return selected;
  }
}
