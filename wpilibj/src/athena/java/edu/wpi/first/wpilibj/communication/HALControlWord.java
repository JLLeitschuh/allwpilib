/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.communication;

/**
 * A wrapper for the HALControlWord bitfield.
 */
public class HALControlWord {
  private final boolean enabled;
  private final boolean autonomous;
  private final boolean test;
  private final boolean emergencyStop;
  private final boolean fmsAttached;
  private final boolean dsAttached;

  protected HALControlWord(boolean enabled, boolean autonomous, boolean test, boolean emergencyStop,
                           boolean fmsAttached, boolean dsAttached) {
    this.enabled = enabled;
    this.autonomous = autonomous;
    this.test = test;
    this.emergencyStop = emergencyStop;
    this.fmsAttached = fmsAttached;
    this.dsAttached = dsAttached;
  }

  public boolean getEnabled() {
    return enabled;
  }

  public boolean getAutonomous() {
    return autonomous;
  }

  public boolean getTest() {
    return test;
  }

  public boolean getEStop() {
    return emergencyStop;
  }

  public boolean getFMSAttached() {
    return fmsAttached;
  }

  public boolean getDSAttached() {
    return dsAttached;
  }


}
