/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.hal.SolenoidJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.AllocationException;
import edu.wpi.first.wpilibj.util.CheckedAllocationException;

/**
 * Solenoid class for running high voltage Digital Output.
 *
 * <p>The Solenoid class is typically used for pneumatics solenoids, but could be used for any
 * device within the current spec of the PCM.
 */
public class Solenoid extends SolenoidBase implements LiveWindowSendable {

  private final int channel; // /< The channel to control.
  private long solenoidPort;

  /**
   * Common function to implement constructor behavior.
   */
  private synchronized void initSolenoid() {
    checkSolenoidModule(moduleNumber);
    checkSolenoidChannel(channel);

    try {
      allocated.allocate(moduleNumber * kSolenoidChannels + channel);
    } catch (CheckedAllocationException ex) {
      throw new AllocationException("Solenoid channel " + channel + " on module "
          + moduleNumber + " is already allocated");
    }

    long port = SolenoidJNI.getPortWithModule((byte) moduleNumber, (byte) channel);
    solenoidPort = SolenoidJNI.initializeSolenoidPort(port);

    LiveWindow.addActuator("Solenoid", moduleNumber, channel, this);
    UsageReporting.report(tResourceType.kResourceType_Solenoid, channel, moduleNumber);
  }

  /**
   * Constructor using the default PCM ID (0)
   *
   * @param channel The channel on the PCM to control (0..7).
   */
  public Solenoid(final int channel) {
    super(getDefaultSolenoidModule());
    this.channel = channel;
    initSolenoid();
  }

  /**
   * Constructor.
   *
   * @param moduleNumber The CAN ID of the PCM the solenoid is attached to.
   * @param channel      The channel on the PCM to control (0..7).
   */
  public Solenoid(final int moduleNumber, final int channel) {
    super(moduleNumber);
    this.channel = channel;
    initSolenoid();
  }

  /**
   * Destructor.
   */
  public synchronized void free() {
    allocated.free(moduleNumber * kSolenoidChannels + channel);
    SolenoidJNI.freeSolenoidPort(solenoidPort);
    solenoidPort = 0;
    super.free();
  }

  /**
   * Set the value of a solenoid.
   *
   * @param on Turn the solenoid output off or on.
   */
  public void set(boolean on) {
    byte value = (byte) (on ? 0xFF : 0x00);
    byte mask = (byte) (1 << channel);

    set(value, mask);
  }

  /**
   * Read the current value of the solenoid.
   *
   * @return The current value of the solenoid.
   */
  public boolean get() {
    int value = getAll() & (1 << channel);
    return (value != 0);
  }

  /**
   * Check if solenoid is blacklisted. If a solenoid is shorted, it is added to the blacklist and
   * disabled until power cycle, or until faults are cleared.
   *
   * @return If solenoid is disabled due to short.
   * @see #clearAllPCMStickyFaults()
   */
  public boolean isBlackListed() {
    int value = getPCMSolenoidBlackList() & (1 << channel);
    return (value != 0);
  }

  /*
   * Live Window code, only does anything if live window is activated.
   */
  public String getSmartDashboardType() {
    return "Solenoid";
  }

  private ITable table;
  private ITableListener tableListener;

  @Override
  public void initTable(ITable subtable) {
    table = subtable;
    updateTable();
  }

  @Override
  public ITable getTable() {
    return table;
  }

  @Override
  public void updateTable() {
    if (table != null) {
      table.putBoolean("Value", get());
    }
  }


  @Override
  public void startLiveWindowMode() {
    set(false); // Stop for safety
    tableListener = new ITableListener() {
      public void valueChanged(ITable itable, String key, Object value, boolean bln) {
        set(((Boolean) value).booleanValue());
      }
    };
    table.addTableListener("Value", tableListener, true);
  }

  @Override
  public void stopLiveWindowMode() {
    set(false); // Stop for safety
    // TODO: Broken, should only remove the listener from "Value" only.
    table.removeTableListener(tableListener);
  }
}
