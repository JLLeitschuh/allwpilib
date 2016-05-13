/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.AllocationException;
import edu.wpi.first.wpilibj.util.CheckedAllocationException;

/**
 * DoubleSolenoid class for running 2 channels of high voltage Digital Output.
 *
 * The DoubleSolenoid class is typically used for pneumatics solenoids that have two positions
 * controlled by two separate channels.
 */
public class DoubleSolenoid extends SolenoidBase implements LiveWindowSendable {

  /**
   * Possible values for a DoubleSolenoid
   */
  public enum Value {
    kOff,
    kForward,
    kReverse
  }

  private int forwardChannel; // /< The forward channel on the module to
  // control.
  private int reverseChannel; // /< The reverse channel on the module to
  // control.
  private byte forwardMask; // /< The mask for the forward channel.
  private byte reverseMask; // /< The mask for the reverse channel.

  /**
   * Common function to implement constructor behavior.
   */
  private synchronized void initSolenoid() {
    checkSolenoidModule(moduleNumber);
    checkSolenoidChannel(forwardChannel);
    checkSolenoidChannel(reverseChannel);

    try {
      allocated.allocate(moduleNumber * kSolenoidChannels + forwardChannel);
    } catch (CheckedAllocationException e) {
      throw new AllocationException("Solenoid channel " + forwardChannel + " on module "
          + moduleNumber + " is already allocated");
    }
    try {
      allocated.allocate(moduleNumber * kSolenoidChannels + reverseChannel);
    } catch (CheckedAllocationException e) {
      throw new AllocationException("Solenoid channel " + reverseChannel + " on module "
          + moduleNumber + " is already allocated");
    }
    forwardMask = (byte) (1 << forwardChannel);
    reverseMask = (byte) (1 << reverseChannel);

    UsageReporting.report(tResourceType.kResourceType_Solenoid, forwardChannel, moduleNumber);
    UsageReporting.report(tResourceType.kResourceType_Solenoid, reverseChannel, moduleNumber);
    LiveWindow.addActuator("DoubleSolenoid", moduleNumber, forwardChannel, this);
  }

  /**
   * Constructor. Uses the default PCM ID of 0 $
   *
   * @param forwardChannel The forward channel number on the PCM (0..7).
   * @param reverseChannel The reverse channel number on the PCM (0..7).
   */
  public DoubleSolenoid(final int forwardChannel, final int reverseChannel) {
    super(getDefaultSolenoidModule());
    this.forwardChannel = forwardChannel;
    this.reverseChannel = reverseChannel;
    initSolenoid();
  }

  /**
   * Constructor.
   *
   * @param moduleNumber   The module number of the solenoid module to use.
   * @param forwardChannel The forward channel on the module to control (0..7).
   * @param reverseChannel The reverse channel on the module to control (0..7).
   */
  public DoubleSolenoid(final int moduleNumber, final int forwardChannel, final int
      reverseChannel) {
    super(moduleNumber);
    this.forwardChannel = forwardChannel;
    this.reverseChannel = reverseChannel;
    initSolenoid();
  }

  /**
   * Destructor.
   */
  public synchronized void free() {
    allocated.free(moduleNumber * kSolenoidChannels + forwardChannel);
    allocated.free(moduleNumber * kSolenoidChannels + reverseChannel);
    super.free();
  }

  /**
   * Set the value of a solenoid.
   *
   * @param value The value to set (Off, Forward, Reverse)
   */
  public void set(final Value value) {
    byte rawValue = 0;

    switch (value) {
      case kOff:
        rawValue = 0x00;
        break;
      case kForward:
        rawValue = forwardMask;
        break;
      case kReverse:
        rawValue = reverseMask;
        break;
    }

    set(rawValue, forwardMask | reverseMask);
  }

  /**
   * Read the current value of the solenoid.
   *
   * @return The current value of the solenoid.
   */
  public Value get() {
    byte value = getAll();

    if ((value & forwardMask) != 0) {
      return Value.kForward;
    }
    if ((value & reverseMask) != 0) {
      return Value.kReverse;
    }
    return Value.kOff;
  }

  /**
   * Check if the forward solenoid is blacklisted. If a solenoid is shorted, it is added to the
   * blacklist and disabled until power cycle, or until faults are cleared.
   *
   * @return If solenoid is disabled due to short.
   * @see #clearAllPCMStickyFaults()
   */
  public boolean isFwdSolenoidBlackListed() {
    int blackList = getPCMSolenoidBlackList();
    return ((blackList & forwardMask) != 0);
  }

  /**
   * Check if the reverse solenoid is blacklisted. If a solenoid is shorted, it is added to the
   * blacklist and disabled until power cycle, or until faults are cleared.
   *
   * @return If solenoid is disabled due to short.
   * @see #clearAllPCMStickyFaults()
   */
  public boolean isRevSolenoidBlackListed() {
    int blackList = getPCMSolenoidBlackList();
    return ((blackList & reverseMask) != 0);
  }

  /*
   * Live Window code, only does anything if live window is activated.
   */
  public String getSmartDashboardType() {
    return "Double Solenoid";
  }

  private ITable table;
  private ITableListener table_listener;

  /**
   * {@inheritDoc}
   */
  public void initTable(ITable subtable) {
    table = subtable;
    updateTable();
  }

  /**
   * {@inheritDoc}
   */
  public ITable getTable() {
    return table;
  }

  /**
   * {@inheritDoc}
   */
  public void updateTable() {
    if (table != null) {
      // TODO: this is bad
      table.putString("Value", (get() == Value.kForward ? "Forward"
          : (get() == Value.kReverse ? "Reverse" : "Off")));
    }
  }

  /**
   * {@inheritDoc}
   */
  public void startLiveWindowMode() {
    set(Value.kOff); // Stop for safety
    table_listener = new ITableListener() {
      public void valueChanged(ITable itable, String key, Object value, boolean bln) {
        // TODO: this is bad also
        if (value.toString().equals("Reverse")) {
          set(Value.kReverse);
        } else if (value.toString().equals("Forward")) {
          set(Value.kForward);
        } else {
          set(Value.kOff);
        }
      }
    };
    table.addTableListener("Value", table_listener, true);
  }

  /**
   * {@inheritDoc}
   */
  public void stopLiveWindowMode() {
    set(Value.kOff); // Stop for safety
    // TODO: Broken, should only remove the listener from "Value" only.
    table.removeTableListener(table_listener);
  }
}
