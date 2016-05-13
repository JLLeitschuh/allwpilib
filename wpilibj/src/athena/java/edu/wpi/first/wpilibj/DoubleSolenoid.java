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
 * <p>The DoubleSolenoid class is typically used for pneumatics solenoids that have two positions
 * controlled by two separate channels.
 */
public class DoubleSolenoid extends SolenoidBase implements LiveWindowSendable {

  /**
   * Possible values for a DoubleSolenoid.
   */
  public enum Value {
    kOff,
    kForward,
    kReverse
  }

  private int m_forwardChannel; // /< The forward channel on the module to
  // control.
  private int m_reverseChannel; // /< The reverse channel on the module to
  // control.
  private byte m_forwardMask; // /< The mask for the forward channel.
  private byte m_reverseMask; // /< The mask for the reverse channel.

  /**
   * Common function to implement constructor behavior.
   */
  private synchronized void initSolenoid() {
    checkSolenoidModule(moduleNumber);
    checkSolenoidChannel(m_forwardChannel);
    checkSolenoidChannel(m_reverseChannel);

    try {
      allocated.allocate(moduleNumber * kSolenoidChannels + m_forwardChannel);
    } catch (CheckedAllocationException exception) {
      throw new AllocationException("Solenoid channel " + m_forwardChannel + " on module "
          + moduleNumber + " is already allocated");
    }
    try {
      allocated.allocate(moduleNumber * kSolenoidChannels + m_reverseChannel);
    } catch (CheckedAllocationException exception) {
      throw new AllocationException("Solenoid channel " + m_reverseChannel + " on module "
          + moduleNumber + " is already allocated");
    }
    m_forwardMask = (byte) (1 << m_forwardChannel);
    m_reverseMask = (byte) (1 << m_reverseChannel);

    UsageReporting.report(tResourceType.kResourceType_Solenoid, m_forwardChannel, moduleNumber);
    UsageReporting.report(tResourceType.kResourceType_Solenoid, m_reverseChannel, moduleNumber);
    LiveWindow.addActuator("DoubleSolenoid", moduleNumber, m_forwardChannel, this);
  }

  /**
   * Constructor. Uses the default PCM ID of 0 $
   *
   * @param forwardChannel The forward channel number on the PCM (0..7).
   * @param reverseChannel The reverse channel number on the PCM (0..7).
   */
  public DoubleSolenoid(final int forwardChannel, final int reverseChannel) {
    super(getDefaultSolenoidModule());
    this.m_forwardChannel = forwardChannel;
    this.m_reverseChannel = reverseChannel;
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
    this.m_forwardChannel = forwardChannel;
    this.m_reverseChannel = reverseChannel;
    initSolenoid();
  }

  /**
   * Destructor.
   */
  public synchronized void free() {
    allocated.free(moduleNumber * kSolenoidChannels + m_forwardChannel);
    allocated.free(moduleNumber * kSolenoidChannels + m_reverseChannel);
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
        rawValue = m_forwardMask;
        break;
      case kReverse:
        rawValue = m_reverseMask;
        break;
      default:
        throw new AssertionError("Illegal value: " + value);

    }

    set(rawValue, m_forwardMask | m_reverseMask);
  }

  /**
   * Read the current value of the solenoid.
   *
   * @return The current value of the solenoid.
   */
  public Value get() {
    byte value = getAll();

    if ((value & m_forwardMask) != 0) {
      return Value.kForward;
    }
    if ((value & m_reverseMask) != 0) {
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
    return ((blackList & m_forwardMask) != 0);
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
    return ((blackList & m_reverseMask) != 0);
  }

  /*
   * Live Window code, only does anything if live window is activated.
   */
  public String getSmartDashboardType() {
    return "Double Solenoid";
  }

  private ITable m_table;
  private ITableListener m_tableListener;

  @Override
  public void initTable(ITable subtable) {
    m_table = subtable;
    updateTable();
  }

  @Override
  public ITable getTable() {
    return m_table;
  }

  @Override
  public void updateTable() {
    if (m_table != null) {
      // TODO: this is bad
      m_table.putString("Value", (get() == Value.kForward ? "Forward"
          : (get() == Value.kReverse ? "Reverse" : "Off")));
    }
  }

  @Override
  public void startLiveWindowMode() {
    set(Value.kOff); // Stop for safety
    m_tableListener = new ITableListener() {
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
    m_table.addTableListener("Value", m_tableListener, true);
  }

  @Override
  public void stopLiveWindowMode() {
    set(Value.kOff); // Stop for safety
    // TODO: Broken, should only remove the listener from "Value" only.
    m_table.removeTableListener(m_tableListener);
  }
}
